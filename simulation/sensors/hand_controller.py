"""ROS2 open/close control of the viewer's live Kinova-derived right gripper.

Call bind(model, data) after a successful load, and update(running) under the
viewer's simulation lock after body control. This module never calls mj_step,
loads a second model, or changes the process ROS/Aurora domain configuration.
"""
import json
from functools import wraps
import time
import warnings

import mujoco
import numpy as np

ACTUATOR = 'rh_2f85_actuator'
COMMAND_TOPIC = 'hand_command'
STATUS_TOPIC = 'hand_status'
PREFIX = 'rh_2f85_'


def with_hand_controller(physics_loop):
    """Own one hand controller for the physics-loop call, closing it on all exits.

    The decorated loop receives an explicit keyword-only ``hand`` argument.
    Model binding and control updates remain visible at their viewer call sites.
    This wrapper neither catches loop errors nor manages camera/LCM resources.
    """
    @wraps(physics_loop)
    def run(*args, **kwargs):
        with HandController() as hand:
            return physics_loop(*args, hand=hand, **kwargs)
    return run


_body_count_cache = None


def body_motor_count(model):
    """Keep the original 31 body channels when the optional right hand exists."""
    global _body_count_cache
    if _body_count_cache is not None and _body_count_cache[0] is model:
        return _body_count_cache[1]
    hand_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'rh_2f85_actuator')
    count = model.nu
    if hand_id >= 0:
        if hand_id != 31 or model.nu != 32 or model.nsensor < 62:
            raise ValueError('Expected 31 original body motors followed by one hand actuator')
        count = 31
        for i in range(count):
            joint_id = model.actuator_trnid[i, 0]
            if model.actuator_trntype[i] != mujoco.mjtTrn.mjTRN_JOINT:
                raise ValueError('Original body actuator transmission changed')
            for sensor_id, sensor_type in ((i, mujoco.mjtSensor.mjSENS_JOINTPOS),
                                           (i+count, mujoco.mjtSensor.mjSENS_JOINTVEL)):
                if (model.sensor_type[sensor_id] != sensor_type or
                        model.sensor_objid[sensor_id] != joint_id or
                        model.sensor_dim[sensor_id] != 1 or model.sensor_adr[sensor_id] != sensor_id):
                    raise ValueError('Original body feedback layout changed')
    _body_count_cache = (model, count)
    return count


class HandDrive:
    """Physics-only controller; all methods run on the owning physics thread."""

    def __init__(self, model, data):
        self.model, self.data = model, data
        self.aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, ACTUATOR)
        if self.aid < 0:
            raise ValueError('Right-hand actuator is absent')
        joints = [('finger_joint', 1), ('left_inner_finger_joint', -1),
                  ('left_inner_knuckle_joint', 1), ('right_outer_knuckle_joint', 1),
                  ('right_inner_finger_joint', -1), ('right_inner_knuckle_joint', 1)]
        self.qadr, self.vadr, signs = [], [], []
        for name, sign in joints:
            joint = model.joint(PREFIX + name)
            if int(model.jnt_type[joint.id]) != int(mujoco.mjtJoint.mjJNT_HINGE):
                raise ValueError('Expected hinge joint: ' + name)
            self.qadr.append(int(model.jnt_qposadr[joint.id]))
            self.vadr.append(int(model.jnt_dofadr[joint.id]))
            signs.append(sign)
        self.signs = np.array(signs)
        if (not model.actuator_ctrllimited[self.aid] or
                not model.actuator_forcelimited[self.aid] or
                not np.allclose(model.actuator_ctrlrange[self.aid], [0, 0.8])):
            raise ValueError('Unexpected right-hand actuator limits')
        primary = model.joint(PREFIX + 'finger_joint').id
        if (model.actuator_trntype[self.aid] != mujoco.mjtTrn.mjTRN_JOINT or
                model.actuator_trnid[self.aid, 0] != primary):
            raise ValueError('Right-hand actuator is not bound to finger_joint')
        # Hold the initial measured pose until an explicit command arrives.
        if not np.isfinite(data.qpos[self.qadr]).all():
            raise ValueError('Non-finite initial gripper pose')
        self.commanded = float(np.clip(data.qpos[self.qadr[0]], 0, 0.8))
        self.target = self.commanded
        self.previous_time = float(data.time)
        self.settled_since = None
        self.active = False
        self.deadline = None

    def start(self, action):
        if action not in ('open', 'close'):
            raise ValueError('action must be open or close')
        self.target = 0.0 if action == 'open' else 0.8
        self.deadline = time.monotonic() + 30.0
        self.settled_since = None
        self.active = True

    def update(self, running=True):
        now = float(self.data.time)
        q = self.data.qpos[self.qadr]
        v = self.data.qvel[self.vadr]
        if not np.isfinite(now) or not np.isfinite(q).all() or not np.isfinite(v).all():
            raise ValueError('Non-finite gripper feedback')
        if now < self.previous_time:
            had_command = self.active
            self.commanded = self.target = float(np.clip(q[0], 0, 0.8))
            self.active = False
            self.settled_since = None
            result = 'reset' if had_command else None
        else:
            result = None
            if self.active and time.monotonic() >= self.deadline:
                self.target = self.commanded
                self.active = False
                result = 'timeout'
            elif running:
                delta = 0.4 * max(0.0, now - self.previous_time)
                self.commanded += float(np.clip(self.target-self.commanded, -delta, delta))
                at_target = (abs(self.commanded-self.target) < 1e-9 and
                             np.max(np.abs(q-self.signs*self.target)) <= 0.02 and
                             np.max(np.abs(v)) <= 0.03)
                if self.active and at_target:
                    if self.settled_since is None:
                        self.settled_since = now
                    if now-self.settled_since >= 0.1:
                        self.active = False
                        result = 'reached'
                else:
                    self.settled_since = None
            else:
                self.settled_since = None
        self.previous_time = now
        # Never touch body actuators or qpos/qvel directly.
        self.data.ctrl[self.aid] = self.commanded
        return result


class HandController:
    """Optional module with a private ROS context and deterministic cleanup."""

    def __init__(self):
        self.drive = None
        self.context = self.node = self.executor = None
        self.publisher = None
        self.active_id = None
        self.pending = None
        self.last_request = self.last_status = None
        self.faulted = False

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, traceback):
        self.close()

    def bind(self, model, data):
        self.close()
        self.active_id = self.pending = self.last_request = self.last_status = None
        self.faulted = False
        if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, ACTUATOR) < 0:
            print('HandController: model has no right gripper; disabled.', flush=True)
            return
        try:
            drive = HandDrive(model, data)
            # Optional ROS imports stay here so a handless model needs no hand ROS node.
            import rclpy
            from rclpy.context import Context
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.node import Node
            from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
            from std_msgs.msg import String
            self.context = Context()
            rclpy.init(args=[], context=self.context)
            self.node = Node('gr3_right_hand', context=self.context)
            qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.VOLATILE)
            self.publisher = self.node.create_publisher(String, STATUS_TOPIC, qos)
            self.node.create_subscription(String, COMMAND_TOPIC, self._receive, qos)
            self.executor = SingleThreadedExecutor(context=self.context)
            self.executor.add_node(self.node)
            self.drive = drive
            print('HandController: ready on hand_command / hand_status.', flush=True)
        except Exception as error:
            warnings.warn(f'HandController disabled: {type(error).__name__}: {error}')
            self.close()

    def _publish(self, command_id, state, reason=None):
        from std_msgs.msg import String
        payload = dict(id=command_id, state=state)
        if reason:
            payload['reason'] = reason
        message = String()
        message.data = json.dumps(payload, allow_nan=False)
        self.publisher.publish(message)
        return payload

    def _receive(self, message):
        """Receive only; physics work is performed by update, on the same thread."""
        command_id = None
        try:
            if len(message.data) > 1024:
                raise ValueError('Command is too long')
            request = json.loads(message.data)
            if not isinstance(request, dict):
                raise ValueError('Expected a JSON object')
            command_id = request.get('id')
            if type(command_id) not in (int, str) or (isinstance(command_id, str) and not command_id):
                command_id = None
                raise ValueError('id must be an integer or nonempty string')
            action = request.get('action')
            if action not in ('open', 'close'):
                raise ValueError('action must be open or close')
            if self.faulted:
                raise ValueError('Hand controller faulted; reload required')
            if self.last_request and command_id == self.last_request['id']:
                if action != self.last_request['action']:
                    raise ValueError('id was reused with a different action')
                self._publish(self.last_status['id'], self.last_status['state'], self.last_status.get('reason'))
                return
            if self.active_id is not None or self.pending is not None:
                raise ValueError('Hand is busy')
            self.pending = dict(id=command_id, action=action)
            self.last_request = self.pending.copy()
            self.last_status = self._publish(command_id, 'RUNNING')
        except (ValueError, TypeError) as error:
            self._publish(command_id, 'FAIL', str(error))

    def update(self, running=True):
        if self.drive is None:
            return
        if self.faulted:
            self.drive.data.ctrl[self.drive.aid] = self.drive.commanded
            return
        try:
            # Process reset/timeout before accepting a new request into this state.
            outcome = self.drive.update(running)
            if outcome and self.active_id is not None:
                self.last_status = self._publish(self.active_id, 'SUCCESS' if outcome == 'reached' else 'FAIL',
                                                  None if outcome == 'reached' else outcome)
                self.active_id = None
            self.executor.spin_once(timeout_sec=0.0)
            if self.pending is not None:
                self.active_id = self.pending['id']
                self.drive.start(self.pending['action'])
                self.pending = None
        except Exception as error:
            self.faulted = True
            warnings.warn(f'HandController fault: {type(error).__name__}: {error}')
            if self.active_id is not None:
                try:
                    self.last_status = self._publish(self.active_id, 'FAIL', str(error))
                except Exception as publish_error:
                    warnings.warn(f'Hand status publish failed: {publish_error}')
            self.active_id = self.pending = None

    def close(self):
        # Called by the physics thread, including reload and exception unwinding.
        if self.publisher is not None and self.active_id is not None:
            try:
                self._publish(self.active_id, 'FAIL', 'controller closed or model reloaded')
            except Exception as error:
                warnings.warn(f'Hand close status failed: {error}')
        if self.executor is not None:
            try:
                self.executor.shutdown(timeout_sec=0.0)
            except Exception as error:
                warnings.warn(f'Hand executor cleanup failed: {error}')
        if self.node is not None:
            try:
                self.node.destroy_node()
            except Exception as error:
                warnings.warn(f'Hand node cleanup failed: {error}')
        if self.context is not None:
            try:
                if self.context.ok():
                    self.context.shutdown()
            except Exception as error:
                warnings.warn(f'Hand ROS context cleanup failed: {error}')
        self.context = self.node = self.executor = self.publisher = self.drive = None
        self.active_id = self.pending = None
