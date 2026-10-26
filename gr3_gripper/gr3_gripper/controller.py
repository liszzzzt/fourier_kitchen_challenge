"""Simulation-only Robotiq control, isolated from Aurora's 31 motor channels.

closure=0 opens, closure=1 closes. This is an actuator target, not a contact
force command or an assertion that an object was grasped. Optional local JSON
commands: {"sequence": 1, "closure": 0.5}; write atomically after viewer startup.
"""
import json
import math
import threading
import time
from pathlib import Path

import mujoco
import numpy as np

CORE_CHANNELS = 31
PREFIX = 'rq_right_'
ACTUATOR = PREFIX + 'fingers_actuator'
CORE_JOINTS = [
    *[f'{s}_{j}_joint' for s in ('left', 'right') for j in
      ('hip_pitch', 'hip_roll', 'hip_yaw', 'knee_pitch', 'ankle_pitch', 'ankle_roll')],
    'waist_yaw_joint', 'waist_roll_joint', 'waist_pitch_joint', 'head_yaw_joint', 'head_pitch_joint',
    *[f'{s}_{j}_joint' for s in ('left', 'right') for j in
      ('shoulder_pitch', 'shoulder_roll', 'shoulder_yaw', 'elbow_pitch',
       'wrist_yaw', 'wrist_pitch', 'wrist_roll')]]


class CoreLayout:
    def __init__(self, model):
        self.model = model
        self.actuators = []
        self.qadr, self.vadr = [], []
        for name in CORE_JOINTS:
            jid = model.joint(name).id
            aids = [i for i in range(model.nu) if
                    model.actuator_trntype[i] == mujoco.mjtTrn.mjTRN_JOINT and
                    model.actuator_trnid[i, 0] == jid]
            if len(aids) != 1:
                raise ValueError('Expected one Core actuator for '+name)
            self.actuators.append(aids[0])
            for typ, addresses in ((mujoco.mjtSensor.mjSENS_JOINTPOS, self.qadr),
                                   (mujoco.mjtSensor.mjSENS_JOINTVEL, self.vadr)):
                sensors = [i for i in range(model.nsensor) if model.sensor_type[i] == typ
                           and model.sensor_objid[i] == jid and model.sensor_dim[i] == 1]
                if len(sensors) != 1:
                    raise ValueError('Expected one joint feedback sensor for '+name)
                addresses.append(int(model.sensor_adr[sensors[0]]))
        if self.actuators != list(range(CORE_CHANNELS)):
            raise ValueError('Original Core actuator order changed')
        self.qadr, self.vadr = np.array(self.qadr), np.array(self.vadr)
        self.base = {}
        for field, name, dim in [('acc', 'baselink-baseAcc', 3), ('omega', 'baselink-gyro', 3),
                                 ('quat', 'baselink-quat', 4), ('lf_touch', 'lf-touch', 1),
                                 ('rf_touch', 'rf-touch', 1), ('pos', 'baselink-position', 3),
                                 ('vel', 'baselink-velocity', 3)]:
            sensor = model.sensor(name)
            if int(sensor.dim[0]) != dim:
                raise ValueError('Unexpected sensor dimension: '+name)
            start = int(sensor.adr[0])
            self.base[field] = slice(start, start+dim)

    def feedback(self, data):
        fields = {'q': data.sensordata[self.qadr].copy(),
                  'qDot': data.sensordata[self.vadr].copy()}
        fields.update({name: data.sensordata[adr].copy() for name, adr in self.base.items()})
        return fields

    def apply(self, data, command):
        # Never clear or overwrite the independent gripper actuator.
        data.ctrl[self.actuators] = 0
        if command is None:
            return False
        try:
            if command.motorNum != CORE_CHANNELS or command.started not in (0, 1):
                return False
            arrays = [np.asarray(getattr(command, k), dtype=float) for k in
                      ('qCmd', 'qDotCmd', 'torqCmd', 'q_kp', 'q_kd')]
            if any(a.shape != (CORE_CHANNELS,) or not np.isfinite(a).all() for a in arrays):
                return False
            if command.started == 0:
                return False
            q, v, torque, kp, kd = arrays
            if np.any(kp < 0) or np.any(kd < 0):
                return False
            with np.errstate(over='ignore', invalid='ignore'):
                output = (kp*(q-data.sensordata[self.qadr]) +
                          kd*(v-data.sensordata[self.vadr]) + torque)
            if not np.isfinite(output).all():
                return False
            data.ctrl[self.actuators] = output
            return True
        except (AttributeError, TypeError, ValueError):
            return False


_layout = None


def core_layout(model):
    global _layout
    if _layout is None or _layout.model is not model:
        _layout = CoreLayout(model)
    return _layout


class GripperController:
    def __init__(self, model, command_file=None, closure_rate=0.5):
        self.model = model
        self.aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, ACTUATOR)
        self.enabled = self.aid >= 0
        self.lock = threading.Lock()
        self.target = self.current = 0.0
        self.last_sim_time = None
        self.rate = float(closure_rate)
        if not math.isfinite(self.rate) or self.rate <= 0:
            raise ValueError('closure_rate must be positive and finite')
        self.path = Path(command_file) if command_file else None
        self.sequence = -1
        self.next_poll = 0.0
        self.last_error = None
        if self.enabled:
            if model.nu >= CORE_CHANNELS and self.aid < CORE_CHANNELS:
                raise ValueError('Robotiq actuator overlaps the 31 Core channels')
            if not np.allclose(model.actuator_ctrlrange[self.aid], [0, 255]):
                raise ValueError('Unexpected Robotiq control range')
            if not model.actuator_ctrllimited[self.aid] or not model.actuator_forcelimited[self.aid]:
                raise ValueError('Robotiq actuator limits must remain enabled')
            self.driver_qadr = [int(model.joint(PREFIX+s+'_driver_joint').qposadr[0])
                                for s in ('left', 'right')]
        # Do not replay an old command file on startup/model reload.
        if self.path and self.path.exists():
            try:
                seq, _ = self.read_command()
                self.sequence = seq
            except (OSError, ValueError, KeyError, TypeError):
                pass

    @staticmethod
    def validate_closure(value):
        if isinstance(value, bool):
            raise ValueError('closure must be a number between 0 and 1')
        value = float(value)
        if not math.isfinite(value) or not 0 <= value <= 1:
            raise ValueError('closure must be finite and between 0 and 1')
        return value

    def set_target(self, closure):
        value = self.validate_closure(closure)
        if not self.enabled:
            raise ValueError('No Robotiq actuator in this model')
        with self.lock:
            self.target = value

    def read_command(self):
        if self.path.stat().st_size > 4096:
            raise ValueError('Gripper command file exceeds 4096 bytes')
        obj = json.loads(self.path.read_text())
        if not isinstance(obj, dict):
            raise ValueError('Gripper command must be an object')
        seq = obj['sequence']
        if isinstance(seq, bool) or not isinstance(seq, int) or seq < 0:
            raise ValueError('sequence must be a nonnegative integer')
        return seq, self.validate_closure(obj['closure'])

    def poll(self):
        now = time.monotonic()
        if not self.path or now < self.next_poll:
            return
        self.next_poll = now + 0.05
        try:
            seq, closure = self.read_command()
            if seq > self.sequence:
                self.set_target(closure)
                self.sequence = seq
                self.last_error = None
        except FileNotFoundError:
            pass
        except (OSError, ValueError, KeyError, TypeError) as exc:
            # Hold the last valid target, do not execute malformed commands.
            self.last_error = str(exc)

    def update(self, data):
        if not self.enabled:
            return
        self.poll()
        now = float(data.time)
        if not math.isfinite(now):
            raise ValueError('Simulation time must be finite')
        with self.lock:
            if self.last_sim_time is not None and now < self.last_sim_time:
                self.target = self.current = 0.0
            dt = 0.0 if self.last_sim_time is None else max(0.0, now-self.last_sim_time)
            self.current += float(np.clip(self.target-self.current, -self.rate*dt, self.rate*dt))
            self.last_sim_time = now
            data.ctrl[self.aid] = 255.0*self.current

    def observation(self, data):
        if not self.enabled:
            return {'enabled': False}
        return dict(enabled=True, target_closure=self.target, commanded_closure=self.current,
                    actuator_control=float(data.ctrl[self.aid]),
                    actuator_force=float(data.actuator_force[self.aid]),
                    driver_positions=[float(data.qpos[adr]) for adr in self.driver_qadr],
                    last_command_error=self.last_error,
                    note='Actuator force is not measured fingertip normal force.')
