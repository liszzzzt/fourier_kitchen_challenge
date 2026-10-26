"""Embed in the process that owns MjModel/MjData; call step on its physics thread."""
import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String

from .protocol import GripperSession


class GripperROSBridge(Node):
    def __init__(self, controller, namespace="/gr3/right_gripper"):
        super().__init__("gripper_bridge", namespace=namespace)
        self.session = GripperSession(controller)
        self.controller = controller
        self.previous_sim_time = None
        self.next_status = 0.0
        self.faulted = False
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
        self.publisher = self.create_publisher(String, "status", qos)
        self.subscription = self.create_subscription(String, "command", self.on_command, qos)

    def publish(self, value):
        msg = String()
        msg.data = json.dumps(value, allow_nan=False)
        self.publisher.publish(msg)

    def on_command(self, msg):
        if self.faulted:
            # Preserve the request id so the caller sees a prompt rejection.
            try:
                value = json.loads(msg.data)
                command_id = value.get("id") if isinstance(value, dict) else None
                if not isinstance(command_id, str):
                    command_id = None
            except ValueError:
                command_id = None
            self.publish(self.session.status(command_id, "REJECTED", detail="Bridge faulted; restart required"))
            return
        self.publish(self.session.request(msg.data))

    def step(self, data):
        """Replace the old controller.update(data) call, immediately before mj_step.

        Invoke even while paused if timeout/status processing is required. Do not
        spin this node in another thread: only this method may access MjData.
        """
        now = float(data.time)
        if self.previous_sim_time is not None and now < self.previous_sim_time:
            self.publish(self.session.reset())
            # Preserve the original controller's reset-to-open behavior.
            self.controller.update(data)
        self.previous_sim_time = now
        rclpy.spin_once(self, timeout_sec=0.0)
        if self.faulted:
            return
        try:
            self.controller.update(data)
            status = self.session.sample(data)
            wall = time.monotonic()
            if status["state"] not in ("READY", "RAMPING") or wall >= self.next_status:
                self.publish(status)
                self.next_status = wall + 0.05
        except Exception as exc:
            self.faulted = True
            self.publish(self.session.fail(exc))
