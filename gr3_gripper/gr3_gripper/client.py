"""One command from an Ubuntu inference process; depends only on ROS2 + stdlib."""
import argparse
import json
import math
import time
import uuid


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--closure", type=float, required=True)
    parser.add_argument("--namespace", default="/gr3/right_gripper")
    parser.add_argument("--timeout", type=float, default=10)
    args, ros_args = parser.parse_known_args()
    if not math.isfinite(args.closure) or not 0 <= args.closure <= 1:
        parser.error("closure must be in [0, 1]")
    if not math.isfinite(args.timeout) or not 0 < args.timeout <= 30:
        parser.error("timeout must be in (0, 30]")
    import rclpy
    from std_msgs.msg import String
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from .protocol import BACKEND

    rclpy.init(args=ros_args)
    node = rclpy.create_node("gripper_client_" + uuid.uuid4().hex[:8], namespace=args.namespace)
    command_id = uuid.uuid4().hex
    received = {"session": None, "selected": None, "result": None, "reset": False}

    def callback(msg):
        try:
            value = json.loads(msg.data)
            if not isinstance(value, dict) or value.get("backend") != BACKEND:
                return
            session = value.get("server_id")
            if not isinstance(session, str):
                return
            if received["selected"] and session != received["selected"]:
                received["reset"] = True
            if value.get("state") == "READY" and value.get("observation", {}).get("enabled"):
                received["session"] = session
            if value.get("id") == command_id and session == received["selected"]:
                received["result"] = value
        except (ValueError, TypeError, AttributeError):
            pass

    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.VOLATILE)
    subscriber = node.create_subscription(String, "status", callback, qos)
    publisher = node.create_publisher(String, "command", qos)
    try:
        deadline = time.monotonic() + args.timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            if received["session"] and publisher.get_subscription_count():
                break
        else:
            print(json.dumps({"state": "DISCOVERY_TIMEOUT"}))
            return 2
        received["selected"] = received["session"]
        command = dict(version=1, id=command_id, server_id=received["selected"],
                       backend=BACKEND, closure=args.closure, expires_at=time.time() + args.timeout)
        msg = String()
        msg.data = json.dumps(command, allow_nan=False)
        publisher.publish(msg)  # Send once; never retry an ambiguous outcome.
        deadline = time.monotonic() + args.timeout + 0.5
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            if received["reset"]:
                print(json.dumps({"id": command_id, "state": "BRIDGE_CHANGED", "robot_task_success": None}))
                return 2
            result = received["result"]
            if result and result["state"] not in ("ACCEPTED", "RAMPING"):
                print(json.dumps(result))
                return 0 if result["state"] == "RAMP_COMPLETE" else 1
        print(json.dumps({"id": command_id, "state": "RESPONSE_TIMEOUT", "robot_task_success": None,
                          "detail": "Outcome unknown; no automatic retry"}))
        return 2
    finally:
        node.destroy_subscription(subscriber)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
