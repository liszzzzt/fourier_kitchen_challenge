"""Read-only checks; never starts Aurora or writes actuator commands."""
import argparse
import importlib
import json
import os
import platform
import sys


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--role", choices=["robot", "compute"], required=True)
    parser.add_argument("--model", help="Optional integrated MuJoCo XML/MJB, robot role only")
    args = parser.parse_args()
    checks = []

    def check(name, actual, expected):
        checks.append(dict(name=name, actual=actual, expected=expected, passed=actual == expected))

    check("python", platform.python_version(), "3.10.12" if args.role == "robot" else "3.10.22")
    check("ROS_DOMAIN_ID", os.getenv("ROS_DOMAIN_ID"), "6")
    check("ROS_DISTRO", os.getenv("ROS_DISTRO"), "humble")
    packages = {"numpy": "1.21.5", "mujoco": "2.3.6"} if args.role == "robot" else {}
    for name, expected in packages.items():
        try:
            check(name, importlib.import_module(name).__version__, expected)
        except Exception as exc:
            check(name, str(exc), expected)
    try:
        import rclpy
        from std_msgs.msg import String
        from rclpy.type_support import check_for_type_support
        check_for_type_support(String)
        check("ROS Python/type support", True, True)
    except Exception as exc:
        check("ROS Python/type support", str(exc), True)
    model_info = None
    if args.model:
        if args.role != "robot":
            parser.error("--model is for the robot role")
        try:
            import mujoco
            from .controller import ACTUATOR, GripperController
            loader = mujoco.MjModel.from_binary_path if args.model.endswith(".mjb") else mujoco.MjModel.from_xml_path
            model = loader(args.model)
            controller = GripperController(model)
            check("gripper actuator", controller.enabled, True)
            if controller.enabled:
                model_info = dict(actuator=ACTUATOR, index=int(controller.aid),
                                  ctrlrange=model.actuator_ctrlrange[controller.aid].tolist(),
                                  forcerange=model.actuator_forcerange[controller.aid].tolist(),
                                  total_actuators=model.nu)
        except Exception as exc:
            check("model", str(exc), "compatible integrated gripper model")
    result = dict(passed=all(c["passed"] for c in checks), role=args.role,
                  interpreter=sys.executable, checks=checks, model=model_info,
                  commands_sent=0, cross_container_ros_verified=False, robot_task_success=None)
    print(json.dumps(result, indent=2))
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
