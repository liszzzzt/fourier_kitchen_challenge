"""Run a synthetic MuJoCo fixture for ROS2 transport testing, without Aurora."""
import argparse
from pathlib import Path
import time


def fixture_path():
    path = Path(__file__).resolve().parents[1] / "models/smoke_fixture.xml"
    if path.exists():
        return path
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory("gr3_gripper")) / "models/smoke_fixture.xml"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--namespace", default="/gr3/right_gripper")
    args, ros_args = parser.parse_known_args()
    import mujoco
    import rclpy
    from .controller import GripperController
    from .ros_bridge import GripperROSBridge

    model = mujoco.MjModel.from_xml_path(str(fixture_path()))
    data = mujoco.MjData(model)
    rclpy.init(args=ros_args)
    bridge = None
    try:
        bridge = GripperROSBridge(GripperController(model), namespace=args.namespace)
        bridge.get_logger().info("SYNTHETIC FIXTURE: tests transport/ramping only; no grasp or robot")
        while rclpy.ok():
            start = time.monotonic()
            bridge.step(data)
            mujoco.mj_step(model, data)
            time.sleep(max(0.0, float(model.opt.timestep) - (time.monotonic() - start)))
    except KeyboardInterrupt:
        pass
    finally:
        if bridge:
            bridge.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
