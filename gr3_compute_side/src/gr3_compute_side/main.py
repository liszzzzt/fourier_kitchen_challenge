import rclpy
from gr3_compute_side.high_level_planner.center_control import (
    CenterControl,
    GR3ComputeSide
)
from gr3_compute_side.remote_control.remote_control import (
    RemoteControl,
    read_key,
)
from gr3_compute_side.vision.rgbd_viewer import RGBDCameraViewer


def main():
    rclpy.init()

    compute_node = GR3ComputeSide()
    rgbd_viewer = None
    center_control = CenterControl(compute_node)
    remote_control = RemoteControl(compute_node)

    try:
        control_key = read_key("Press 1 to remote control; Press 2 to execute schedule: ")
        if control_key != "1" and control_key != "2":
            print("Invalid mode")
            return
        vision_key = read_key("Press 3 to enable display; Press 4 to disable display: ")
        if vision_key != "3" and vision_key != "4":
            print("Invalid mode")
            return

        if vision_key == "3":
            rgbd_viewer = RGBDCameraViewer()
            rgbd_viewer.start()
            rgbd_viewer.enable_display()
        elif vision_key == "4":
            pass

        if control_key == "1":
            remote_control.launch()
        elif control_key == "2":
            center_control.execute_schedule()
            print("Schedule finished. Remote control enabled.")
            remote_control.launch()
    finally:
        if rgbd_viewer is not None:
            rgbd_viewer.close()

        compute_node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()