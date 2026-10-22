import sys
import termios
import tty
import rclpy
from gr3_compute_side.high_level_planner.center_control import (
    CenterControl,
    GR3ComputeSide
)
from gr3_compute_side.remote_control.remote_control import (
    RemoteControl,
    read_key,
)


def main():
    rclpy.init()

    compute_node = GR3ComputeSide()
    center_control = CenterControl(compute_node)
    remote_control = RemoteControl(compute_node)

    key = read_key("Press 1 to remote control; Press 2 to execute schedule: ")

    if key == "1":
        try:
            remote_control.launch()
        finally:
            compute_node.destroy_node()
            rclpy.shutdown()
    elif key == "2":
        try:
            center_control.execute_schedule()
        finally:
            compute_node.destroy_node()
            rclpy.shutdown()
    else:
        print("Invalid command")

if __name__ == "__main__":
    main()