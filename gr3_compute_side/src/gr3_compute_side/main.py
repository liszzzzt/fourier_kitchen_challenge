import rclpy
from gr3_compute_side.high_level_planner.center_control import (
    CenterControl,
    GR3ComputeSide
)

def main():
    rclpy.init()

    compute_node = GR3ComputeSide()
    control = CenterControl(compute_node)

    try:
        control.launch()
    finally:
        compute_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()