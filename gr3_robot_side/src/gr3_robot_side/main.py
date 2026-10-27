import json
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String

from gr3_robot_side.robot.gr3 import FourierGR3
from gr3_robot_side.robot.lidar_publisher import GR3LidarPublisher
from gr3_robot_side.robot.state_publisher import GR3StatePublisher


class GR3RobotSide(Node):
    def __init__(self, gr3):
        super().__init__("gr3_robot_side")
        self.gr3 = gr3

        # subscriber
        self.locomotion_command_subscription = self.create_subscription(String, "locomotion_command", self.locomotion_command_callback, 10)
        self.manipulation_command_subscription = self.create_subscription(String, "manipulation_command", self.manipulation_command_callback, 10)

        # publisher
        self.locomotion_status_publisher = self.create_publisher(
            String, "locomotion_status", 10
        )
        self.manipulation_status_publisher = self.create_publisher(
            String, "manipulation_status", 10
        )

    def publish_status(self, publisher, command_id, state):
        msg = String()
        msg.data = json.dumps({
            "id": command_id,
            "state": state,
        })
        publisher.publish(msg)

    def locomotion_command_callback(self, msg):
        command = json.loads(msg.data)
        command_id = command["id"]

        self.publish_status(
            self.locomotion_status_publisher,
            command_id,
            "RUNNING",
        )

        self.get_logger().info(f"locomotion: message={msg.data}")
        result = self.gr3.locomotion(msg)

        self.publish_status(
            self.locomotion_status_publisher,
            command_id,
            "SUCCESS" if result else "FAIL",
        )


    def manipulation_command_callback(self, msg):
        command = json.loads(msg.data)
        command_id = command["id"]

        self.publish_status(
            self.manipulation_status_publisher,
            command_id,
            "RUNNING",
        )

        self.get_logger().info(f"manipulation: message={msg.data}")
        result = self.gr3.manipulation(msg)

        self.publish_status(
            self.manipulation_status_publisher,
            command_id,
            "SUCCESS" if result else "FAIL",
        )


def main():
    rclpy.init()

    gr3 = FourierGR3()
    gr3.launch()

    command_node = GR3RobotSide(gr3)
    state_node = GR3StatePublisher(gr3)
    lidar_node = GR3LidarPublisher()

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(command_node)
    executor.add_node(state_node)
    executor.add_node(lidar_node)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        command_node.destroy_node()
        state_node.destroy_node()
        lidar_node.destroy_node()
        gr3.shutdown()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
