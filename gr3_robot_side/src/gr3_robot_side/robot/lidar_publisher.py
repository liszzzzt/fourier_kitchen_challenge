from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class GR3LidarPublisher(Node):
    def __init__(self):
        super().__init__("gr3_lidar_publisher")

        self.lidar_publisher = self.create_publisher(
            LaserScan,
            "/gr3/lidar/scan",
            qos_profile_sensor_data,
        )
        self.lidar_subscription = self.create_subscription(
            LaserScan,
            "/gr3/mujoco_raw/lidar/scan",
            self.lidar_callback,
            qos_profile_sensor_data,
        )

    def lidar_callback(self, message):
        # lidar-processing
        self.lidar_publisher.publish(message)
