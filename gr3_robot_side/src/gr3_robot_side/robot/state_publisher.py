from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, JointState

# 31 movable joints
GROUP_JOINT_NAMES = {
    "waist": [
        "waist_yaw_joint",
        "waist_roll_joint",
        "waist_pitch_joint",
    ],
    "head": [
        "head_yaw_joint",
        "head_pitch_joint",
    ],
    "left_manipulator": [
        "left_shoulder_pitch_joint",
        "left_shoulder_roll_joint",
        "left_shoulder_yaw_joint",
        "left_elbow_pitch_joint",
        "left_wrist_yaw_joint",
        "left_wrist_pitch_joint",
        "left_wrist_roll_joint",
    ],
    "right_manipulator": [
        "right_shoulder_pitch_joint",
        "right_shoulder_roll_joint",
        "right_shoulder_yaw_joint",
        "right_elbow_pitch_joint",
        "right_wrist_yaw_joint",
        "right_wrist_pitch_joint",
        "right_wrist_roll_joint",
    ],
    "left_leg": [
        "left_hip_pitch_joint",
        "left_hip_roll_joint",
        "left_hip_yaw_joint",
        "left_knee_pitch_joint",
        "left_ankle_pitch_joint",
        "left_ankle_roll_joint",
    ],
    "right_leg": [
        "right_hip_pitch_joint",
        "right_hip_roll_joint",
        "right_hip_yaw_joint",
        "right_knee_pitch_joint",
        "right_ankle_pitch_joint",
        "right_ankle_roll_joint",
    ],
}


class GR3StatePublisher(Node):
    def __init__(self, gr3):
        super().__init__("gr3_state_publisher")
        self.gr3 = gr3

        self.joint_state_publisher = self.create_publisher(
            JointState,
            "/joint_states",
            10,
        )
        self.imu_publisher = self.create_publisher(
            Imu,
            "/imu/data_raw",
            qos_profile_sensor_data,
        )

        self.joint_state_timer = self.create_timer(
            1.0 / 30.0,
            self.publish_joint_states,
        )
        self.imu_timer = self.create_timer(
            1.0 / 50.0,
            self.publish_imu,
        )

    def publish_joint_states(self):
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()

        for group_name, joint_names in GROUP_JOINT_NAMES.items():
            positions = self.gr3.get_group_state(group_name, "position")
            velocities = self.gr3.get_group_state(group_name, "velocity")
            efforts = self.gr3.get_group_state(group_name, "effort")

            expected_length = len(joint_names)
            if (
                len(positions) != expected_length
                or len(velocities) != expected_length
                or len(efforts) != expected_length
            ):
                self.get_logger().error(
                    f"Invalid state length for {group_name}: "
                )
                return

            message.name.extend(joint_names)
            message.position.extend(float(value) for value in positions)
            message.velocity.extend(float(value) for value in velocities)
            message.effort.extend(float(value) for value in efforts)

        self.joint_state_publisher.publish(message)

    def publish_imu(self):
        angular_velocity = self.gr3.get_base_data("omega_B")
        linear_acceleration = self.gr3.get_base_data("acc_B")

        if len(angular_velocity) != 3 or len(linear_acceleration) != 3:
            self.get_logger().error(
                "Invalid IMU state length: "
                f"omega_B={len(angular_velocity)}, "
                f"acc_B={len(linear_acceleration)}"
            )
            return

        message = Imu()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "imu_link"

        # Raw IMU does not provide an orientation estimate.
        message.orientation_covariance[0] = -1.0

        message.angular_velocity.x = float(angular_velocity[0])
        message.angular_velocity.y = float(angular_velocity[1])
        message.angular_velocity.z = float(angular_velocity[2])

        message.linear_acceleration.x = float(linear_acceleration[0])
        message.linear_acceleration.y = float(linear_acceleration[1])
        message.linear_acceleration.z = float(linear_acceleration[2])

        self.imu_publisher.publish(message)
