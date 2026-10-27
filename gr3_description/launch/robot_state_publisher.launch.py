import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("gr3_description")
    urdf_path = os.path.join(package_share, "urdf", "gr3v2_3_3.urdf")

    with open(urdf_path, "r", encoding="utf-8") as urdf_file:
        robot_description = urdf_file.read()

    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use the ROS simulation clock when true.",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="gr3_robot_state_publisher",
                output="screen",
                parameters=[
                    {
                        "robot_description": robot_description,
                        "use_sim_time": use_sim_time,
                    }
                ],
            ),
        ]
    )
