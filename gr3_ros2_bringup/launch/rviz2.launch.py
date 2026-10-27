import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("gr3_ros2_bringup")
    rviz2_config = os.path.join(
        package_share,
        "config",
        "rviz2",
        "rviz2.rviz",
    )
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            Node(
                package="rviz2",
                executable="rviz2",
                name="gr3_rviz2",
                output="screen",
                arguments=["-d", rviz2_config],
                parameters=[{"use_sim_time": use_sim_time}],
            ),
        ]
    )
