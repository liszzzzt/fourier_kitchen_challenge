import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("gr3_ros2_bringup")
    configuration_directory = os.path.join(
        package_share,
        "config",
        "cartographer",
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    configuration_basename = LaunchConfiguration("configuration_basename")
    scan_topic = LaunchConfiguration("scan_topic")
    imu_topic = LaunchConfiguration("imu_topic")
    resolution = LaunchConfiguration("resolution")
    publish_period_sec = LaunchConfiguration("publish_period_sec")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "configuration_basename",
                default_value="cartographer_mapping.lua",
            ),
            DeclareLaunchArgument(
                "scan_topic",
                default_value="/gr3/lidar/scan",
            ),
            DeclareLaunchArgument(
                "imu_topic",
                default_value="/imu/data_raw",
            ),
            DeclareLaunchArgument("resolution", default_value="0.05"),
            DeclareLaunchArgument("publish_period_sec", default_value="1.0"),
            Node(
                package="cartographer_ros",
                executable="cartographer_node",
                name="gr3_cartographer",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
                arguments=[
                    "-configuration_directory",
                    configuration_directory,
                    "-configuration_basename",
                    configuration_basename,
                ],
                remappings=[
                    ("scan", scan_topic),
                    ("imu", imu_topic),
                ],
            ),
            Node(
                package="cartographer_ros",
                executable="cartographer_occupancy_grid_node",
                name="gr3_occupancy_grid",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
                arguments=[
                    "-resolution",
                    resolution,
                    "-publish_period_sec",
                    publish_period_sec,
                ],
            ),
        ]
    )
