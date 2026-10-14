"""Starts perception, planning and action with config/params.yaml.

    ros2 launch picar_ros picar.launch.py hardware:=fake
    ros2 launch picar_ros picar.launch.py hardware:=picar4wd
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    params_file = os.path.join(get_package_share_directory("picar_ros"), "config", "params.yaml")
    hardware = LaunchConfiguration("hardware")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "hardware",
                default_value="fake",
                description="Hardware adapter: fake (in memory) or picar4wd (the robot).",
            ),
            Node(
                package="picar_ros",
                executable="perception_node",
                name="perception",
                parameters=[params_file, {"hardware": hardware}],
                output="screen",
            ),
            Node(
                package="picar_ros",
                executable="planning_node",
                name="planning",
                parameters=[params_file],
                output="screen",
            ),
            Node(
                package="picar_ros",
                executable="action_node",
                name="action",
                parameters=[params_file, {"hardware": hardware}],
                output="screen",
            ),
        ]
    )
