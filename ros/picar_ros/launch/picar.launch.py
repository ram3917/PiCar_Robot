"""Starts the controller and the motion planner with config/params.yaml.

    ros2 launch picar_ros picar.launch.py hardware:=fake
    ros2 launch picar_ros picar.launch.py hardware:=picar4wd
    ros2 launch picar_ros picar.launch.py hardware:=picar4wd record:=true   # rosbag2 to /data
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
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
            DeclareLaunchArgument(
                "record",
                default_value="false",
                description="Record the sensor frames and plans with rosbag2 into record_dir.",
            ),
            DeclareLaunchArgument(
                "record_dir",
                default_value="/data",
                description="Folder the rosbag2 recording is written under.",
            ),
            Node(
                package="picar_ros",
                executable="controller_node",
                name="controller",
                parameters=[params_file, {"hardware": hardware}],
                output="screen",
            ),
            Node(
                package="picar_ros",
                executable="motion_planner_node",
                name="motion_planner",
                parameters=[params_file],
                output="screen",
            ),
            ExecuteProcess(
                cmd=[
                    "ros2", "bag", "record",
                    "/controller/sensor_frame", "/planner/motion_plan",
                    # rosbag2 refuses an existing folder, so each recording gets its own.
                    "-o", PythonExpression([
                        "'", LaunchConfiguration("record_dir"),
                        "/picar_' + __import__('time').strftime('%Y%m%d_%H%M%S')",
                    ]),
                ],
                condition=IfCondition(LaunchConfiguration("record")),
                output="screen",
            ),
        ]
    )
