"""Starts the controller and the planner with config/params.yaml.

    ros2 launch picar picar.launch.py hardware:=fake
    ros2 launch picar picar.launch.py hardware:=sim drive:=true        # fake wall to avoid
    ros2 launch picar picar.launch.py hardware:=picar4wd drive:=true
    ros2 launch picar picar.launch.py hardware:=picar4wd record:=true   # rosbag2 into /data
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

from picar.interface import (
    CMD_VEL_TOPIC,
    GREYSCALE_TOPIC,
    JOINT_STATES_TOPIC,
    RANGE_TOPIC,
    SCAN_TOPIC,
)


def generate_launch_description() -> LaunchDescription:
    params_file = os.path.join(get_package_share_directory("picar"), "config", "params.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "hardware",
                default_value="fake",
                description="Hardware adapter: fake (scripted ranges), sim (fake range from a "
                "simulated wall that follows the motors) or picar4wd (the robot).",
            ),
            DeclareLaunchArgument(
                "drive",
                default_value="false",
                description="Let the planner command the robot. Also settable while running: "
                "ros2 param set /planner drive true",
            ),
            DeclareLaunchArgument(
                "record",
                default_value="false",
                description="Record the sensor topics and commands with rosbag2 into /data.",
            ),
            Node(
                package="picar",
                executable="controller_node",
                name="controller",
                parameters=[params_file, {"hardware": LaunchConfiguration("hardware")}],
                output="screen",
            ),
            Node(
                package="picar",
                executable="planner_node",
                name="planner",
                parameters=[
                    params_file,
                    {"drive": PythonExpression(["'", LaunchConfiguration("drive"), "' == 'true'"])},
                ],
                output="screen",
            ),
            # rosbag2 names each recording folder by the time it starts, inside cwd.
            ExecuteProcess(
                cmd=[
                    "ros2", "bag", "record",
                    RANGE_TOPIC, SCAN_TOPIC, GREYSCALE_TOPIC, JOINT_STATES_TOPIC, CMD_VEL_TOPIC,
                ],
                cwd="/data",
                condition=IfCondition(LaunchConfiguration("record")),
                output="screen",
            ),
        ]
    )
