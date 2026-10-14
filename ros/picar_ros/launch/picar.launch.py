"""Starts the controller and the motion planner with config/params.yaml.

    ros2 launch picar_ros picar.launch.py hardware:=fake
    ros2 launch picar_ros picar.launch.py hardware:=picar4wd
    ros2 launch picar_ros picar.launch.py hardware:=picar4wd record:=true   # rosbag2 into /data
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from picar_ros.interface import (
    ARMED_TOPIC,
    GREYSCALE_TOPIC,
    JOINT_STATES_TOPIC,
    PLAN_TOPIC,
    RANGE_TOPIC,
)


def generate_launch_description() -> LaunchDescription:
    params_file = os.path.join(get_package_share_directory("picar_ros"), "config", "params.yaml")

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
                description="Record the sensor topics and plans with rosbag2 into /data.",
            ),
            Node(
                package="picar_ros",
                executable="controller_node",
                name="controller",
                parameters=[params_file, {"hardware": LaunchConfiguration("hardware")}],
                output="screen",
            ),
            Node(
                package="picar_ros",
                executable="motion_planner_node",
                name="motion_planner",
                parameters=[params_file],
                output="screen",
            ),
            # rosbag2 names each recording folder by the time it starts, inside cwd.
            ExecuteProcess(
                cmd=[
                    "ros2", "bag", "record",
                    RANGE_TOPIC, GREYSCALE_TOPIC, JOINT_STATES_TOPIC, ARMED_TOPIC, PLAN_TOPIC,
                ],
                cwd="/data",
                condition=IfCondition(LaunchConfiguration("record")),
                output="screen",
            ),
        ]
    )
