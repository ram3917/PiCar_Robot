"""Hand testing in the dev container: the stack on fake hardware, with RViz showing the sensor and motors.

    ros2 launch picar manual_test.launch.py                              # then open localhost:6080
    ros2 launch picar manual_test.launch.py ultrasonic_m:=0.5
    ros2 launch picar manual_test.launch.py ultrasonic_m:=1.0,0.5,0.15   # scripted readings, last repeats
    ros2 launch picar manual_test.launch.py drive:=false                 # no planner commands

ultrasonic_m is the fake ultrasonic output in metres, one reading per sensor period. RViz shows
the ultrasonic cone and the four motor powers (green forward, red reverse). With drive:=true the
planner cruises until the range is under stop_distance_m, then stops, reverses and turns, so a
short ultrasonic_m makes the motor bars move. Drive by hand with, for example:
    ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}}"
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _stack(context):
    share = get_package_share_directory("picar")
    params_file = os.path.join(share, "config", "params.yaml")
    ranges = [float(v) for v in LaunchConfiguration("ultrasonic_m").perform(context).split(",")]
    drive = LaunchConfiguration("drive").perform(context) == "true"
    return [
        Node(
            package="picar",
            executable="controller_node",
            name="controller",
            parameters=[params_file, {"hardware": "fake", "fake_ranges_m": ranges}],
            output="screen",
        ),
        Node(
            package="picar",
            executable="planner_node",
            name="planner",
            parameters=[params_file, {"drive": drive}],
            output="screen",
        ),
        # A test helper, installed under share/picar/test rather than as an entry point.
        ExecuteProcess(
            cmd=["python3", os.path.join(share, "test", "motor_markers_node.py")],
            output="screen",
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "ultrasonic_m",
                default_value="1.0",
                description="Fake ultrasonic readings in metres, comma separated; the last repeats.",
            ),
            DeclareLaunchArgument("drive", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="true"),
            OpaqueFunction(function=_stack),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", os.path.join(get_package_share_directory("picar"), "config", "picar.rviz")],
                condition=IfCondition(LaunchConfiguration("rviz")),
            ),
        ]
    )
