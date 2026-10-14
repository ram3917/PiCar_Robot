"""Starts RViz and rqt for watching the stack by hand, in the dev container's browser desktop.

    ros2 launch picar picar.launch.py hardware:=fake     # in one terminal
    ros2 launch picar viz.launch.py                       # in another, then open localhost:6080

RViz shows the ultrasonic range; rqt opens the topic monitor, node graph and plot plugins so
/cmd_vel and /joint_states can be watched. The viewers are installed by the dev container, not
the robot image. Turn RViz or rqt off with rviz:=false or rqt:=false; add PlotJuggler with
plotjuggler:=true.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    rviz_config = os.path.join(get_package_share_directory("picar"), "config", "picar.rviz")

    return LaunchDescription(
        [
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("rqt", default_value="true"),
            DeclareLaunchArgument("plotjuggler", default_value="false"),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", rviz_config],
                condition=IfCondition(LaunchConfiguration("rviz")),
            ),
            # rqt takes its plugins by name: the topic monitor, then the node graph.
            ExecuteProcess(
                cmd=["rqt", "--standalone", "rqt_topic"],
                condition=IfCondition(LaunchConfiguration("rqt")),
                output="screen",
            ),
            ExecuteProcess(
                cmd=["rqt", "--standalone", "rqt_graph"],
                condition=IfCondition(LaunchConfiguration("rqt")),
                output="screen",
            ),
            ExecuteProcess(
                cmd=["rqt", "--standalone", "rqt_plot"],
                condition=IfCondition(LaunchConfiguration("rqt")),
                output="screen",
            ),
            Node(
                package="plotjuggler",
                executable="plotjuggler",
                condition=IfCondition(LaunchConfiguration("plotjuggler")),
            ),
        ]
    )
