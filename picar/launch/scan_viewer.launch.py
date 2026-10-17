"""Throwaway: builds the occupancy grid from /controller/scan and publishes it on /map.

Run it next to the controller (``ros2 launch picar picar.launch.py hardware:=sim``). View /map in
rviz2 with the dev container's desktop. Grid geometry is set here, not in params.yaml.

    ros2 launch picar scan_viewer.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="picar",
                executable="scan_viewer_node",
                name="scan_viewer",
                parameters=[
                    {
                        "resolution": 0.05,  # metres per cell
                        "width_m": 4.0,      # grid size, centred on the sensor
                        "height_m": 4.0,
                        "cone_deg": 15.0,    # width of the beam each reading fills in
                        "l_occ": 0.85,       # log-odds added per hit
                        "l_free": -0.4,      # log-odds added per cell passed through
                        "l_min": -2.0,       # clamps keep the map able to change
                        "l_max": 3.5,
                    }
                ],
                output="screen",
            ),
        ]
    )
