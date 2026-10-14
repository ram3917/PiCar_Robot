#!/bin/bash
# Starts the PiCar launch file. PICAR_HARDWARE selects the adapter: picar4wd (default) or fake.
# No "set -u": the ROS setup scripts read variables that may be unset.
set -eo pipefail

source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash

exec ros2 launch picar_ros picar.launch.py "hardware:=${PICAR_HARDWARE:-picar4wd}"
