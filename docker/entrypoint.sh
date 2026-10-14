#!/bin/bash
# Starts the PiCar launch file. PICAR_HARDWARE selects the adapter: picar4wd (default) or fake.
# PICAR_DRIVE=true lets the planner drive (default false: the robot stays still).
# PICAR_RECORD=true also records the sensor topics and commands with rosbag2 into /data.
# No "set -u": the ROS setup scripts read variables that may be unset.
set -eo pipefail

source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash

exec ros2 launch picar picar.launch.py \
    "hardware:=${PICAR_HARDWARE:-picar4wd}" \
    "drive:=${PICAR_DRIVE:-false}" \
    "record:=${PICAR_RECORD:-false}"
