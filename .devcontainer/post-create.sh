#!/bin/bash
# Runs once after the dev container is created. The image already has ROS 2, colcon, uv and
# picar_core (from docker/Dockerfile). This sets up the uv workspace for tests and tools,
# makes picar_core editable in the ROS Python so the nodes see workspace edits, and builds
# picar_ros from the mounted source.
# picar-4wd is not installed here: only the robot needs it, and the tests use a stand-in.
# No "set -u": the ROS setup script reads variables that may be unset.
set -eo pipefail

repo="$(cd "$(dirname "$0")/.." && pwd)"
cd "${repo}"

uv sync --locked
uv pip install --system --break-system-packages -e "${repo}/core"

# Viewers for watching the stack run. They live here, not in docker/Dockerfile, to keep the
# robot image small. rqt_graph shows the nodes and topics, rqt_plot and PlotJuggler plot values
# over time, rviz2 draws the ultrasonic range, and rqt_console shows the node logs.
apt-get update
apt-get install -y --no-install-recommends \
    ros-jazzy-plotjuggler-ros \
    ros-jazzy-rqt-console \
    ros-jazzy-rqt-graph \
    ros-jazzy-rqt-plot \
    ros-jazzy-rqt-service-caller \
    ros-jazzy-rqt-topic \
    ros-jazzy-rviz2
rm -rf /var/lib/apt/lists/*

# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
cd "${repo}/ros"
colcon build --symlink-install --packages-select picar_ros

echo "Dev container ready. Tests: uv run --package picar-core pytest core (core), scripts/test_ros.sh (ROS)"
echo "Nodes: source ros/install/setup.bash, then ros2 launch picar_ros picar.launch.py hardware:=fake"
