#!/bin/bash
# Runs once after the dev container is created. The image already has ROS 2, colcon and the
# built picar package (from docker/Dockerfile). This installs the viewers and the test tools,
# and builds picar from the mounted source so edits show up (--symlink-install).
# picar-4wd is not installed here: only the robot needs it, and the tests use a stand-in.
# No "set -u": the ROS setup script reads variables that may be unset.
set -eo pipefail

repo="$(cd "$(dirname "$0")/.." && pwd)"

# Viewers for watching the stack run (ros2 launch picar viz.launch.py). They live here, not in
# docker/Dockerfile, to keep the robot image small.
apt-get update
apt-get install -y --no-install-recommends \
    python3-pytest \
    ros-jazzy-launch-testing \
    ros-jazzy-launch-testing-ros \
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
cd "${repo}/picar"
colcon build --symlink-install --packages-select picar

echo "Dev container ready. In each terminal: source picar/install/setup.bash"
echo "Tests: pytest picar. Stack: ros2 launch picar picar.launch.py hardware:=fake drive:=true"
echo "Viewers: ros2 launch picar viz.launch.py, then open http://localhost:6080"
