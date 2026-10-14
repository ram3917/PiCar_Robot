#!/bin/bash
# Runs the picar_ros tests (ROS nodes and the launch file) in the dev container.
# Builds picar_ros first so the launch test runs the current entry points.
# The pure-Python core tests are separate: uv run --package picar-core pytest core
#
#   scripts/test_ros.sh              # all ROS tests
#   scripts/test_ros.sh -k planning  # extra arguments go to pytest
#
# No "set -u": the ROS setup scripts read variables that may be unset.
set -eo pipefail

repo="$(cd "$(dirname "$0")/.." && pwd)"

# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
cd "${repo}/ros"
colcon build --symlink-install --packages-select picar_ros
# shellcheck disable=SC1091
source "${repo}/ros/install/setup.bash"

# rclpy lives in the ROS system Python, not in the uv .venv that may be first on PATH.
exec /usr/bin/python3 -m pytest "${repo}/ros/picar_ros" "$@"
