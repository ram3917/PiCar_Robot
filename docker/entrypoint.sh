#!/bin/bash
# Starts the PiCar launch file from the installed picar release in /opt/picar/current.
# This script never downloads or installs anything: run install_picar.sh by hand for that.
# PICAR_HARDWARE selects the adapter: picar4wd (default) or fake.
# PICAR_DRIVE=true lets the planner drive (default false: the robot stays still).
# PICAR_RECORD=true also records the sensor topics and commands with rosbag2 into /data.
# No "set -u": the ROS setup scripts read variables that may be unset.
set -eo pipefail

source /ws/picar_env.sh

if [ ! -d /opt/picar/current ]; then
    echo "picar is not installed. Set PICAR_VERSION in docker/.env, then run:" >&2
    echo "  docker compose run --rm --entrypoint /ws/install_picar.sh picar" >&2
    exit 1
fi

installed="$(cat /opt/picar/current/PICAR_VERSION 2>/dev/null || echo unknown)"
if [ "${installed}" != "${PICAR_VERSION:-}" ]; then
    echo "WARNING: picar ${installed} is installed, but PICAR_VERSION is ${PICAR_VERSION:-unset}." >&2
    echo "         To update: docker compose run --rm --entrypoint /ws/install_picar.sh picar" >&2
fi

exec ros2 launch picar picar.launch.py \
    "hardware:=${PICAR_HARDWARE:-picar4wd}" \
    "drive:=${PICAR_DRIVE:-false}" \
    "record:=${PICAR_RECORD:-false}"
