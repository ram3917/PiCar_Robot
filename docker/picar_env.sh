# Puts ROS 2 and the installed picar release on the shell's paths. Sourced by entrypoint.sh, by
# install_picar.sh, and by every interactive shell (see docker/Dockerfile). Safe to source twice.
# The installed release is in /opt/picar/current, made by install_picar.sh. Without one (for
# example in the dev container, which builds with colcon), only ROS 2 is set up.
# No "set -u": the ROS setup script reads variables that may be unset.

# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash

PICAR_PREFIX="${PICAR_PREFIX:-/opt/picar/current}"
if [ -d "${PICAR_PREFIX}" ]; then
    export AMENT_PREFIX_PATH="${PICAR_PREFIX}${AMENT_PREFIX_PATH:+:${AMENT_PREFIX_PATH}}"
    for site in "${PICAR_PREFIX}"/lib/python3*/site-packages; do
        if [ -d "${site}" ]; then
            export PYTHONPATH="/opt/picar-4wd:${site}${PYTHONPATH:+:${PYTHONPATH}}"
        fi
    done
fi
