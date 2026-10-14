# The PiCar image: ROS 2 Jazzy, picar_core, picar_interfaces, picar_ros and SunFounder's
# picar-4wd library.
# Built for linux/arm64 in CI; a linux/amd64 build works for smoke tests on a dev box.
FROM ros:jazzy-ros-base

# uv installs Python packages into the ROS system Python (rclpy lives there too).
COPY --from=ghcr.io/astral-sh/uv:0.12.5 /uv /usr/local/bin/uv

# picar-4wd is not on PyPI, and its setup.py cannot run in a build (it calls getlogin(), which
# needs a terminal). So the pinned commit's picar_4wd/ package is copied in, not installed.
ARG PICAR_4WD_REF=61de4663716098d1257e148a6181d023f4c69475

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        build-essential \
        git \
        python3-colcon-common-extensions \
        python3-dev \
        python3-smbus \
        ros-jazzy-rosbag2 \
    && rm -rf /var/lib/apt/lists/*

# Dependencies picar-4wd imports (RPi.GPIO for pins, smbus2 for I2C), then the library itself,
# taken from the pinned commit and put on PYTHONPATH. rpi-lgpio provides the RPi.GPIO module
# without relying on /proc/cpuinfo "Hardware", which current Pi kernels do not report.
RUN uv pip install --system --break-system-packages rpi-lgpio smbus2 \
    && git clone --quiet https://github.com/sunfounder/picar-4wd.git /tmp/picar-4wd \
    && git -C /tmp/picar-4wd checkout --quiet "${PICAR_4WD_REF}" \
    && mkdir -p /opt/picar-4wd \
    && cp -r /tmp/picar-4wd/picar_4wd /opt/picar-4wd/ \
    && rm -rf /tmp/picar-4wd
ENV PYTHONPATH=/opt/picar-4wd

WORKDIR /ws

COPY core /ws/core
RUN uv pip install --system --break-system-packages /ws/core

COPY ros/picar_interfaces /ws/src/picar_interfaces
COPY ros/picar_ros /ws/src/picar_ros
RUN . /opt/ros/jazzy/setup.sh \
    && colcon build --packages-select picar_interfaces picar_ros \
    && rm -rf /ws/build /ws/log /ws/src

COPY docker/entrypoint.sh /ws/entrypoint.sh
RUN chmod +x /ws/entrypoint.sh

ENV PYTHONUNBUFFERED=1
ENTRYPOINT ["/ws/entrypoint.sh"]
