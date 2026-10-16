# The PiCar base image: ROS 2 Jazzy, uv, SunFounder's picar-4wd library. It does not contain the
# picar package. That is installed from a GitHub Release by install_picar.sh, so this image
# changes only when the OS, ROS or picar-4wd change. Built for linux/arm64 for the Pi.
FROM ros:jazzy-ros-base

# uv installs Python packages into the ROS system Python (rclpy lives there too).
COPY --from=ghcr.io/astral-sh/uv:0.12.5 /uv /usr/local/bin/uv

# picar-4wd is not on PyPI, and its setup.py cannot run in a build (it calls getlogin(), which
# needs a terminal). So the pinned commit's picar_4wd/ package is copied in, not installed.
ARG PICAR_4WD_REF=61de4663716098d1257e148a6181d023f4c69475

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        build-essential \
        ca-certificates \
        curl \
        git \
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

# picar-4wd reads its motor-reverse flags and servo offset from ~/.picar-4wd/config, under the
# first folder in /home, and prints an error for each setting when the file is missing. An empty
# file keeps the defaults quietly. To flip a motor, add e.g. "left_front_reverse = True" to it.
RUN user="$(ls /home | head -n 1)" \
    && mkdir -p "/home/${user}/.picar-4wd" \
    && touch "/home/${user}/.picar-4wd/config"

# The ROS and picar environment, for shells opened with `docker compose exec picar bash`.
# The entrypoint and install_picar.sh source the same file.
WORKDIR /ws
COPY docker/picar_env.sh docker/entrypoint.sh docker/install_picar.sh /ws/
RUN chmod +x /ws/entrypoint.sh /ws/install_picar.sh \
    && echo 'source /ws/picar_env.sh' >> /etc/bash.bashrc

ENV PYTHONUNBUFFERED=1
ENTRYPOINT ["/ws/entrypoint.sh"]
