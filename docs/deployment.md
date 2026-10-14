# Deployment

Diagram: [deployment.puml](deployment.puml).

## Flow

```
dev machine: docker buildx (arm64) -> docker save -> scp -> Pi: docker load -> docker compose up -d
```

There is no registry: the image is built on the dev machine and copied to the Pi. The steps are
in [Deploy to the Pi](../README.md#deploy-to-the-pi).

## Updating and rolling back

Build and load a new image with a new tag, set `IMAGE_TAG` in `docker/.env` and run
`docker compose up -d --pull never`. To roll back, set `IMAGE_TAG` to the previous tag, which is
still loaded on the Pi (`docker images` lists it), and run the same command.

Recordings and parameters are not part of the image, so a rollback keeps them.

## Recording data

The `data/` folder next to `docker/` is mounted at `/data` in the container. To record the sensor
topics and commands with rosbag2, set `PICAR_RECORD=true` in `docker/.env` (next to `IMAGE_TAG`) and
recreate the container. Each start writes a new `rosbag2_<timestamp>` folder, and the folder is
ignored by git.

## What runs on the Pi

One container, started by Docker Compose:

| Setting | Value | Why |
|---|---|---|
| Image | `picar:${IMAGE_TAG:-latest}` | Tag selectable for rollback |
| Base | `ros:jazzy-ros-base` | ROS 2 Jazzy |
| `network_mode` | `host` | ROS 2 discovery from other machines on the network |
| `restart` | `unless-stopped` | Stack comes back after a power cycle |
| Devices | `/dev/i2c-1`, `/dev/gpiomem`, `/dev/gpiochip0` | HAT access without `privileged`; `rpi-lgpio` drives the pins through the GPIO chip |
| `RPI_LGPIO_REVISION` | `a020d3` (Pi 3 Model B+) | The container cannot read `/proc/device-tree`, so `rpi-lgpio` needs the board revision |
| Volume | `../data` mounted at `/data` | rosbag2 recordings when `PICAR_RECORD=true` |

The container's entrypoint launches the controller and planner nodes. The robot stays still
until `PICAR_DRIVE=true` is set in `docker/.env` (and the container recreated). To switch it on
without a restart, open a shell in the container (`docker compose exec picar bash`), source
`/opt/ros/jazzy/setup.bash` and run `ros2 param set /planner drive true`.

## Pi prerequisites

- 64-bit Raspberry Pi OS (`uname -m` prints `aarch64`)
- Docker with the compose plugin, and the user in the `docker` group
- I2C enabled **permanently**: `dtparam=i2c_arm=on` uncommented in `/boot/firmware/config.txt`
  (or `sudo raspi-config nonint do_i2c 0`). `dtparam i2c_arm=on` enables it only until the next
  reboot. `/dev/i2c-1` must exist, and `sudo i2cdetect -y 1` should show `14` (the HAT)
- `/dev/gpiochip0` present (`ls -l /dev/gpiochip0`)
