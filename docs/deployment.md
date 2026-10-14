# Deployment

Diagram: [deployment.puml](deployment.puml).

> **Status:** the registry pipeline below (CI, GHCR, `update.sh`) is planned and not set up.
> Today the image is built on the dev machine and copied to the Pi by hand, as described in
> [Deploy to the Pi](../README.md#deploy-to-the-pi). The sections on what runs on the Pi and the
> Pi prerequisites apply either way.

## Pipeline

```
git push → GitHub Actions → GHCR (linux/arm64) → update.sh on the Pi
```

1. A push to `main`, or a `v*` tag, starts GitHub Actions.
2. CI runs the `picar_core` tests, then builds the image for `linux/arm64`.
3. The image is pushed to `ghcr.io/ram3917/picar_robot` with these tags:

   | Tag | Points at |
   |---|---|
   | `latest` | The newest build of `main` |
   | `sha-<short>` | One specific commit |
   | `vX.Y.Z` | A tagged release |

4. On the Pi, `./scripts/update.sh` pulls the image and restarts the container.

Nothing is built on the Pi; it only pulls finished images.

## Updating the Pi

```bash
./scripts/update.sh
```

This runs `docker compose pull` and `docker compose up -d` against
[docker/compose.yaml](../docker/compose.yaml), then prints the image tag that is running.
Running it again with nothing new to pull changes nothing.

## Rolling back

`compose.yaml` takes the image tag from the `IMAGE_TAG` environment variable, defaulting to
`latest`. To roll back, run the update with an older tag:

```bash
IMAGE_TAG=v0.1.0 ./scripts/update.sh
```

Any pushed tag works, including `sha-<short>`.

With the manual flow, keep the previous image loaded on the Pi (`docker images` lists it) and
point compose at it:

```bash
cd ~/docker
echo "IMAGE_TAG=<old tag>" > .env
docker compose up -d --force-recreate --pull never
```

Recordings and parameters are not part of the image, so a rollback keeps them.

## Recording data

The `data/` folder next to `docker/` is mounted at `/data` in the container. To record the sensor
frames and plans with rosbag2, set `PICAR_RECORD=true` in `docker/.env` (next to `IMAGE_TAG`) and
recreate the container. Each start writes a new `picar_<timestamp>` folder, and the folder is
ignored by git.

## What runs on the Pi

One container, started by Docker Compose:

| Setting | Value | Why |
|---|---|---|
| Image | `ghcr.io/ram3917/picar_robot:${IMAGE_TAG:-latest}` | Tag selectable for rollback |
| Base | `ros:jazzy-ros-base` | ROS 2 Jazzy |
| `network_mode` | `host` | ROS 2 discovery from other machines on the network |
| `restart` | `unless-stopped` | Stack comes back after a power cycle |
| Devices | `/dev/i2c-1`, `/dev/gpiomem`, `/dev/gpiochip0` | HAT access without `privileged`; `rpi-lgpio` drives the pins through the GPIO chip |
| `RPI_LGPIO_REVISION` | `a020d3` (Pi 3 Model B+) | The container cannot read `/proc/device-tree`, so `rpi-lgpio` needs the board revision |
| Volume | `../data` mounted at `/data` | rosbag2 recordings when `PICAR_RECORD=true` |

The container's entrypoint launches the controller and motion planner nodes. The robot
boots `DISARMED`; arm it with the `/arm` service (served by the controller).

## Pi prerequisites

- 64-bit Raspberry Pi OS (`uname -m` prints `aarch64`)
- Docker with the compose plugin, and the user in the `docker` group
- I2C enabled **permanently**: `dtparam=i2c_arm=on` uncommented in `/boot/firmware/config.txt`
  (or `sudo raspi-config nonint do_i2c 0`). `dtparam i2c_arm=on` enables it only until the next
  reboot. `/dev/i2c-1` must exist, and `sudo i2cdetect -y 1` should show `14` (the HAT)
- `/dev/gpiochip0` present (`ls -l /dev/gpiochip0`)
- Read access to the GHCR package (public package, or a read token)
