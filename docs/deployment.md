# Deployment

Diagram: [deployment.puml](deployment.puml).

> **Status:** this describes the planned pipeline. The image and `compose.yaml` arrive with
> MVP-11, CI with MVP-12, `update.sh` with MVP-13 and the Pi setup notes with MVP-14.

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

## What runs on the Pi

One container, started by Docker Compose:

| Setting | Value | Why |
|---|---|---|
| Image | `ghcr.io/ram3917/picar_robot:${IMAGE_TAG:-latest}` | Tag selectable for rollback |
| Base | `ros:jazzy-ros-base` | ROS 2 Jazzy |
| `network_mode` | `host` | ROS 2 discovery from other machines on the network |
| `restart` | `unless-stopped` | Stack comes back after a power cycle |
| Devices | `/dev/i2c-1`, `/dev/gpiomem` | HAT access without `privileged` |

The container's entrypoint launches the perception, planning and action nodes. The robot
boots `DISARMED`; arm it with the `/arm` service.

## Pi prerequisites

- 64-bit Raspberry Pi OS (`uname -m` prints `aarch64`)
- Docker with the compose plugin, and the user in the `docker` group
- I2C enabled (`raspi-config`)
- Read access to the GHCR package (public package, or a read token)
