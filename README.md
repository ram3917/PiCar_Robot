# PiCar_Robot

An autonomous obstacle-avoiding robot built on the SunFounder **PiCar-4WD** kit and a
Raspberry Pi 3B+, running **ROS 2 Jazzy** inside Docker.

The robot's logic is split in two layers:

- **`picar_core`** — pure Python, no ROS imports. Hardware interface, state machine and
  watchdog. Runs and is tested on any machine, including Windows.
- **`picar_ros`** — a thin ROS 2 layer of three nodes (perception → planning → action)
  that wraps the core.

## MVP scope

Drive around a room on its own, using only the ultrasonic sensor to avoid obstacles.

**In scope**

- Reactive avoidance: cruise → stop → reverse → turn → cruise
- Ultrasonic sensor fixed facing forward (servo locked)
- Arm / disarm over a ROS service; the robot always boots **disarmed**
- Safety stops: stale or invalid range → fault and stop; no `/cmd_vel` → motors stop
- Manual driving with `teleop_twist_keyboard` while disarmed
- One image, built for `linux/arm64` on the dev machine and copied to the Pi by hand
- Runs without a robot (`hardware:=fake`) for development and smoke tests

**Out of scope for the MVP**

- Greyscale sensor, edge/cliff detection, line following
- Wheel-speed odometry, `/odom`, goal-seeking
- Machine-learned policies
- Unity teleop / digital twin / simulator
- Camera stream
- Servo sweeping, mapping, SLAM, IMU

## Interfaces

### Nodes

| Node | Role | Subscribes | Publishes / serves |
|---|---|---|---|
| perception | Reads the ultrasonic sensor | — | `/ultrasonic/range` |
| planning | Runs the avoidance state machine | `/ultrasonic/range` | `/cmd_vel`, `/robot_state`, service `/arm` |
| action | Drives the motors, enforces the command watchdog | `/cmd_vel` | — |

### Topics and services

| Name | Kind | Type | Notes |
|---|---|---|---|
| `/ultrasonic/range` | topic | `sensor_msgs/Range` | 10 Hz, `radiation_type=ULTRASOUND`; invalid readings are `inf`/NaN (REP 117) |
| `/cmd_vel` | topic | `geometry_msgs/Twist` | 20 Hz while armed (zeros when stopped); planning publishes nothing while disarmed |
| `/robot_state` | topic | `std_msgs/String` | `DISARMED`, `CRUISE`, `STOP`, `REVERSE`, `TURN`, `FAULT` |
| `/arm` | service | `std_srvs/SetBool` | `true` arms, `false` disarms |

### Parameters

| Parameter | Node | Meaning |
|---|---|---|
| `hardware` | perception, action | `fake` or `picar4wd` — which hardware adapter to use |
| `stop_distance_cm` | planning | Obstacle closer than this triggers the stop/reverse/turn sequence |
| `cruise_speed` | planning | Forward speed while cruising |
| `turn_duration_s` | planning | How long the turn lasts |
| `range_stale_ms` | planning | Range older than this is treated as a fault |
| `cmd_timeout_ms` | action | No `/cmd_vel` for this long → motors stop (default 500) |

## Repository layout

```
core/                pure-Python picar_core package + tests
ros/picar_ros/       ROS 2 (ament_python) package + ROS tests
docker/              Dockerfile and compose.yaml
scripts/             helper scripts (update.sh, hardware checks)
docs/                architecture, state machine and deployment docs
.github/workflows/   CI
```

## Quick start

### Develop on Windows (no robot, no ROS)

Project and dependencies are managed with [uv](https://docs.astral.sh/uv/). The workspace pins
Python 3.12 in `.python-version`, so uv installs that version if you do not have it.

```powershell
uv sync                                        # creates .venv and installs everything
uv run --package picar-core pytest core        # run the picar_core tests
```

The ROS package (`ros/picar_ros`) is built with colcon in the container, not with uv.
The dev container (`.devcontainer/`) sets all of this up in VS Code.

### Test the ROS layer (dev container)

The `picar_ros` tests (`ros/picar_ros/test/`) need ROS 2, so they run in the dev container
with the ROS Python, separate from the core tests above. They cover each node with fake
hardware and launch the whole stack with `picar.launch.py`.

```bash
scripts/test_ros.sh              # builds picar_ros, then runs its tests
scripts/test_ros.sh -k planning  # extra arguments go to pytest
```

### Deploy to the Pi

The image is built on the dev machine and copied to the Pi by hand. Nothing is pushed to a
registry and nothing is deployed automatically. Replace `PI_USER@PI_HOST` below with the Pi's
SSH login (for example `pi@192.168.1.50`).

**Prerequisites**

- Raspberry Pi 3B+ running 64-bit Raspberry Pi OS (Lite)
- Docker with the compose plugin, and your user in the `docker` group
- I2C enabled (`sudo raspi-config`, Interface Options)
- Docker Desktop running on the dev machine, with buildx available

**1. Check the Pi is ready** (on the Pi, from the repo directory)

```bash
./scripts/setup_pi.sh    # changes nothing; lists anything to fix
```

**2. Build the arm64 image** (on the dev machine, from the repo root)

```powershell
docker buildx create --use --name picar 2>$null
docker buildx build --platform linux/arm64 -f docker/Dockerfile `
    -t ghcr.io/ram3917/picar_robot:latest --load .
```

The tag matches the image name in `docker/compose.yaml`, so compose uses it without edits.
Building for arm64 on an x86 machine runs under emulation and can take a long time.

**3. Save the image to a file and copy it to the Pi**

```powershell
docker save -o picar.tar ghcr.io/ram3917/picar_robot:latest
scp picar.tar PI_USER@PI_HOST:~/
```

The file is several hundred megabytes, so the copy takes a while over Wi-Fi.

**4. Load the image on the Pi**

```bash
docker load -i ~/picar.tar
rm ~/picar.tar          # optional: free the space
```

**5. Get the deployment files onto the Pi**

Copy the `docker/` folder, or check out this branch with `git pull`. The Pi needs
`docker/compose.yaml` to match the image.

**6. Start the stack** (on the Pi, from the repo root)

```bash
cd docker
docker compose up -d --pull never     # use the local image; do not contact the registry
docker compose ps                     # the picar container should be "running"
docker compose logs -f picar          # the robot should start DISARMED
```

**Updating or rolling back**

Repeat steps 2 to 6 with a new image. To stop the stack, run `docker compose down` in `docker/`.

Note that `scripts/update.sh` pulls from the registry and will fail while no image is published
there. Use the steps above until a registry is set up again.

See [docs/deployment.md](docs/deployment.md) for the planned registry-based pipeline.

## Documentation

- [Architecture](docs/architecture.md)
- [Deployment](docs/deployment.md)
- State machine: [docs/state-machine.puml](docs/state-machine.puml)

## Roadmap

Work is tracked as [GitHub issues](https://github.com/ram3917/PiCar_Robot/issues).

1. **MVP** — ultrasonic obstacle avoidance on ROS 2 in Docker, released as `v0.1.0`
2. **Post-MVP**
   - Greyscale sensor: edge/cliff detection, then line following
   - Wheel-speed odometry and relative goals (`/goal_pose`)
   - ML goal-seeking policy behind a safety supervisor
   - rosbag2 data logging
   - Unity teleop, digital twin and simulator
   - Camera stream

## License

[Apache-2.0](LICENSE)
