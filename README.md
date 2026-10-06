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
- One image, built for `linux/arm64` in CI, pulled onto the Pi with one command
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
ros/picar_ros/       ROS 2 (ament_python) package
docker/              Dockerfile and compose.yaml
scripts/             helper scripts (update.sh, hardware checks)
docs/                architecture, state machine and deployment docs
.github/workflows/   CI
```

## Quick start

### Develop on Windows (no robot, no ROS)

Needs Python 3.10 or newer.

```powershell
python -m venv .venv
.venv\Scripts\activate
pip install -e "core[test]"
pytest core
```

### Deploy to the Pi

> Not available yet — the image, `compose.yaml` and `update.sh` arrive with MVP-11 to MVP-14.

The Pi needs a 64-bit OS, Docker with the compose plugin, and I2C enabled. Then:

```bash
./scripts/update.sh                    # pull the latest image and restart
IMAGE_TAG=v0.1.0 ./scripts/update.sh   # roll back to a tagged release
```

See [docs/deployment.md](docs/deployment.md).

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
