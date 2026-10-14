# PiCar_Robot

An autonomous obstacle-avoiding robot built on the SunFounder **PiCar-4WD** kit and a
Raspberry Pi 3B+, running **ROS 2 Jazzy** inside Docker.

The robot's logic is split in two layers:

- **`picar_core`** — pure Python, no ROS imports. Hardware interface, reactive planner, and the
  plan playback with its safety limits. Runs and is tested on any machine, including Windows.
- **`picar_ros`** — a thin ROS 2 layer of two nodes, a **controller** that owns the hardware and
  a **motion planner**, wrapping the core. They talk over standard ROS messages.

## v0.2 scope

Drive around a room on its own, using only the ultrasonic sensor to avoid obstacles, with the
planner behind a fixed message contract so a learned model can replace it later.

**In scope**

- Reactive avoidance planned as a 2 s motor plan: cruise → stop → reverse → turn → cruise
- Ultrasonic sensor fixed facing forward (servo locked), a sensor frame about every 30 ms
- The controller is the only process that touches the hardware, and enforces the safety limits
- Arm / disarm over a ROS service; the robot always boots **disarmed**
- Safety stops: disarm, no new plan for 500 ms, forward range under 10 cm, no valid range for 150 ms
- rosbag2 recording of sensor frames and plans (`PICAR_RECORD=true`)
- One image, built for `linux/arm64` on the dev machine and copied to the Pi by hand
- Runs without a robot (`hardware:=fake`) for development and smoke tests

**Out of scope for v0.2**

- Edge/cliff stops and line following (the greyscale values are in every frame, but unused)
- Wheel-speed odometry, `/odom`, goal-seeking
- The machine-learned planner itself (only the contract that lets it be swapped in)
- Manual driving with `teleop_twist_keyboard` (dropped: the controller takes plans only)
- Unity teleop / digital twin / simulator
- Camera stream
- Servo sweeping, mapping, SLAM, IMU

## Interfaces

### Nodes

| Node | Role | Subscribes | Publishes / serves |
|---|---|---|---|
| controller | Owns all hardware, plays plans onto the motors, enforces safety | `/planner/motion_plan` | `/controller/range`, `/controller/greyscale`, `/joint_states`, `/controller/armed`, service `/arm` |
| motion_planner | Plans the next 2 s from the latest range | `/controller/range`, `/controller/armed` | `/planner/motion_plan` |

### Topics and services

| Name | Kind | Type | Notes |
|---|---|---|---|
| `/controller/range` | topic | `sensor_msgs/Range` | About every 30 ms (a target); invalid readings are NaN (REP 117) |
| `/controller/greyscale` | topic | `std_msgs/UInt16MultiArray` | The three greyscale ADC values |
| `/joint_states` | topic | `sensor_msgs/JointState` | Servo angle (`position`) and applied motor power in percent (`effort`) |
| `/controller/armed` | topic | `std_msgs/Bool` | Latched; whether the robot is armed |
| `/planner/motion_plan` | topic | `trajectory_msgs/JointTrajectory` | 5-10 Hz: 10 points from 0.2 s to 2.0 s, each four motor powers in `[-100, 100]`; nothing is published while disarmed |
| `/arm` | service | `std_srvs/SetBool` | `true` arms, `false` disarms; served by the controller |

All messages are standard ROS ones. See [docs/architecture.md](docs/architecture.md) for the
details and the safety limits.

### Parameters

| Parameter | Node | Meaning |
|---|---|---|
| `hardware` | controller | `fake` or `picar4wd` — which hardware adapter to use |
| `hard_stop_cm` | controller | Forward drive stops closer than this (default 10) |
| `range_stale_ms` | controller | No valid range for this long stops forward drive (default 150) |
| `plan_timeout_ms` | controller | No new plan for this long stops all motors (default 500) |
| `rate_hz` | motion_planner | Plans per second (default 10) |
| `stop_distance_cm` | motion_planner | Obstacle closer than this starts the stop/reverse/turn manoeuvre |
| `cruise_power`, `turn_duration_s`, ... | motion_planner | Motor powers and manoeuvre timing |

The full list is in [docs/architecture.md](docs/architecture.md).

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
scripts/test_ros.sh                   # builds picar_ros, then runs the tests
scripts/test_ros.sh -k motion_planner # extra arguments go to pytest
```

### Watch the stack run (dev container)

The dev container has rviz2, rqt and PlotJuggler, and a desktop in your browser at
<http://localhost:6080> (password `noPassword`). Rebuild the container once to get them. Start the
stack with fake hardware in one terminal, and open the viewers from a terminal in the desktop
(or any dev container terminal; `DISPLAY` is set for you):

```bash
source ros/install/setup.bash
ros2 launch picar_ros picar.launch.py hardware:=fake     # constant 100 cm: the planner cruises

ros2 service call /arm std_srvs/srv/SetBool "{data: true}"
rqt_graph                          # nodes and topics; Node Graph shows controller <-> motion_planner
plotjuggler                        # plot /joint_states effort (motor power) and /controller/range
rviz2 -d .devcontainer/picar.rviz  # the ultrasonic cone from /controller/range
rqt_console                        # node logs
ros2 topic echo /planner/motion_plan --once
```

To see the avoidance manoeuvre, run the nodes with an obstacle at 15 cm instead of the launch file:

```bash
ros2 run picar_ros controller_node --ros-args -p hardware:=fake -p "fake_distances_cm:=[15.0]" &
ros2 run picar_ros motion_planner_node
```

Armed, motor power in PlotJuggler then cycles stop, reverse, turn, as the plan ramps in. The
controller's 10 cm hard stop and 500 ms plan timeout are visible too: disarm, or stop the planner
(`Ctrl+C` it), and the power falls to zero.

### Deploy to the Pi

The image is built on the dev machine and copied to the Pi by hand. Nothing is pushed to a
registry and nothing is deployed automatically. Replace `PI_USER@PI_HOST` below with the Pi's
SSH login (for example `pi@192.168.1.50`).

**Prerequisites**

- Raspberry Pi 3B+ running 64-bit Raspberry Pi OS (Lite)
- Docker with the compose plugin, and your user in the `docker` group
- I2C enabled permanently: `dtparam=i2c_arm=on` in `/boot/firmware/config.txt` (or
  `sudo raspi-config nonint do_i2c 0`), then a reboot. `/dev/i2c-1` must exist
- Docker Desktop running on the dev machine, with buildx available

**1. Check the Pi is ready** (on the Pi, from the repo directory)

```bash
./scripts/setup_pi.sh    # changes nothing; lists anything to fix
```

**2. Build the arm64 image** (on the dev machine, from the repo root)

```powershell
docker buildx create --use --name picar 2>$null
docker buildx build --platform linux/arm64 -f docker/Dockerfile `
    -t ghcr.io/ram3917/picar_robot:0.2.0 --load .
```

The image name matches `docker/compose.yaml`; compose picks the tag from `IMAGE_TAG` (step 6).
Building for arm64 on an x86 machine runs under emulation and can take a long time.

**3. Save the image to a file and copy it to the Pi**

```powershell
docker save -o picar.tar ghcr.io/ram3917/picar_robot:0.2.0
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
echo "IMAGE_TAG=0.2.0" > .env           # which image tag compose runs
mkdir -p ../data                      # rosbag2 recordings are written here
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
- Planner manoeuvre: [docs/state-machine.puml](docs/state-machine.puml)

## Roadmap

Work is tracked as [GitHub issues](https://github.com/ram3917/PiCar_Robot/issues).

1. **MVP** — ultrasonic obstacle avoidance on ROS 2 in Docker, released as `v0.1.0`
2. **v0.2** — controller and motion planner nodes, one hardware owner, swappable planner
3. **Later**
   - Greyscale sensor: edge/cliff detection, then line following
   - Wheel-speed odometry and relative goals (`/goal_pose`)
   - ML planner node behind the same message contract
   - Unity teleop, digital twin and simulator
   - Camera stream

## License

[Apache-2.0](LICENSE)
