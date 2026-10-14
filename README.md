# PiCar_Robot

An autonomous obstacle-avoiding robot built on the SunFounder **PiCar-4WD** kit and a
Raspberry Pi 3B+, running **ROS 2 Jazzy** inside Docker.

One ROS 2 package, [picar/](picar/), holds everything: two nodes, a **controller** that owns the
hardware and a **planner**, talking over standard ROS messages. The planner drives the robot with
`geometry_msgs/Twist` on `/cmd_vel`, so any node that publishes there (teleop, a learned model)
can replace it.

## v0.2 scope

Drive around a room on its own, using only the ultrasonic sensor to avoid obstacles.

**In scope**

- Reactive avoidance: cruise → stop → reverse → turn → cruise
- Ultrasonic sensor fixed facing forward (servo locked), a range about every 30 ms
- The controller is the only process that touches the hardware, and enforces the safety limits
- A `drive` parameter on the planner is the on switch; it is **off** by default
- Safety stops: no new command for 500 ms, forward range under 10 cm, no valid range for 150 ms
- rosbag2 recording of sensor topics and commands (`PICAR_RECORD=true`)
- Runs without a robot (`hardware:=fake`) for development and smoke tests

**Out of scope for v0.2**

- Edge/cliff stops and line following (the greyscale values are published, but unused)
- Wheel-speed odometry, `/odom`, goal-seeking
- The machine-learned planner itself
- Unity teleop / digital twin / simulator
- Camera stream
- Servo sweeping, mapping, SLAM, IMU

## Interfaces

### Nodes

| Node | Role | Subscribes | Publishes |
|---|---|---|---|
| controller | Owns all hardware, applies the command to the motors, enforces safety | `/cmd_vel` | `/controller/range`, `/controller/greyscale`, `/joint_states` |
| planner | Plans the next command from the latest range, while `drive` is true | `/controller/range` | `/cmd_vel` |

### Topics

| Name | Type | Notes |
|---|---|---|
| `/controller/range` | `sensor_msgs/Range` | About every 30 ms (a target); invalid readings are NaN (REP 117) |
| `/controller/greyscale` | `std_msgs/UInt16MultiArray` | The three greyscale ADC values |
| `/joint_states` | `sensor_msgs/JointState` | Servo angle (`position`) and applied motor power in percent (`effort`) |
| `/cmd_vel` | `geometry_msgs/Twist` | `linear.x` and `angular.z`, normalized to [-1, 1]; `angular.z` > 0 turns counter-clockwise |

See [docs/architecture.md](docs/architecture.md) for the details, the safety limits and the full
parameter list.

### Parameters

| Parameter | Node | Meaning |
|---|---|---|
| `hardware` | controller | `fake` or `picar4wd` — which hardware adapter to use |
| `hard_stop_m` | controller | Forward drive stops closer than this (default 0.10) |
| `range_stale_ms` | controller | No valid range for this long stops forward drive (default 150) |
| `cmd_timeout_ms` | controller | No new command for this long stops all motors (default 500) |
| `drive` | planner | The on switch (default `false`) |
| `stop_distance_m` | planner | Obstacle closer than this starts the stop/reverse/turn manoeuvre |
| `cruise_speed`, `turn_duration_s`, ... | planner | Speeds and manoeuvre timing |

## Repository layout

```
picar/               the ROS 2 (ament_python) package: nodes, launch files, config, tests
docker/              Dockerfile, compose.yaml, entrypoint, Pi check script
docs/                architecture, state machine and deployment docs
.devcontainer/       VS Code dev container, built from docker/Dockerfile
.github/workflows/   CI
```

## Quick start

### Develop (dev container)

Open the repo in VS Code and choose *Reopen in Container*. The container has ROS 2 Jazzy, colcon,
rviz2, rqt and PlotJuggler, and builds the package for you. There is no robot, so use
`hardware:=fake`.

```bash
source picar/install/setup.bash        # after the first build, in each new terminal
pytest picar                           # unit tests, node tests and the launch test
```

After editing the package, rebuild with `cd picar && colcon build --symlink-install`.

### Watch the stack run (dev container)

The dev container has a desktop in your browser at <http://localhost:6080> (password
`noPassword`), so RViz and rqt work from a Windows host without an X server. Start the stack in
one terminal and the viewers in another:

```bash
ros2 launch picar picar.launch.py hardware:=fake drive:=true   # a clear 1 m path: the planner cruises
ros2 launch picar picar.launch.py hardware:=sim drive:=true    # a simulated wall: cruise, stop, reverse, turn, repeat
ros2 launch picar viz.launch.py                                 # RViz (range cone) + rqt topic, graph, plot
```

Open <http://localhost:6080> to see the windows. Add `plotjuggler:=true` to also start PlotJuggler.
Without `drive:=true` the robot stays still; switch it on while running:

```bash
ros2 param set /planner drive true
ros2 topic echo /cmd_vel
```

To see the avoidance manoeuvre, run the nodes with an obstacle at 15 cm instead of the launch file:

```bash
ros2 run picar controller_node --ros-args -p hardware:=fake -p "fake_ranges_m:=[0.15]" &
ros2 run picar planner_node --ros-args -p drive:=true
```

Motor power in the rqt plot then cycles stop, reverse, turn. The controller's 10 cm hard stop and
500 ms command timeout are visible too: set `drive` back to false, or stop the planner (`Ctrl+C`
it), and the power falls to zero.

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
./docker/check_pi.sh    # changes nothing; lists anything to fix
```

**2. Build the arm64 image** (on the dev machine, from the repo root)

```powershell
docker buildx create --use --name picar 2>$null
docker buildx build --platform linux/arm64 -f docker/Dockerfile `
    -t picar:0.2.0 --load .
```

The image name matches `docker/compose.yaml`; compose picks the tag from `IMAGE_TAG` (step 6).
Building for arm64 on an x86 machine runs under emulation and can take a long time.

**3. Save the image to a file and copy it to the Pi**

```powershell
docker save -o picar.tar picar:0.2.0
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
docker compose up -d --pull never     # use the local image; do not contact a registry
docker compose ps                     # the picar container should be "running"
docker compose logs -f picar          # the robot starts but does not drive
```

The robot only drives once `drive` is true. Put `PICAR_DRIVE=true` in `docker/.env` and recreate
the container (`docker compose up -d --pull never`), or switch it on without a restart as
described in [docs/deployment.md](docs/deployment.md).

**Updating or rolling back**

Repeat steps 2 to 6 with a new tag. To roll back, set `IMAGE_TAG` in `docker/.env` to the previous
tag, which is still on the Pi, and run `docker compose up -d --pull never`. To stop the stack,
run `docker compose down` in `docker/`.

## Documentation

- [Architecture](docs/architecture.md)
- [Deployment](docs/deployment.md)
- Planner manoeuvre: [docs/state-machine.puml](docs/state-machine.puml)

## Roadmap

Work is tracked as [GitHub issues](https://github.com/ram3917/PiCar_Robot/issues).

1. **MVP** — ultrasonic obstacle avoidance on ROS 2 in Docker, released as `v0.1.0`
2. **v0.2** — controller and planner nodes, one hardware owner, swappable planner
3. **Later**
   - Greyscale sensor: edge/cliff detection, then line following
   - Wheel-speed odometry and relative goals (`/goal_pose`)
   - ML planner node publishing `/cmd_vel`
   - Unity teleop, digital twin and simulator
   - Camera stream

## License

[Apache-2.0](LICENSE)
