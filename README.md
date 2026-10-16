# PiCar_Robot

An autonomous obstacle-avoiding robot built on the SunFounder **PiCar-4WD** kit and a
Raspberry Pi 3B+, running **ROS 2 Jazzy** inside Docker.

One ROS 2 package, [picar/](picar/), holds everything: two nodes, a **controller** that owns the
hardware and a **planner**, talking over standard ROS messages. The planner drives the robot with
`geometry_msgs/Twist` on `/cmd_vel`, so any node that publishes there (teleop, a learned model)
can replace it.

## v0.3 scope

The ultrasonic sensor sweeps the room and publishes a scan, while the robot stays still. Driving
and avoidance come back in a later version, once the sweep has been tested on the robot.

**In scope**

- Ultrasonic servo sweep from −90° to +90° and back, one step per 30 ms. Each pass publishes one
  `sensor_msgs/LaserScan` on `/controller/scan`
- Only readings within 15° of straight ahead count as the forward range, so the drive safety rules
  never act on a reading from the side
- The motors stay stopped. The planner's `drive` is false by default, so nothing moves unless it is
  turned on
- Reactive avoidance code (cruise → stop → reverse → turn → cruise) stays in the planner, off by
  default
- The controller is the only process that touches the hardware, and enforces the safety limits
- A `drive` parameter on the planner is the on switch; it is **off** by default
- Safety stops: no new command for 500 ms, forward range under 25 cm, no forward range for 1.5 s
- rosbag2 recording of sensor topics and commands (`PICAR_RECORD=true`)
- Runs without a robot (`hardware:=fake`) for development and smoke tests

**Out of scope for v0.3**

- Edge/cliff stops and line following (the greyscale values are published, but unused)
- Wheel-speed odometry, `/odom`, goal-seeking
- The machine-learned planner itself
- Unity teleop / digital twin / simulator
- Camera stream
- Driving while sweeping, mapping, SLAM, IMU

## Interfaces

### Nodes

| Node | Role | Subscribes | Publishes |
|---|---|---|---|
| controller | Owns all hardware, sweeps the servo, applies the command to the motors, enforces safety | `/cmd_vel` | `/controller/range`, `/controller/scan`, `/controller/greyscale`, `/joint_states` |
| planner | Plans the next command from the latest range, while `drive` is true | `/controller/range` | `/cmd_vel` |

### Topics

| Name | Type | Notes |
|---|---|---|
| `/controller/range` | `sensor_msgs/Range` | The forward range, about every 30 ms while the servo is forward; only readings within 15° of forward are published here (a target); invalid readings are NaN (REP 117) |
| `/controller/scan` | `sensor_msgs/LaserScan` | One sweep, published at the end of each pass (about every 1.1 s). Angles in radians, positive to the left; invalid or missing readings are NaN, clear is +inf |
| `/controller/greyscale` | `std_msgs/UInt16MultiArray` | The three greyscale ADC values |
| `/joint_states` | `sensor_msgs/JointState` | Servo angle (`position`) and applied motor power in percent (`effort`) |
| `/cmd_vel` | `geometry_msgs/Twist` | `linear.x` and `angular.z`, normalized to [-1, 1]; `angular.z` > 0 turns counter-clockwise |

See [docs/architecture.md](docs/architecture.md) for the overview and the safety limits.

### Parameters

| Parameter | Node | Meaning |
|---|---|---|
| `hardware` | controller | `fake` or `picar4wd` — which hardware adapter to use |
| `sweep_enabled` | controller | Sweep the ultrasonic servo (default `false` in the node, `true` in `params.yaml`) |
| `sweep_min_deg`, `sweep_max_deg`, `sweep_step_deg` | controller | The sweep: −90 to 90 in 5° steps |
| `hard_stop_m` | controller | Forward drive stops closer than this (default 0.25 in `params.yaml`) |
| `range_stale_ms` | controller | No forward range for this long stops forward drive (default 1500 in `params.yaml`: one sweep pass) |
| `cmd_timeout_ms` | controller | No new command for this long stops all motors (default 500) |
| `drive` | planner | The on switch (default `false`) |
| `stop_distance_m` | planner | Obstacle closer than this starts the stop/reverse/turn manoeuvre |
| `cruise_speed`, `turn_duration_s`, ... | planner | Speeds and manoeuvre timing |

## Repository layout

```
picar/               the ROS 2 (ament_python) package: nodes, launch files, config, tests
docker/              base image Dockerfile, compose.yaml, entrypoint, install_picar.sh, smoke test, Pi check script
docs/                architecture overview and diagram
.devcontainer/       VS Code dev container, built from docker/Dockerfile
.github/workflows/   CI
```

## Quick start

### Develop (dev container)

Open the repo in VS Code and choose *Reopen in Container*. The container has ROS 2 Jazzy, colcon,
rviz2, and builds the package for you. There is no robot, so use
`hardware:=fake`.

```bash
source picar/install/setup.bash        # after the first build, in each new terminal
pytest picar                           # unit tests, node tests and the launch test
```

After editing the package, rebuild with `cd picar && colcon build --symlink-install`.

### Test by hand (dev container)

The dev container has a desktop in your browser at <http://localhost:6080> (password
`noPassword`), so RViz works from a Windows host without an X server. One launch file starts the
stack on fake hardware and RViz, which shows the ultrasonic cone and the four motor powers (green
forward, red reverse):

```bash
ros2 launch picar manual_test.launch.py                              # a clear 1 m path: the planner cruises
ros2 launch picar manual_test.launch.py ultrasonic_m:=0.15           # obstacle at 15 cm: stop, reverse, turn
ros2 launch picar manual_test.launch.py ultrasonic_m:=1.0,0.5,0.15   # scripted readings; the last repeats
ros2 launch picar manual_test.launch.py drive:=false                 # the robot stays still
```

Open <http://localhost:6080> to see RViz. Drive by hand with
`ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}}"`. The controller's
25 cm hard stop and 500 ms command timeout are visible in the motor bars too.

The package has two launch files: `manual_test.launch.py` (this) and `picar.launch.py`, which the
Pi image runs.

### Deploy to the Pi

The Pi runs two things, each with its own cadence:

- **The base image** (rare: an OS, ROS or picar-4wd change). It holds Ubuntu, ROS 2 Jazzy, `uv` and
  picar-4wd. It does not hold picar. You build it here and copy it to the Pi by hand.
- **The picar release** (often). A `v<version>` tag on `main` is published as a GitHub Release with
  the wheel. The Pi installs it into a volume with `install_picar.sh`, and runs it.

**Prerequisites**

- Raspberry Pi 3B+ running 64-bit Raspberry Pi OS (Lite), on Wi-Fi
- Docker with the compose plugin, and your user in the `docker` group
- I2C enabled permanently: `dtparam=i2c_arm=on` in `/boot/firmware/config.txt` (or
  `sudo raspi-config nonint do_i2c 0`), then a reboot. `/dev/i2c-1` must exist
- Docker Desktop running on the dev machine, with buildx available

**1. Check the Pi is ready** (on the Pi, from the repo directory)

```bash
./docker/check_pi.sh    # changes nothing; lists anything to fix
```

**2. Copy the base image** (only when it changes; on the dev machine, from the repo root)

```powershell
docker buildx create --use --name picar 2>$null
docker buildx build --platform linux/arm64 -f docker/Dockerfile -t picar:latest --load .
docker save -o picar.tar picar:latest
scp picar.tar PI_USER@PI_HOST:~/
```

On the Pi, load it and free the file:

```bash
docker load -i ~/picar.tar && rm ~/picar.tar
```

**3. Get the deployment files onto the Pi**

Copy the `docker/` folder, or check out `main` with `git pull`.

**4. Choose the release** (on the Pi, in `docker/`, first time only)

```bash
cp .env.example .env     # then edit .env and set PICAR_VERSION, e.g. PICAR_VERSION=0.3.0
```

**5. Install the release** (only when `PICAR_VERSION` changes)

```bash
docker compose run --rm --entrypoint /ws/install_picar.sh picar
```

It downloads `picar-<version>-py3-none-any.whl` and its `.sha256` from GitHub Release `v<version>`,
checks the checksum, and installs into the `picar-install` volume. The old release is deleted only
after the new one installs. If the download or checksum fails, the old release stays.

**6. Start the stack** (on the Pi, in `docker/`)

```bash
docker compose up -d                  # runs the installed release; the robot stays still
docker compose logs -f picar
```

`docker compose up -d` does not download anything. The container restarts on reboot and runs the
installed release again. The robot only drives once `PICAR_DRIVE=true` is set in `docker/.env` and
the stack is recreated (`docker compose up -d`). Stop it with `docker compose down`.

**Publishing a release** (on the dev machine)

Merge to `main`, then tag the merge commit:

```bash
git tag v0.3.0 && git push origin v0.3.0
```

The `release` workflow runs the tests, builds the wheel with the version from the tag, and creates
the GitHub Release. A tag must be `v<major>.<minor>.<patch>` and point at a commit on `main`, or the
workflow stops. Pushes to `main` build nothing.

**Before merging: the smoke test** (on the dev machine, needs Docker)

```bash
docker/smoke_test.sh
```

It builds the base image and a wheel from this checkout, installs it with the same script the Pi
uses, runs the launch file on fake hardware, and checks the nodes come up and a sweep scan is
published. It does not touch the robot.

**Updating or rolling back**

Update: change `PICAR_VERSION` in `docker/.env`, then run steps 5 and 6. Roll back: set an earlier
version and run the same steps. Only the installed release is kept on the Pi, so a rollback
downloads that version again. It needs the Pi to be on the network.

## Documentation

- [Architecture](docs/architecture.md) and its diagram, [architecture.puml](docs/architecture.puml)

## Roadmap

Work is tracked as [GitHub issues](https://github.com/ram3917/PiCar_Robot/issues).

1. **MVP** — ultrasonic obstacle avoidance on ROS 2 in Docker, released as `v0.1.0`
2. **v0.2** — controller and planner nodes, one hardware owner, swappable planner
3. **v0.3** — ultrasonic servo sweep publishing `LaserScan`, motors still; release pipeline
   from tags, with the wheel installed on the Pi
4. **Later**
   - Greyscale sensor: edge/cliff detection, then line following
   - Wheel-speed odometry and relative goals (`/goal_pose`)
   - ML planner node publishing `/cmd_vel`
   - Unity teleop, digital twin and simulator
   - Camera stream

## License

[Apache-2.0](LICENSE)
