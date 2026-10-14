# PiCar_Robot — ticket plan

Labels: `mvp` / `post-mvp`, `afk` (agent can do unattended) / `hitl` (human feedback or physical work needed), plus an area label.


## MVP

| ID | Title | Mode | Blocked by |
|---|---|---|---|
| MVP-01 | Scaffold repo layout, .gitignore and README | 🤖 AFK | — |
| MVP-02 | Architecture, state machine and deployment docs (Markdown + PlantUML) | 🤖 AFK | MVP-01 |
| MVP-03 | picar_core: hardware interface + FakeRobot | 🤖 AFK | MVP-01 |
| MVP-04 | picar_core: reactive obstacle-avoidance state machine | 🤖 AFK | MVP-03 |
| MVP-05 | picar_core: command watchdog | 🤖 AFK | MVP-03 |
| MVP-06 | picar_core: SunFounder picar-4wd hardware adapter | 🧑 Human needed | MVP-03, MVP-15 |
| MVP-07 | picar_ros: perception node (/ultrasonic/range) | 🤖 AFK (superseded by v0.2) | MVP-03 |
| MVP-08 | picar_ros: planning node (avoider + /arm + /robot_state) | 🤖 AFK (superseded by v0.2) | MVP-04, MVP-07 |
| MVP-09 | picar_ros: action node (motor control + watchdog) | 🤖 AFK (superseded by v0.2) | MVP-05, MVP-03 |
| MVP-10 | Launch file + params YAML | 🤖 AFK | MVP-07, MVP-08, MVP-09 |
| MVP-11 | Dockerfile + compose.yaml (arm64, ros:jazzy-ros-base) | 🤖 AFK | MVP-10 |
| MVP-12 | CI: test core + build/push arm64 image to GHCR | 🧑 Human needed | MVP-03, MVP-11 |
| MVP-13 | scripts/update.sh — one-command update + rollback | 🤖 AFK | MVP-11 |
| MVP-14 | Pi first-time setup (64-bit OS, Docker, I2C, compose) | 🧑 Human needed | MVP-12, MVP-13 |
| MVP-15 | Wire motors and HAT; hardware bring-up | 🧑 Human needed | — |
| MVP-16 | Tune avoidance parameters on the real robot | 🧑 Human needed | MVP-06, MVP-14 |
| MVP-17 | MVP acceptance test + release v0.1.0 | 🧑 Human needed | MVP-16, MVP-02 |

## v0.2

Replaces the MVP's perception, planning and action nodes (MVP-07, MVP-08, MVP-09) with a
controller that owns the hardware and a motion planner behind fixed messages. The cause was that
two processes cannot both import `picar_4wd`. See [docs/architecture.md](docs/architecture.md).

| ID | Title | Mode | Status |
|---|---|---|---|
| V02-01 | picar_core: per-motor power, `MotionPlan`, `SensorFrame` | 🤖 AFK | Done, tested |
| V02-02 | picar_core: `ReactivePlanner` (2 s plan of 10 steps) | 🤖 AFK | Done, tested |
| V02-03 | picar_core: `MotorGovernor` (plan playback, disarm, heartbeat, hard stop, blind stop) | 🤖 AFK | Done, tested |
| V02-04 | picar_interfaces: `SensorFrame`, `MotionPlan`, `MotorPower` messages | 🤖 AFK | Written, not built |
| V02-05 | picar_ros: controller node (sole hardware owner, `/arm`, safety) | 🤖 AFK | Written, not run on ROS |
| V02-06 | picar_ros: motion_planner node | 🤖 AFK | Written, not run on ROS |
| V02-07 | Docker and launch: rpi-lgpio, `/dev/gpiochip0`, rosbag2 recording, `data/` volume | 🤖 AFK | Written, not built |
| V02-08 | Run `scripts/test_ros.sh` in the dev container and fix what it finds | 🧑 Human needed | Open |
| V02-09 | Build and deploy 0.2.0; check frame rate, arm, hard stop, planner heartbeat on the robot | 🧑 Human needed | Open, blocked by V02-08 |
| V02-10 | Check what the ultrasonic library returns on an open path (-2 may mean clear, not invalid) | 🧑 Human needed | Open |

## Post-MVP

| ID | Title | Mode | Blocked by |
|---|---|---|---|
| POST-01 | Greyscale sensor: edge/cliff detection in the controller | 🧑 Human needed | V02-09 |
| POST-02 | Decide: line-following mode and how it coexists with avoidance | 🧑 Human needed | POST-01 |
| POST-03 | Wheel-speed odometry (/odom) | 🧑 Human needed | MVP-17 |
| POST-04 | Goal input: relative (x, y) goal via /goal_pose | 🤖 AFK | POST-03 |
| POST-05 | Decide ML approach for ultrasonic goal-seeking | 🧑 Human needed | POST-04 |
| POST-06 | Decide safety arbitration: ML policy vs safety supervisor | 🧑 Human needed | POST-05 |
| POST-07 | Implement ML goal-seeking policy behind the Policy interface | 🧑 Human needed | POST-05, POST-06 |
| POST-08 | Optional rosbag2 data logging | 🤖 AFK | MVP-17 |
| POST-09 | Unity integration: teleop + digital twin | 🧑 Human needed | MVP-17 |
| POST-10 | Unity as simulator for the same ROS interfaces | 🧑 Human needed | POST-09 |
| POST-11 | Camera stream | 🧑 Human needed | POST-09 |

---

# Ticket details


## [MVP-01] Scaffold repo layout, .gitignore and README

**Mode:** 🤖 AFK  
**Milestone:** MVP

Create the agreed layout: `core/`, `ros/picar_ros/`, `docker/`, `scripts/`, `docs/`, `.github/workflows/`.

- `.gitignore` for Python, colcon (`build/ install/ log/`), venvs, IDE files, `.env`
- `README.md`: project goal, MVP scope + out-of-scope list, node/topic/param interface table, quick start (dev on Windows, deploy to Pi), roadmap

**Acceptance**
- [ ] Layout exists with placeholder `__init__.py` / `package.xml` where needed
- [ ] README contains the interface table (`/ultrasonic/range`, `/cmd_vel`, `/robot_state`, `/arm`, params)


## [MVP-02] Architecture, state machine and deployment docs (Markdown + PlantUML)

**Mode:** 🤖 AFK  
**Milestone:** MVP

Under `docs/`:
- `architecture.md` + `architecture.puml` — component diagram: perception → planning → action, core vs ROS layer, hardware adapter
- `state-machine.puml` — DISARMED / CRUISE / STOP / REVERSE / TURN / FAULT with transitions
- `deployment.md` + `deployment.puml` — push → GitHub Actions → GHCR (arm64) → `update.sh` on Pi; rollback via `IMAGE_TAG`

**Acceptance**
- [ ] All `.puml` files render without errors (CI or local `plantuml -checkonly`)
- [ ] Docs reference the same topic/param names as the README

**Blocked by:** MVP-01 (Scaffold repo layout, .gitignore and README)


## [MVP-03] picar_core: hardware interface + FakeRobot

**Mode:** 🤖 AFK  
**Milestone:** MVP

Pure-Python (no ROS imports) package `picar_core` with a `pyproject.toml` (pip installable).

- `RobotHardware` protocol: `set_wheel_speeds(left, right)`, `stop()`, `read_distance_cm() -> float | None`, `set_servo_angle(deg)`
- `FakeRobot` for tests: scripted distances, records commands

**Acceptance**
- [ ] `pip install -e core[test] && pytest` passes on Windows and Linux
- [ ] No `rclpy` import anywhere under `core/`

**Blocked by:** MVP-01 (Scaffold repo layout, .gitignore and README)


## [MVP-04] picar_core: reactive obstacle-avoidance state machine

**Mode:** 🤖 AFK  
**Milestone:** MVP

Implement the reactive avoider: DISARMED → (arm) → CRUISE → obstacle `< stop_distance_cm` → STOP → REVERSE → TURN (`turn_duration_s`) → CRUISE. Servo fixed forward.

- Pure function/class: `step(observation, now) -> (Command(linear, angular), State)`
- Range stale (`> range_stale_ms`) or invalid → zero command + FAULT
- Disarm from any state → zero command + DISARMED
- All thresholds/speeds come from a config dataclass (mapped to ROS params later)

**Acceptance**
- [ ] pytest covers every transition incl. stale/invalid range and disarm-while-moving
- [ ] Runs against `FakeRobot` in a simple simulated loop test

**Blocked by:** MVP-03 (picar_core: hardware interface + FakeRobot)


## [MVP-05] picar_core: command watchdog

**Mode:** 🤖 AFK  
**Milestone:** MVP

Watchdog used by the action node: if no command received within `cmd_timeout_ms` (default 500), output stop.

**Acceptance**
- [ ] Unit tests with an injected clock: fresh cmd passes through, stale cmd → stop, recovery after new cmd

**Blocked by:** MVP-03 (picar_core: hardware interface + FakeRobot)


## [MVP-06] picar_core: SunFounder picar-4wd hardware adapter

**Mode:** 🧑 Human needed  
**Milestone:** MVP

Implement `RobotHardware` using SunFounder's `picar-4wd` library (motors, ultrasonic, servo locked forward). Import lazily so tests on Windows never touch it.

**Human needed:** verify on the real robot — motor directions, left/right mapping, distance readings sane (compare to a ruler), behaviour when sensor returns error values.

**Acceptance**
- [ ] Small manual script `scripts/hw_check.py` drives each side briefly and prints 20 distance readings
- [ ] Results noted in the PR (which values were wrong and how mapped)

**Blocked by:** MVP-03 (picar_core: hardware interface + FakeRobot), MVP-15 (Wire motors and HAT; hardware bring-up)


## [MVP-07] picar_ros: perception node (/ultrasonic/range)

**Mode:** 🤖 AFK  
**Milestone:** MVP

`ament_python` package `picar_ros`. Perception node publishes `sensor_msgs/Range` on `/ultrasonic/range` at 10 Hz (`radiation_type=ULTRASOUND`, min/max range set, invalid → `inf`/NaN per REP 117).

- Hardware injected via parameter `hardware:=fake|picar4wd` so it runs without a robot

**Acceptance**
- [ ] `ros2 topic hz /ultrasonic/range` ≈ 10 Hz with fake hardware in the container

**Blocked by:** MVP-03 (picar_core: hardware interface + FakeRobot)


## [MVP-08] picar_ros: planning node (avoider + /arm + /robot_state)

**Mode:** 🤖 AFK  
**Milestone:** MVP

Wraps the core state machine.

- Sub `/ultrasonic/range`; pub `/cmd_vel` (`geometry_msgs/Twist`) at 20 Hz while ARMED (zeros when stopped); publishes **nothing** while DISARMED so `teleop_twist_keyboard` can drive (action watchdog keeps motors stopped otherwise); and `/robot_state` (`std_msgs/String`)
- Service `/arm` (`std_srvs/SetBool`); boots DISARMED
- Params: `stop_distance_cm`, `cruise_speed`, `turn_duration_s`, `range_stale_ms`

**Acceptance**
- [ ] With fake hardware: arm → `/cmd_vel` non-zero; fake obstacle → STOP/REVERSE/TURN sequence visible on `/robot_state`

**Blocked by:** MVP-04 (picar_core: reactive obstacle-avoidance state machine), MVP-07 (picar_ros: perception node (/ultrasonic/range))


## [MVP-09] picar_ros: action node (motor control + watchdog)

**Mode:** 🤖 AFK  
**Milestone:** MVP

Sub `/cmd_vel`, convert Twist → left/right wheel speeds (skid-steer), apply via `RobotHardware`. Uses core watchdog: no `/cmd_vel` for `cmd_timeout_ms` → stop.

**Acceptance**
- [ ] Killing the planning node stops motors within 500 ms (verified with fake hardware logs)
- [ ] Works with `teleop_twist_keyboard`

**Blocked by:** MVP-05 (picar_core: command watchdog), MVP-03 (picar_core: hardware interface + FakeRobot)


## [MVP-10] Launch file + params YAML

**Mode:** 🤖 AFK  
**Milestone:** MVP

`picar.launch.py` starts perception, planning, action with `config/params.yaml`. Launch args: `hardware:=fake|picar4wd`.

**Acceptance**
- [ ] `ros2 launch picar_ros picar.launch.py hardware:=fake` runs all three nodes cleanly

**Blocked by:** MVP-07 (picar_ros: perception node (/ultrasonic/range)), MVP-08 (picar_ros: planning node (avoider + /arm + /robot_state)), MVP-09 (picar_ros: action node (motor control + watchdog))


## [MVP-11] Dockerfile + compose.yaml (arm64, ros:jazzy-ros-base)

**Mode:** 🤖 AFK  
**Milestone:** MVP

- `docker/Dockerfile`: `ros:jazzy-ros-base`, install `picar_core`, `picar-4wd` lib, `teleop_twist_keyboard`, colcon build `picar_ros`; entrypoint runs the launch file
- `docker/compose.yaml`: `network_mode: host`, `restart: unless-stopped`, devices `/dev/i2c-1`, `/dev/gpiomem` (avoid `privileged` unless proven necessary), image `ghcr.io/ram3917/picar_robot:${IMAGE_TAG:-latest}`

**Acceptance**
- [ ] Image builds for `linux/arm64`
- [ ] `docker compose run` with `hardware:=fake` works on an x86 dev box (amd64 build) for smoke testing

**Blocked by:** MVP-10 (Launch file + params YAML)


## [MVP-12] CI: test core + build/push arm64 image to GHCR

**Mode:** 🧑 Human needed  
**Milestone:** MVP

GitHub Actions:
- `test.yml`: pytest on `picar_core` (matrix `ubuntu-latest`, `windows-latest`)
- `image.yml`: on push to `main` and tags `v*` → build `linux/arm64` (native arm runner, fallback QEMU/buildx) → push to GHCR with tags `latest`, `sha-<short>`, `vX.Y.Z`

**Human needed:** after first push, set the GHCR package visibility to public (or create a read token for the Pi).

**Acceptance**
- [ ] Green pipeline on `main`; image visible at `ghcr.io/ram3917/picar_robot`

**Blocked by:** MVP-03 (picar_core: hardware interface + FakeRobot), MVP-11 (Dockerfile + compose.yaml (arm64, ros:jazzy-ros-base))


## [MVP-13] scripts/update.sh — one-command update + rollback

**Mode:** 🤖 AFK  
**Milestone:** MVP

`./scripts/update.sh` on the Pi: `docker compose pull && docker compose up -d`, prints running image tag. `IMAGE_TAG=v0.x ./scripts/update.sh` rolls back.

**Acceptance**
- [ ] Idempotent; fails loudly if Docker or compose missing
- [ ] Documented in `docs/deployment.md`

**Blocked by:** MVP-11 (Dockerfile + compose.yaml (arm64, ros:jazzy-ros-base))


## [MVP-14] Pi first-time setup (64-bit OS, Docker, I2C, compose)

**Mode:** 🧑 Human needed  
**Milestone:** MVP

Done by a human on the Pi (a guided `scripts/setup_pi.sh` / wizard can be generated AFK).

- [ ] Confirm `uname -m` = `aarch64`; if `armv7l`, reflash Raspberry Pi OS 64-bit (Lite)
- [ ] Enable I2C (`raspi-config`), install Docker + compose plugin, add user to `docker` group
- [ ] Clone repo (or copy `docker/compose.yaml` + `scripts/update.sh`), run `update.sh`
- [ ] Record findings in `docs/deployment.md`

**Blocked by:** MVP-12 (CI: test core + build/push arm64 image to GHCR), MVP-13 (scripts/update.sh — one-command update + rollback)


## [MVP-15] Wire motors and HAT; hardware bring-up

**Mode:** 🧑 Human needed  
**Milestone:** MVP

Physical work: wire the 4 motors to the PiCar HAT per SunFounder docs, mount ultrasonic + servo, charge batteries. Run SunFounder's own examples once to confirm the hardware works before our stack touches it.

**Acceptance**
- [ ] All 4 wheels spin in the commanded direction
- [ ] Ultrasonic returns plausible distances


## [MVP-16] Tune avoidance parameters on the real robot

**Mode:** 🧑 Human needed  
**Milestone:** MVP

Drive in a test area; tune `stop_distance_cm`, `cruise_speed`, `turn_duration_s` in `params.yaml`. Note floor type and battery level.

**Acceptance**
- [ ] 5 consecutive minutes of avoidance in a cluttered room without collision
- [ ] Final values committed with a short note in the PR

**Blocked by:** MVP-06 (picar_core: SunFounder picar-4wd hardware adapter), MVP-14 (Pi first-time setup (64-bit OS, Docker, I2C, compose))


## [MVP-17] MVP acceptance test + release v0.1.0

**Mode:** 🧑 Human needed  
**Milestone:** MVP

Checklist on the real robot:
- [ ] Boots DISARMED after power-on (container auto-starts)
- [ ] `/arm true` via SSH → avoids obstacles; `/arm false` → stops
- [ ] Kill planning node → motors stop ≤ 500 ms
- [ ] Unplug/cover ultrasonic → FAULT + stop
- [ ] `teleop_twist_keyboard` drives the robot while DISARMED (planning silent on `/cmd_vel`)
- [ ] Tag `v0.1.0`; Pi updated via `update.sh`; rollback tested

**Blocked by:** MVP-16 (Tune avoidance parameters on the real robot), MVP-02 (Architecture, state machine and deployment docs (Markdown + PlantUML))


## [POST-01] Greyscale sensor: edge/cliff detection in the controller

**Mode:** 🧑 Human needed  
**Milestone:** Post-MVP

The three greyscale values already arrive in every `SensorFrame` (v0.2) and the planner can read them. Add an edge flag to the frame and make the controller refuse forward drive at an edge, as it does for the ultrasonic hard stop, so it holds for any planner.

**Human needed:** calibrate thresholds on your floor surfaces and at a real table edge.

**Blocked by:** V02-09 (Build and deploy 0.2.0)


## [POST-02] Decide: line-following mode and how it coexists with avoidance

**Mode:** 🧑 Human needed  
**Milestone:** Post-MVP

Design decision (grilling session): separate mode vs combined behaviour, mode switching interface, priority vs obstacle avoidance.

**Blocked by:** POST-01 (Greyscale sensor: edge/cliff detection in the controller)


## [POST-03] Wheel-speed odometry (/odom)

**Mode:** 🧑 Human needed  
**Milestone:** Post-MVP

Read the kit's speed sensors, publish `nav_msgs/Odometry` on `/odom` + TF `odom → base_link`. Heading drift accepted (no IMU, by decision).

**Human needed:** measure wheel diameter / track width; drive a 1 m square and record drift.

**Blocked by:** MVP-17 (MVP acceptance test + release v0.1.0)


## [POST-04] Goal input: relative (x, y) goal via /goal_pose

**Mode:** 🤖 AFK  
**Milestone:** Post-MVP

Accept `geometry_msgs/PoseStamped` on `/goal_pose` in the `odom` frame; the motion planner exposes goal distance/bearing in its observation (this will need new fields in `SensorFrame` or a second input message).

**Blocked by:** POST-03 (Wheel-speed odometry (/odom))


## [POST-05] Decide ML approach for ultrasonic goal-seeking

**Mode:** 🧑 Human needed  
**Milestone:** Post-MVP

Open decisions (grilling): learning approach (behaviour cloning vs RL in a 2D sim vs Unity ML-Agents), observation (single beam vs servo sweep), training/inference runtime on Pi 3B+, data needs.

**Blocked by:** POST-04 (Goal input: relative (x, y) goal via /goal_pose)


## [POST-06] Decide safety arbitration: ML policy vs safety supervisor

**Mode:** 🧑 Human needed  
**Milestone:** Post-MVP

Partly settled in v0.2: the controller's `MotorGovernor` always applies its limits (disarm, plan heartbeat, 10 cm hard stop, blind stop) to whatever plan it receives, so a learned planner cannot bypass them. Still open: whether those limits are enough for an ML policy (acceleration limits, edge stop), and how this is tested.

**Blocked by:** POST-05 (Decide ML approach for ultrasonic goal-seeking)


## [POST-07] Implement ML goal-seeking planner as a replacement motion_planner node

**Mode:** 🧑 Human needed  
**Milestone:** Post-MVP

Implement per POST-05/POST-06 decisions as a node that subscribes to `/controller/sensor_frame` and publishes `/planner/motion_plan` (10 steps of 4 motor powers), replacing `motion_planner` in the launch file. The message contract stays fixed. It may run on the Pi or off-board. Human needed for real-robot evaluation.

**Blocked by:** POST-05 (Decide ML approach for ultrasonic goal-seeking), POST-06 (Decide safety arbitration: ML policy vs safety supervisor)


## [POST-08] Optional rosbag2 data logging

**Mode:** 🤖 AFK  
**Milestone:** Post-MVP

Done in v0.2 (V02-07, not yet run on the robot): launch arg `record:=true` (container env `PICAR_RECORD=true`) records `/controller/sensor_frame` and `/planner/motion_plan` into the `data/` volume. Remaining: add `/odom` once POST-03 exists, and check the recordings are usable for training.

**Blocked by:** MVP-17 (MVP acceptance test + release v0.1.0)


## [POST-09] Unity integration: teleop + digital twin

**Mode:** 🧑 Human needed  
**Milestone:** Post-MVP

Decide transport (ROS-TCP-Connector vs rosbridge vs native DDS), then Unity scene showing robot state + range, sending plans (v0.2 has no `/cmd_vel`; manual driving would publish `MotionPlan` messages) and arm/disarm on `/arm`. Add operator heartbeat dead-man switch (the controller's plan heartbeat already stops the motors).

**Blocked by:** MVP-17 (MVP acceptance test + release v0.1.0)


## [POST-10] Unity as simulator for the same ROS interfaces

**Mode:** 🧑 Human needed  
**Milestone:** Post-MVP

Simulated PiCar + ultrasonic in Unity publishing `/controller/sensor_frame` and consuming `/planner/motion_plan`, so the motion planner runs unchanged in sim and on the real robot.

**Blocked by:** POST-09 (Unity integration: teleop + digital twin)


## [POST-11] Camera stream

**Mode:** 🧑 Human needed  
**Milestone:** Post-MVP

Decide resolution/compression/transport for the Pi Camera on a 3B+ and integrate into Unity view.

**Blocked by:** POST-09 (Unity integration: teleop + digital twin)
