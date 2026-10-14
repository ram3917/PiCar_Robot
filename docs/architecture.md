# Architecture

Diagrams: [architecture.puml](architecture.puml) (components) and
[state-machine.puml](state-machine.puml) (the planner's avoidance manoeuvre).

## Two layers

| Layer | Package | Contains | Depends on |
|---|---|---|---|
| Core | `picar_core` ([core/](../core/)) | `RobotHardware` interface, `FakeRobot`, reactive planner, plan playback and safety limits | Python standard library only |
| ROS | `picar_ros` ([ros/picar_ros/](../ros/picar_ros/)), `picar_interfaces` ([ros/picar_interfaces/](../ros/picar_interfaces/)) | the controller and motion planner nodes, and the messages between them | `picar_core`, `rclpy` |

All decisions live in the core. It never imports ROS, so it is developed and tested on any
machine, including Windows. The ROS nodes only translate between topics, services and
parameters on one side and core calls on the other.

## Two nodes

```
Controller  --/controller/sensor_frame (SensorFrame, ~30 ms)-->  MotionPlanner
Controller  <--/planner/motion_plan   (MotionPlan, 5-10 Hz)----  MotionPlanner
```

| Node | Role | Subscribes | Publishes / serves |
|---|---|---|---|
| controller | Owns all hardware, plays plans onto the motors, enforces safety, owns arming | `/planner/motion_plan` | `/controller/sensor_frame`, service `/arm` |
| motion_planner | Plans the next 2 s from the latest sensor frame | `/controller/sensor_frame` | `/planner/motion_plan` |

**Why exactly one hardware owner.** SunFounder's `picar-4wd` library resets the HAT, claims every
motor and ultrasonic GPIO line, and writes the PWM registers when it is imported, and it keeps
unlocked global state. Two processes that import it fight over the same lines (the second one
fails with `GPIO not allocated`) and the second import would reset the first one's HAT. So the
controller is the only node that touches the robot; the planner is plain software.

**Swapping the planner.** The planner is replaced as a whole node: a node that subscribes to
`/controller/sensor_frame` and publishes `/planner/motion_plan` with the same message types.
Nothing else changes, and the controller's safety limits apply to whatever it sends, so a
learned model cannot bypass them.

## Messages

Defined in `picar_interfaces`:

| Message | Fields |
|---|---|
| `MotorPower` | `int8[4] power` in `[-100, 100]`: left front, right front, left rear, right rear |
| `SensorFrame` | `header`, `range_m` (REP 117: NaN invalid, -inf too close, +inf nothing in range), `range_valid`, `servo_angle_deg`, `uint16[3] greyscale`, `int8[4] last_motor_power`, `armed` |
| `MotionPlan` | `header`, `step_s`, `MotorPower[] steps` (v0.2: 10 steps of 0.2 s = 2 s) |

## Controller

Two worker threads do the timed work:

- **Sensor thread**, every `sensor_period_ms` (30): reads the ultrasonic sensor and the greyscale
  ADC and publishes a `SensorFrame`. 30 ms is a target: an ultrasonic read takes 10 to 20 ms,
  and every frame is timestamped so consumers can see the real rate.
- **Motor thread**, every `motor_tick_ms` (10): asks the `MotorGovernor` for the motor powers and
  writes them when they change.

The ultrasonic read is plain GPIO and runs unlocked. The greyscale ADC and the motors share the
HAT's I2C address, so those calls take one lock and never interleave.

### Plan playback

A plan plays from the moment it arrives, so clock differences between machines do not matter.
Each step is a keyframe at the start of its 200 ms slot; the controller ramps linearly from one
step to the next and holds the last one until the plan's 2 s horizon ends.

### Safety limits

These are enforced by the controller on every plan, whoever produced it:

| Rule | Behaviour |
|---|---|
| Boots disarmed | Nothing moves until `/arm` is called with `true` |
| Disarm wins | `/arm false` stops the motors within one motor tick and drops the plan |
| Plans while disarmed | Ignored |
| Planner heartbeat | No new plan for `plan_timeout_ms` (500) stops all motors, even if the old plan has time left |
| Hard stop | Forward drive is refused while the forward range is under `hard_stop_cm` (10) |
| Blind stop | Forward drive is refused after `range_stale_ms` (150) without a valid range |

"Forward" means the sum of the four motor powers is positive. When refused, every positive power
becomes zero and negative power is kept, so the robot can still reverse and spin in place.

Not enforced in v0.2: acceleration limits, edge detection from the greyscale sensors, and a stop
on I2C faults (a slow motor write is logged). If the controller process is killed hard or power
is cut, the HAT keeps its last PWM output until the next start resets it.

## Motion planner

`ReactivePlanner` (in `picar_core`) turns the latest frame into 10 steps. While the path is clear
the plan is ten cruise steps. An obstacle closer than `stop_distance_cm` starts a manoeuvre:
stop, reverse, turn, then cruise again (see [state-machine.puml](state-machine.puml)). The planner
keeps to the manoeuvre on every replan whatever the sensor says, so replanning does not make it
flap. A frame that is stale (`frame_stale_ms`) or has no valid distance gives a stop, and
nothing is planned while the robot is disarmed.

The v0.2 planner has no world model: the ultrasonic sensor is fixed facing forward and the plan is
dead-reckoned from commanded powers.

## Hardware adapter

The controller reaches the robot only through the `RobotHardware` interface:

| Method | Meaning |
|---|---|
| `set_motor_power(powers)` | Four ints in `[-100, 100]`; negative is reverse |
| `stop()` | Stop all motors |
| `read_distance_cm()` | Distance in centimetres, or `None` if the reading is invalid |
| `read_greyscale()` | The three greyscale ADC values |
| `set_servo_angle(deg)` | Point the ultrasonic sensor; `0` is straight ahead |

The `hardware` parameter selects the implementation:

- `fake` — `FakeRobot`, in memory. Plays back scripted distances and records commands. Used
  by the tests and for running the stack without a robot.
- `picar4wd` — adapter over SunFounder's `picar-4wd` library. It is imported lazily, so
  nothing on a development machine needs the library installed.

## Data logging

`ros2 launch picar_ros picar.launch.py record:=true` (the container's `PICAR_RECORD=true`) records
`/controller/sensor_frame` and `/planner/motion_plan` with rosbag2 into `/data/picar_<timestamp>`.
Compose mounts that folder as `data/` next to `docker/`. The recordings are the training data for
a learned planner.

## Parameters

| Parameter | Node | Meaning |
|---|---|---|
| `hardware` | controller | `fake` or `picar4wd` — which hardware adapter to use |
| `sensor_period_ms` | controller | Time between sensor frames (default 30) |
| `motor_tick_ms` | controller | Time between motor updates (default 10) |
| `hard_stop_cm` | controller | Forward drive stops closer than this (default 10) |
| `range_stale_ms` | controller | No valid range for this long stops forward drive (default 150) |
| `plan_timeout_ms` | controller | No new plan for this long stops all motors (default 500) |
| `slow_write_ms` | controller | Log a warning when one motor write takes longer than this |
| `rate_hz` | motion_planner | Plans per second (default 10) |
| `stop_distance_cm` | motion_planner | Obstacle closer than this starts the manoeuvre (default 20) |
| `cruise_speed`, `reverse_speed`, `turn_rate` | motion_planner | Speeds in `(0, 1]` |
| `stop_duration_s`, `reverse_duration_s`, `turn_duration_s` | motion_planner | Manoeuvre timing |
| `frame_stale_ms` | motion_planner | An older sensor frame makes the plan a stop (default 500) |
