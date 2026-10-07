# Architecture

Diagrams: [architecture.puml](architecture.puml) (components) and
[state-machine.puml](state-machine.puml) (avoidance states).

## Two layers

| Layer | Package | Contains | Depends on |
|---|---|---|---|
| Core | `picar_core` ([core/](../core/)) | `RobotHardware` interface, `FakeRobot`, avoidance state machine, command watchdog | Python standard library only |
| ROS | `picar_ros` ([ros/picar_ros/](../ros/picar_ros/)) | perception, planning and action nodes | `picar_core`, `rclpy` |

All decisions live in the core. It never imports ROS, so it is developed and tested on any
machine, including Windows. The ROS nodes only translate between topics, services and
parameters on one side and core calls on the other.

## Data flow

```
ultrasonic sensor → perception → /ultrasonic/range → planning → /cmd_vel → action → motors
```

| Node | Role | Subscribes | Publishes / serves |
|---|---|---|---|
| perception | Reads the ultrasonic sensor | — | `/ultrasonic/range` |
| planning | Runs the avoidance state machine | `/ultrasonic/range` | `/cmd_vel`, `/robot_state`, service `/arm` |
| action | Drives the motors, enforces the command watchdog | `/cmd_vel` | — |

- **perception** publishes `sensor_msgs/Range` on `/ultrasonic/range` at 10 Hz. Invalid
  readings are published as `inf`/NaN (REP 117).
- **planning** feeds each range into the core state machine and publishes the resulting
  `geometry_msgs/Twist` on `/cmd_vel` at 20 Hz while armed. It reports the current state on
  `/robot_state` (`std_msgs/String`) and is armed or disarmed through the `/arm` service
  (`std_srvs/SetBool`).
- **action** converts each Twist into left and right wheel speeds (skid-steer) and applies
  them through `RobotHardware`.

## Hardware adapter

Perception and action reach the robot only through the `RobotHardware` interface:

| Method | Meaning |
|---|---|
| `set_wheel_speeds(left, right)` | Drive each side at a normalised speed in `[-1.0, 1.0]`; negative is reverse |
| `stop()` | Stop all motors |
| `read_distance_cm()` | Distance in centimetres, or `None` if the reading is invalid |
| `set_servo_angle(deg)` | Point the ultrasonic sensor; `0` is straight ahead |

The `hardware` parameter selects the implementation:

- `fake` — `FakeRobot`, in memory. Plays back scripted distances and records commands. Used
  by the tests and for running the stack without a robot.
- `picar4wd` — adapter over SunFounder's `picar-4wd` library. It is imported lazily, so
  nothing on a development machine needs the library installed.

## State machine

The robot boots `DISARMED`. Once armed it cycles `CRUISE` → `STOP` → `REVERSE` → `TURN` →
`CRUISE`, leaving `CRUISE` when an obstacle is closer than `stop_distance_cm` and leaving
`TURN` after `turn_duration_s`. The ultrasonic servo stays locked forward throughout.

| State | `/cmd_vel` |
|---|---|
| `DISARMED` | Nothing published |
| `CRUISE` | Forward at `cruise_speed` |
| `STOP` | Zeros |
| `REVERSE` | Backwards |
| `TURN` | Turning on the spot |
| `FAULT` | Zeros |

## Safety behaviour

- **Boots disarmed.** Nothing moves until `/arm` is called with `true`.
- **Disarm wins.** `/arm false` from any state gives a zero command and `DISARMED`.
- **Bad range is a fault.** A range that is invalid or older than `range_stale_ms` gives a
  zero command and `FAULT`. `FAULT` holds until the robot is disarmed.
- **Command watchdog.** If action receives no `/cmd_vel` for `cmd_timeout_ms` (default 500)
  it stops the motors. This covers a crashed or hung planning node.
- **Silent while disarmed.** Planning publishes nothing on `/cmd_vel` while `DISARMED`, so
  `teleop_twist_keyboard` can drive the robot; when teleop goes quiet the watchdog stops it.

## Parameters

| Parameter | Node | Meaning |
|---|---|---|
| `hardware` | perception, action | `fake` or `picar4wd` — which hardware adapter to use |
| `stop_distance_cm` | planning | Obstacle closer than this triggers the stop/reverse/turn sequence |
| `cruise_speed` | planning | Forward speed while cruising |
| `turn_duration_s` | planning | How long the turn lasts |
| `range_stale_ms` | planning | Range older than this is treated as a fault |
| `cmd_timeout_ms` | action | No `/cmd_vel` for this long → motors stop (default 500) |
