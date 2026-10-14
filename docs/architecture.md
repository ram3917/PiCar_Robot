# Architecture

Diagrams: [architecture.puml](architecture.puml) (components) and
[state-machine.puml](state-machine.puml) (the planner's avoidance manoeuvre).

## One package, two nodes

Everything is the ROS 2 package [picar/](../picar/). Its modules are flat, and only the two node
files and `interface.py` do ROS work:

| Module | Role |
|---|---|
| `controller_node.py` | The node that owns the hardware, applies `/cmd_vel` and enforces safety |
| `planner_node.py` | The node that turns the latest range into `/cmd_vel`; the `drive` parameter switches it on |
| `governor.py` | `MotorGovernor`: mixes a `Twist` into four motor powers behind the safety limits |
| `planner.py` | `ReactivePlanner`: the stop, reverse, turn manoeuvre |
| `hardware.py` | `RobotHardware` interface, `FakeRobot`, the picar-4wd adapter and `make_hardware()` |
| `interface.py` | Topic names |
| `hw_check.py` | `ros2 run picar hw_check`: manual hardware check on the robot |

```
Controller  --/controller/range (~30 ms)-->  Planner
Controller  <--/cmd_vel (Twist, 10 Hz)------  Planner
```

| Node | Role | Subscribes | Publishes |
|---|---|---|---|
| controller | Owns all hardware, applies the command to the motors, enforces safety | `/cmd_vel` | `/controller/range`, `/controller/greyscale`, `/joint_states` |
| planner | Plans the next command from the latest range, while `drive` is true | `/controller/range` | `/cmd_vel` |

**Why exactly one hardware owner.** SunFounder's `picar-4wd` library resets the HAT, claims every
motor and ultrasonic GPIO line, and writes the PWM registers when it is imported, and it keeps
unlocked global state. Two processes that import it fight over the same lines (the second one
fails with `GPIO not allocated`) and the second import would reset the first one's HAT. So the
controller is the only node that touches the robot; the planner is plain software.

**Swapping the planner.** Any node that publishes `geometry_msgs/Twist` on `/cmd_vel` can drive the
robot: a different planner, a learned model, or `teleop_twist_keyboard`. The controller's safety
limits apply to whatever it receives.

## The drive switch

The robot has no arm or disarm state. The planner's `drive` parameter (default `false`) is the
only switch: while it is false the planner publishes nothing, so the controller's command timeout
stops the motors. Set it with the launch argument `drive:=true`, the container's `PICAR_DRIVE=true`,
or while running with `ros2 param set /planner drive true`. It gates the planner only; a
different node publishing on `/cmd_vel` is not affected by it, and is held to the safety limits below.

## Topics and messages

All messages are standard ROS ones. The names are in [interface.py](../picar/picar/interface.py).

| Topic | Type | Content |
|---|---|---|
| `/controller/range` | `sensor_msgs/Range` | Forward ultrasonic range in metres (REP 117: NaN invalid, -inf too close, +inf nothing in range), about every 30 ms |
| `/controller/greyscale` | `std_msgs/UInt16MultiArray` | The three greyscale ADC values |
| `/joint_states` | `sensor_msgs/JointState` | `ultrasonic_servo` angle in `position` (radians) and the applied power of the four motors in `effort` (percent); NaN means not measured |
| `/cmd_vel` | `geometry_msgs/Twist` | `linear.x` and `angular.z`, normalized to [-1, 1]; `angular.z` > 0 turns counter-clockwise. Normalized because there is no odometry yet, so they are not m/s and rad/s |

## Controller

Two ROS timers do the timed work:

- **Sensor timer**, every `sensor_period_ms` (30): reads the ultrasonic sensor and the greyscale
  ADC and publishes the range, greyscale and joint states. 30 ms is a target: an ultrasonic read
  takes 10 to 20 ms, and every message is timestamped so consumers can see the real rate.
- **Motor timer**, every `motor_tick_ms` (10): asks the `MotorGovernor` for the motor powers and
  writes them when they change.

The timers and the command subscription are in separate callback groups, and the node runs under
a `MultiThreadedExecutor`, so a slow ultrasonic read never delays a motor tick. The ultrasonic
read is plain GPIO and runs unlocked. The greyscale ADC and the motors share the HAT's I2C
address, so those calls take one lock and never interleave.

### Command mixing

The governor holds the latest `Twist` and mixes it as a differential drive:
`left = linear.x - angular.z`, `right = linear.x + angular.z`, times 100, rounded and clamped to
`[-100, 100]`. The four powers are left front, right front, left rear, right rear. There is no
ramping: the motors follow the command as it changes.

### Safety limits

These are enforced by the controller on every command, whoever produced it:

| Rule | Behaviour |
|---|---|
| Command heartbeat | No new `/cmd_vel` for `cmd_timeout_ms` (500) stops all motors |
| Bad command | A NaN or infinite value gives zero power |
| Hard stop | Forward drive is refused while the forward range is under `hard_stop_m` (0.10) |
| Blind stop | Forward drive is refused after `range_stale_ms` (150) without a valid range |

A command's age counts from the moment it arrives, so clock differences between machines do not
matter. "Forward" means the sum of the four motor powers is positive. When refused, every
positive power becomes zero and negative power is kept, so the robot can still reverse and spin
in place.

Not enforced in v0.2: acceleration limits, edge detection from the greyscale sensors, and a stop
on I2C faults (a slow motor write is logged). If the controller process is killed hard or power
is cut, the HAT keeps its last PWM output until the next start resets it.

## Planner

`ReactivePlanner` turns the latest range into a `Twist`. While the path is clear it cruises. An
obstacle closer than `stop_distance_m` starts a manoeuvre: stop, reverse, turn, then cruise again
(see [state-machine.puml](state-machine.puml)). The planner keeps to the manoeuvre on every step
whatever the sensor says, so it does not flap. A range that is stale (`range_stale_ms`) or
invalid gives a stop.

The v0.2 planner has no world model: the ultrasonic sensor is fixed facing forward.

## Hardware adapter

The controller reaches the robot only through the `RobotHardware` interface:

| Method | Meaning |
|---|---|
| `set_motor_power(powers)` | Four ints in `[-100, 100]`; negative is reverse |
| `stop()` | Stop all motors |
| `read_range_m()` | Distance in metres, or `None` if the reading is invalid |
| `read_greyscale()` | The three greyscale ADC values |
| `set_servo_angle(deg)` | Point the ultrasonic sensor; `0` is straight ahead |

The `hardware` parameter selects the implementation:

- `fake` — `FakeRobot`, in memory. Plays back scripted ranges (`fake_ranges_m`) and records
  commands. Used by the tests and for running the stack without a robot.
- `picar4wd` — adapter over SunFounder's `picar-4wd` library. It is imported lazily, so
  nothing on a development machine needs the library installed.

## Data logging

`ros2 launch picar picar.launch.py record:=true` (the container's `PICAR_RECORD=true`) records the
controller's range, greyscale and joint-state topics and `/cmd_vel` with rosbag2 into a new folder
under `/data`. Compose mounts that folder as `data/` next to `docker/`. The recordings are the
training data for a learned planner.

## Parameters

| Parameter | Node | Meaning |
|---|---|---|
| `hardware` | controller | `fake` or `picar4wd` — which hardware adapter to use |
| `fake_ranges_m` | controller | Scripted ranges for `fake`, in metres; the last repeats |
| `sensor_period_ms` | controller | Time between sensor readings (default 30) |
| `motor_tick_ms` | controller | Time between motor updates (default 10) |
| `hard_stop_m` | controller | Forward drive stops closer than this (default 0.10) |
| `range_stale_ms` | controller | No valid range for this long stops forward drive (default 150) |
| `cmd_timeout_ms` | controller | No new command for this long stops all motors (default 500) |
| `slow_write_ms` | controller | Log a warning when one motor write takes longer than this |
| `drive` | planner | The on switch (default `false`) |
| `rate_hz` | planner | Commands per second (default 10) |
| `stop_distance_m` | planner | Obstacle closer than this starts the manoeuvre (default 0.20) |
| `cruise_speed`, `reverse_speed`, `turn_speed` | planner | Normalized speed during each phase, in `(0, 1]` |
| `stop_duration_s`, `reverse_duration_s`, `turn_duration_s` | planner | Manoeuvre timing |
| `range_stale_ms` | planner | A range older than this makes the command a stop (default 500) |
