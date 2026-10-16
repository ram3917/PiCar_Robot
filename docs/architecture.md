# Architecture

Diagram: [architecture.puml](architecture.puml).

## Overview

An obstacle-avoiding robot on the SunFounder PiCar-4WD and a Raspberry Pi 3B+. One ROS 2 Jazzy
package, [picar/](../picar/), runs as one Docker container on the Pi and holds two nodes that talk
over standard ROS messages:

| Node | Role |
|---|---|
| controller | The only process that touches the hardware. Sweeps the ultrasonic servo and publishes the scan and the forward range, applies `/cmd_vel` to the motors and enforces the safety limits |
| planner | Turns the latest range into `/cmd_vel`: cruise, and on an obstacle stop, reverse, turn, cruise again |

```
controller --/controller/range-->  planner
controller --/controller/scan-->    (sweep, for later planners)
controller <--/cmd_vel (Twist)---  planner
```

## Key decisions

- **One hardware owner.** SunFounder's `picar-4wd` library resets the HAT and claims the GPIO lines
  when imported, so two processes cannot share it. The controller owns the robot; the planner is
  plain software.
- **Swappable planner.** Any node that publishes `geometry_msgs/Twist` on `/cmd_vel` (teleop, a
  learned model) can drive the robot. The controller's safety limits apply to whatever it receives.
- **Off by default.** The planner's `drive` parameter is the only on switch. While it is false the
  planner publishes nothing and the controller's command timeout stops the motors.
- **Hardware behind an interface.** `RobotHardware` has a `fake` implementation for tests and
  development without a robot, and a `picar4wd` adapter for the real one.

## Safety limits

Enforced by the controller on every command, whoever sent it:

- No new `/cmd_vel` for 500 ms stops all motors
- A NaN or infinite command gives zero power
- Forward drive is refused when the forward range is under 25 cm, or when there is no forward range for
  1.5 s. Only readings within 15° of straight ahead count as the forward range, so a sweep (which
  reads the sides) cannot stop or allow forward drive
  (reversing and spinning in place stay allowed)

## Topics

| Topic | Type |
|---|---|
| `/controller/range` | `sensor_msgs/Range` (the forward range, about every 30 ms while the servo points forward) |
| `/controller/scan` | `sensor_msgs/LaserScan` (one sweep per pass, about every 1.1 s) |
| `/controller/greyscale` | `std_msgs/UInt16MultiArray` (published, unused) |
| `/joint_states` | `sensor_msgs/JointState` (servo angle, applied motor power) |
| `/cmd_vel` | `geometry_msgs/Twist`, normalized to [-1, 1] |

## Deployment

There are two parts. The base image (Ubuntu, ROS 2, uv, picar-4wd, without picar) is built for arm64
on the dev machine and copied to the Pi with `docker save`, `scp` and `docker load`. It changes rarely.

The picar package is released from a `v<version>` tag on `main`: CI runs the tests and attaches the
wheel and its checksum to a GitHub Release. The Pi installs a pinned version into a volume with
`docker/install_picar.sh`, which verifies the checksum before it replaces the old version. The base
image is not rebuilt for a release. `docker compose up -d` runs the installed version. Setting `PICAR_RECORD=true` records the sensor
topics and commands with rosbag2 to `data/`, as training data for a learned planner. The steps are
in the [README](../README.md#deploy-to-the-pi).
