# Architecture

Diagram: [architecture.puml](architecture.puml).

## Overview

An obstacle-avoiding robot on the SunFounder PiCar-4WD and a Raspberry Pi 3B+. One ROS 2 Jazzy
package, [picar/](../picar/), runs as one Docker container on the Pi and holds two nodes that talk
over standard ROS messages:

| Node | Role |
|---|---|
| controller | The only process that touches the hardware. Publishes the ultrasonic range, applies `/cmd_vel` to the motors and enforces the safety limits |
| planner | Turns the latest range into `/cmd_vel`: cruise, and on an obstacle stop, reverse, turn, cruise again |

```
controller --/controller/range-->  planner
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
- Forward drive is refused when the range is under 10 cm, or when there is no valid range for 150 ms
  (reversing and spinning in place stay allowed)

## Topics

| Topic | Type |
|---|---|
| `/controller/range` | `sensor_msgs/Range` (about every 30 ms) |
| `/controller/greyscale` | `std_msgs/UInt16MultiArray` (published, unused) |
| `/joint_states` | `sensor_msgs/JointState` (servo angle, applied motor power) |
| `/cmd_vel` | `geometry_msgs/Twist`, normalized to [-1, 1] |

## Deployment

The image is built for arm64 on the dev machine, copied to the Pi with `docker save`, `scp` and
`docker load`, and started with Docker Compose. There is no registry. Rolling back means setting
`IMAGE_TAG` to a previous tag still on the Pi. Setting `PICAR_RECORD=true` records the sensor
topics and commands with rosbag2 to `data/`, as training data for a learned planner. The steps are
in the [README](../README.md#deploy-to-the-pi).
