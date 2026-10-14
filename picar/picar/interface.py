"""The topics and joint names the controller and the planner agree on. All messages are standard.

- ``RANGE_TOPIC``: ``sensor_msgs/Range``, the forward ultrasonic range (REP 117)
- ``GREYSCALE_TOPIC``: ``std_msgs/UInt16MultiArray``, the three greyscale ADC values
- ``JOINT_STATES_TOPIC``: ``sensor_msgs/JointState``, the servo angle (``position``, radians) and the
  applied motor power in percent (``effort``); NaN means not measured
- ``CMD_VEL_TOPIC``: ``geometry_msgs/Twist``, the drive command. ``linear.x`` and ``angular.z``
  are normalized to [-1, 1] (there is no odometry yet, so they are not m/s and rad/s);
  ``angular.z`` > 0 turns counter-clockwise
"""

from __future__ import annotations

import math

from sensor_msgs.msg import Range

RANGE_TOPIC = "/controller/range"
GREYSCALE_TOPIC = "/controller/greyscale"
JOINT_STATES_TOPIC = "/joint_states"
CMD_VEL_TOPIC = "/cmd_vel"

SERVO_JOINT = "ultrasonic_servo"
# Motor order of picar-4wd.
MOTOR_JOINTS = ("left_front", "right_front", "left_rear", "right_rear")


def valid_range_m(msg: Range) -> float | None:
    """The distance in a Range message, or ``None`` if unusable.

    +inf means no echo within max range, which is a clear path, so it stays infinite. -inf (too
    close) and NaN are invalid readings.
    """
    if math.isnan(msg.range) or msg.range == -math.inf:
        return None
    return msg.range
