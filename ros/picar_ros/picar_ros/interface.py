"""The topics, service and joint names the controller and the motion planner agree on.

All messages are standard ROS ones:

- ``RANGE_TOPIC``: ``sensor_msgs/Range``, the forward ultrasonic range (REP 117)
- ``GREYSCALE_TOPIC``: ``std_msgs/UInt16MultiArray``, the three greyscale ADC values
- ``JOINT_STATES_TOPIC``: ``sensor_msgs/JointState``, the servo angle (``position``, radians) and the
  applied motor power in percent (``effort``); NaN means not measured
- ``ARMED_TOPIC``: ``std_msgs/Bool``, latched, whether the robot is armed
- ``PLAN_TOPIC``: ``trajectory_msgs/JointTrajectory``, motor power (``positions``, -100 to 100)
  reached at each point's ``time_from_start``; the controller ramps between points
- ``ARM_SERVICE``: ``std_srvs/SetBool``, served by the controller
"""

from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

RANGE_TOPIC = "/controller/range"
GREYSCALE_TOPIC = "/controller/greyscale"
JOINT_STATES_TOPIC = "/joint_states"
ARMED_TOPIC = "/controller/armed"
PLAN_TOPIC = "/planner/motion_plan"
ARM_SERVICE = "/arm"

SERVO_JOINT = "ultrasonic_servo"
# Motor order of picar-4wd, and of every plan point.
MOTOR_JOINTS = ("left_front", "right_front", "left_rear", "right_rear")

# Latched, so a node that starts late still learns the current armed state.
ARMED_QOS = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
)
