import pytest
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState, Range
from std_msgs.msg import UInt16MultiArray

from picar.controller_node import ControllerNode
from picar.interface import CMD_VEL_TOPIC, GREYSCALE_TOPIC, JOINT_STATES_TOPIC, RANGE_TOPIC

from conftest import twist

CMD_TIMEOUT_MS = 300.0
FORWARD = (50, 50, 50, 50)
REVERSE = (-30, -30, -30, -30)
STOPPED = (0, 0, 0, 0)


def _start(ros, range_m: float):
    node = ros.start(
        ControllerNode, hardware="fake", fake_ranges_m=[range_m], cmd_timeout_ms=CMD_TIMEOUT_MS
    )
    ros.ranges = ros.record(Range, RANGE_TOPIC)
    ros.greyscale = ros.record(UInt16MultiArray, GREYSCALE_TOPIC)
    ros.joints = ros.record(JointState, JOINT_STATES_TOPIC)
    ros.cmd_vel = ros.publisher(Twist, CMD_VEL_TOPIC)
    ros.motors = lambda: node._hardware.motor_powers
    return ros


@pytest.fixture
def controller(ros):
    return _start(ros, range_m=1.0)


@pytest.fixture
def blocked(ros):
    """A controller whose forward range is inside the 10 cm hard stop."""
    return _start(ros, range_m=0.05)


def _drive(controller, cmd: Twist, seconds: float = 0.5) -> None:
    """Publish a steady command at 10 Hz, well inside the command timeout."""
    for _ in range(int(seconds / 0.1)):
        controller.cmd_vel.publish(cmd)
        controller.spin_for(0.1)


def test_motors_are_stopped_and_the_servo_is_forward_at_startup(controller):
    controller.spin_for(0.2)

    commands = controller.node._hardware.commands
    assert commands[0] == ("servo", 0.0)
    assert ("stop",) in commands
    assert controller.motors() == STOPPED


def test_sensors_are_published_about_every_30_ms(controller):
    controller.spin_for(1.0)

    assert 15 <= len(controller.ranges) <= 40  # a target, not a guarantee
    assert len(controller.greyscale) == len(controller.joints) == len(controller.ranges)


def test_the_range_is_the_ultrasonic_reading_in_metres(controller):
    controller.spin_for(0.3)

    assert controller.ranges[-1].range == pytest.approx(1.0)
    assert controller.ranges[-1].radiation_type == Range.ULTRASOUND


def test_greyscale_and_joint_states_describe_a_stopped_robot(controller):
    controller.spin_for(0.3)

    assert list(controller.greyscale[-1].data) == [0, 0, 0]
    joints = controller.joints[-1]
    assert joints.position[0] == 0.0  # the servo points forward
    assert list(joints.effort[1:]) == [0.0, 0.0, 0.0, 0.0]  # no motor power applied


def test_the_robot_drives_the_command(controller):
    _drive(controller, twist(linear=0.5))

    assert controller.motors() == FORWARD
    assert list(controller.joints[-1].effort[1:]) == pytest.approx(list(FORWARD))


def test_a_spin_command_turns_counter_clockwise(controller):
    _drive(controller, twist(angular=0.5))

    assert controller.motors() == (-50, 50, -50, 50)


def test_motors_stop_when_commands_stop_coming(controller):
    _drive(controller, twist(linear=0.5))
    assert controller.motors() == FORWARD

    controller.spin_for(CMD_TIMEOUT_MS / 1000.0 + 0.2)

    assert controller.motors() == STOPPED


def test_a_non_finite_command_is_ignored(controller):
    _drive(controller, twist(linear=float("nan")))

    assert controller.motors() == STOPPED


def test_powers_beyond_the_motor_limits_are_clamped(controller):
    _drive(controller, twist(linear=2.5))

    assert controller.motors() == (100, 100, 100, 100)


def test_forward_drive_is_refused_inside_the_hard_stop_distance(blocked):
    _drive(blocked, twist(linear=0.5))

    assert blocked.motors() == STOPPED


def test_reversing_is_still_allowed_inside_the_hard_stop_distance(blocked):
    _drive(blocked, twist(linear=-0.3))

    assert blocked.motors() == REVERSE


def test_destroying_the_node_stops_the_motors(controller):
    _drive(controller, twist(linear=0.5))
    node = controller.node
    assert controller.motors() == FORWARD

    controller.executor.remove_node(node)
    node.destroy_node()
    controller.node = None  # already destroyed; keep the harness from doing it twice

    assert node._hardware.motor_powers == STOPPED
