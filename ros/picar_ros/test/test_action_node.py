import pytest
from geometry_msgs.msg import Twist

from picar_core import Stop
from picar_ros.action_node import ActionNode

from conftest import twist

TIMEOUT_MS = 200.0


@pytest.fixture
def action(ros):
    node = ros.start(ActionNode, hardware="fake", cmd_timeout_ms=TIMEOUT_MS)
    ros.cmd_vel = ros.publisher(Twist, "/cmd_vel")
    ros.wheels = lambda: node._hardware.wheel_speeds
    return ros


def _drive(action, msg: Twist, seconds: float = 0.3) -> None:
    """Publish ``msg`` at 20 Hz, well inside the watchdog timeout."""
    for _ in range(int(seconds / 0.05)):
        action.cmd_vel.publish(msg)
        action.spin_for(0.05)


def test_motors_idle_without_commands(action):
    action.spin_for(0.3)

    assert action.wheels() == (0.0, 0.0)


def test_forward_command_drives_both_wheels(action):
    _drive(action, twist(linear=0.5))

    assert action.wheels() == pytest.approx((0.5, 0.5))


def test_positive_angular_turns_counter_clockwise(action):
    _drive(action, twist(angular=0.5))

    left, right = action.wheels()
    assert left == pytest.approx(-0.5)
    assert right == pytest.approx(0.5)


def test_teleop_values_out_of_range_are_clamped(action):
    _drive(action, twist(linear=3.0))

    assert action.wheels() == pytest.approx((1.0, 1.0))


def test_watchdog_stops_motors_when_commands_stop(action):
    _drive(action, twist(linear=0.5))
    assert action.wheels() == pytest.approx((0.5, 0.5))

    action.spin_for(TIMEOUT_MS / 1000.0 + 0.2)

    assert action.wheels() == (0.0, 0.0)


def test_motors_resume_after_watchdog_stop(action):
    _drive(action, twist(linear=0.5))
    action.spin_for(TIMEOUT_MS / 1000.0 + 0.2)

    _drive(action, twist(linear=0.3))

    assert action.wheels() == pytest.approx((0.3, 0.3))


def test_stop_motors_stops_the_hardware(action):
    _drive(action, twist(linear=0.5))

    action.node.stop_motors()

    assert action.node._hardware.commands[-1] == Stop()
    assert action.wheels() == (0.0, 0.0)
