import math

import pytest
from sensor_msgs.msg import JointState, Range
from std_msgs.msg import Bool, UInt16MultiArray
from trajectory_msgs.msg import JointTrajectory

from picar_core import SetServoAngle, Stop
from picar_ros.controller_node import ControllerNode
from picar_ros.interface import (
    ARMED_QOS,
    ARMED_TOPIC,
    GREYSCALE_TOPIC,
    JOINT_STATES_TOPIC,
    PLAN_TOPIC,
    RANGE_TOPIC,
)

from conftest import call_arm, plan_msg

PLAN_TIMEOUT_MS = 300.0
FORWARD = (50, 50, 50, 50)
REVERSE = (-30, -30, -30, -30)


def _start(ros, distance_cm: float):
    node = ros.start(
        ControllerNode,
        hardware="fake",
        fake_distances_cm=[distance_cm],
        plan_timeout_ms=PLAN_TIMEOUT_MS,
    )
    ros.ranges = ros.record(Range, RANGE_TOPIC)
    ros.greyscale = ros.record(UInt16MultiArray, GREYSCALE_TOPIC)
    ros.joints = ros.record(JointState, JOINT_STATES_TOPIC)
    ros.armed = ros.record(Bool, ARMED_TOPIC, ARMED_QOS)
    ros.plans = ros.publisher(JointTrajectory, PLAN_TOPIC)
    ros.motors = lambda: node._hardware.motor_powers
    return ros


@pytest.fixture
def controller(ros):
    return _start(ros, distance_cm=100.0)


@pytest.fixture
def blocked(ros):
    """A controller whose forward range is inside the 10 cm hard stop."""
    return _start(ros, distance_cm=5.0)


def _send_plans(controller, powers, seconds: float = 0.5) -> None:
    """Publish a steady plan at 10 Hz, well inside the plan timeout."""
    plan = plan_msg(*[powers] * 10)
    for _ in range(int(seconds / 0.1)):
        controller.plans.publish(plan)
        controller.spin_for(0.1)


def test_motors_are_stopped_and_the_servo_is_forward_at_startup(controller):
    controller.spin_for(0.2)

    commands = controller.node._hardware.commands
    assert commands[0] == SetServoAngle(0.0)
    assert Stop() in commands
    assert controller.motors() == (0, 0, 0, 0)


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


def test_the_robot_boots_disarmed_and_the_state_is_latched(controller):
    controller.spin_for(0.2)

    assert controller.armed[-1].data is False


def test_arm_and_disarm_are_reported_on_the_armed_topic(controller):
    assert call_arm(controller, True).success
    controller.spin_for(0.2)
    assert controller.armed[-1].data is True

    assert call_arm(controller, False).success
    controller.spin_for(0.2)
    assert controller.armed[-1].data is False


def test_the_robot_ignores_plans_while_disarmed(controller):
    _send_plans(controller, FORWARD)

    assert controller.motors() == (0, 0, 0, 0)


def test_an_armed_robot_drives_the_plan(controller):
    call_arm(controller, True)

    _send_plans(controller, FORWARD)

    assert controller.motors() == FORWARD
    assert list(controller.joints[-1].effort[1:]) == pytest.approx(list(FORWARD))


def test_motors_stop_when_the_planner_goes_quiet(controller):
    call_arm(controller, True)
    _send_plans(controller, FORWARD)
    assert controller.motors() == FORWARD

    controller.spin_for(PLAN_TIMEOUT_MS / 1000.0 + 0.2)

    assert controller.motors() == (0, 0, 0, 0)


def test_disarming_stops_the_motors_at_once(controller):
    call_arm(controller, True)
    _send_plans(controller, FORWARD)

    call_arm(controller, False)
    controller.spin_for(0.1)

    assert controller.motors() == (0, 0, 0, 0)


def test_a_plan_sent_after_disarming_is_ignored(controller):
    call_arm(controller, True)
    call_arm(controller, False)

    _send_plans(controller, FORWARD)

    assert controller.motors() == (0, 0, 0, 0)


@pytest.mark.parametrize(
    "bad",
    [
        plan_msg(*[(math.nan, 0, 0, 0)] * 10),  # not a number
        plan_msg(*[(0, 0, 0)] * 10),  # three powers instead of four
    ],
    ids=["nan", "wrong-width"],
)
def test_a_malformed_plan_is_ignored(controller, bad):
    call_arm(controller, True)

    for _ in range(5):
        controller.plans.publish(bad)
        controller.spin_for(0.1)

    assert controller.motors() == (0, 0, 0, 0)


def test_a_plan_for_the_wrong_joints_is_ignored(controller):
    call_arm(controller, True)
    wrong = plan_msg(*[FORWARD] * 10)
    wrong.joint_names = ["a", "b", "c", "d"]

    for _ in range(5):
        controller.plans.publish(wrong)
        controller.spin_for(0.1)

    assert controller.motors() == (0, 0, 0, 0)


def test_powers_beyond_the_motor_limits_are_clamped(controller):
    call_arm(controller, True)

    _send_plans(controller, (250, 250, 250, 250))

    assert controller.motors() == (100, 100, 100, 100)


def test_forward_drive_is_refused_inside_the_hard_stop_distance(blocked):
    call_arm(blocked, True)

    _send_plans(blocked, FORWARD)

    assert blocked.motors() == (0, 0, 0, 0)


def test_reversing_is_still_allowed_inside_the_hard_stop_distance(blocked):
    call_arm(blocked, True)

    _send_plans(blocked, REVERSE)

    assert blocked.motors() == REVERSE


def test_destroying_the_node_stops_the_motors(controller):
    call_arm(controller, True)
    _send_plans(controller, FORWARD)
    node = controller.node
    assert controller.motors() == FORWARD

    controller.executor.remove_node(node)
    node.destroy_node()
    controller.node = None  # already destroyed; keep the harness from doing it twice

    assert node._hardware.motor_powers == (0, 0, 0, 0)
