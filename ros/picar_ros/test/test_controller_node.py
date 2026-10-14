import pytest
from picar_interfaces.msg import MotionPlan, SensorFrame

from picar_core import SetServoAngle, Stop
from picar_ros.controller_node import ControllerNode

from conftest import call_arm, plan_msg

PLAN_TIMEOUT_MS = 300.0
FORWARD = (50, 50, 50, 50)
REVERSE = (-30, -30, -30, -30)


@pytest.fixture
def controller(ros):
    node = ros.start(
        ControllerNode,
        hardware="fake",
        fake_distances_cm=[100.0],
        plan_timeout_ms=PLAN_TIMEOUT_MS,
    )
    ros.frames = ros.record(SensorFrame, "/controller/sensor_frame")
    ros.plans = ros.publisher(MotionPlan, "/planner/motion_plan")
    ros.motors = lambda: node._hardware.motor_powers
    return ros


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


def test_sensor_frames_are_published_about_every_30_ms(controller):
    controller.spin_for(1.0)

    assert 15 <= len(controller.frames) <= 40  # a target, not a guarantee


def test_a_frame_describes_the_sensors_and_a_disarmed_stopped_robot(controller):
    controller.spin_for(0.3)

    frame = controller.frames[-1]
    assert frame.range_m == pytest.approx(1.0)
    assert frame.range_valid
    assert frame.servo_angle_deg == 0.0
    assert list(frame.greyscale) == [0, 0, 0]
    assert list(frame.last_motor_power) == [0, 0, 0, 0]
    assert not frame.armed


def test_the_robot_boots_disarmed_and_ignores_plans(controller):
    _send_plans(controller, FORWARD)

    assert controller.motors() == (0, 0, 0, 0)


def test_arm_and_disarm_are_reported_in_the_frames(controller):
    assert call_arm(controller, True).success
    controller.spin_for(0.2)
    assert controller.frames[-1].armed

    assert call_arm(controller, False).success
    controller.spin_for(0.2)
    assert not controller.frames[-1].armed


def test_an_armed_robot_drives_the_plan(controller):
    call_arm(controller, True)

    _send_plans(controller, FORWARD)

    assert controller.motors() == FORWARD
    assert list(controller.frames[-1].last_motor_power) == list(FORWARD)


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


def test_a_bad_plan_is_ignored(controller):
    call_arm(controller, True)
    bad = plan_msg(*[(120, 0, 0, 0)] * 10)  # fits in int8 but is outside [-100, 100]
    for _ in range(5):
        controller.plans.publish(bad)
        controller.spin_for(0.1)

    assert controller.motors() == (0, 0, 0, 0)


@pytest.fixture
def blocked(ros):
    """A controller whose forward range is inside the 10 cm hard stop."""
    node = ros.start(
        ControllerNode, hardware="fake", fake_distances_cm=[5.0], plan_timeout_ms=PLAN_TIMEOUT_MS
    )
    ros.plans = ros.publisher(MotionPlan, "/planner/motion_plan")
    ros.motors = lambda: node._hardware.motor_powers
    return ros


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
