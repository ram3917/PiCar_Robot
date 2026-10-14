import math

import pytest
from rclpy.duration import Duration
from trajectory_msgs.msg import JointTrajectory

from picar_ros.interface import MOTOR_JOINTS, PLAN_TOPIC
from picar_ros.motion_planner_node import MotionPlannerNode

from conftest import RangeFeed

STALE_MS = 300.0
CRUISE = [50.0, 50.0, 50.0, 50.0]
ZERO = [0.0, 0.0, 0.0, 0.0]


@pytest.fixture
def planner(ros):
    ros.start(MotionPlannerNode, rate_hz=20.0, frame_stale_ms=STALE_MS)
    ros.plans = ros.record(JointTrajectory, PLAN_TOPIC)
    return ros


def _steps(plan: JointTrajectory) -> list[list[float]]:
    return [list(point.positions) for point in plan.points]


def test_nothing_is_planned_before_the_first_range(planner):
    planner.spin_for(0.5)

    assert planner.plans == []


def test_nothing_is_planned_while_disarmed(planner):
    RangeFeed(planner, range_m=1.0, armed=False)

    planner.spin_for(0.5)

    assert planner.plans == []


def test_a_clear_path_gives_ten_cruise_points_ending_after_two_seconds(planner):
    RangeFeed(planner, range_m=1.0)

    assert planner.spin_until(lambda: planner.plans)

    plan = planner.plans[-1]
    assert list(plan.joint_names) == list(MOTOR_JOINTS)
    assert _steps(plan) == [CRUISE] * 10
    times = [Duration.from_msg(p.time_from_start).nanoseconds / 1e9 for p in plan.points]
    assert times == pytest.approx([0.2 * (i + 1) for i in range(10)])


def test_plans_are_published_at_the_planner_rate(planner):
    RangeFeed(planner, range_m=1.0)

    planner.spin_for(1.0)

    assert 8 <= len(planner.plans) <= 25  # 20 Hz target


def test_an_obstacle_starts_a_stop_reverse_turn_manoeuvre(planner):
    RangeFeed(planner, range_m=0.1)

    assert planner.spin_until(lambda: planner.plans)

    steps = _steps(planner.plans[0])
    assert steps[0] == ZERO  # stop
    assert steps[1] == [-30.0] * 4  # reverse
    assert steps[3] == [-50.0, 50.0, -50.0, 50.0]  # then spin counter-clockwise


def test_a_stale_range_turns_the_plan_into_a_stop(planner):
    feed = RangeFeed(planner, range_m=1.0)
    assert planner.spin_until(lambda: planner.plans)
    feed.stop()

    planner.spin_for(STALE_MS / 1000.0 + 0.3)

    assert _steps(planner.plans[-1]) == [ZERO] * 10


def test_an_invalid_range_gives_a_stop(planner):
    RangeFeed(planner, range_m=math.nan)

    assert planner.spin_until(lambda: planner.plans)

    assert _steps(planner.plans[-1]) == [ZERO] * 10
