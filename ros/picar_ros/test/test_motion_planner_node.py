import math

import pytest
from picar_interfaces.msg import MotionPlan

from picar_ros.motion_planner_node import MotionPlannerNode

from conftest import FrameFeed

STALE_MS = 300.0
CRUISE = [50, 50, 50, 50]


@pytest.fixture
def planner(ros):
    ros.start(MotionPlannerNode, rate_hz=20.0, frame_stale_ms=STALE_MS)
    ros.plans = ros.record(MotionPlan, "/planner/motion_plan")
    return ros


def _steps(plan: MotionPlan) -> list[list[int]]:
    return [list(step.power) for step in plan.steps]


def test_nothing_is_planned_before_the_first_frame(planner):
    planner.spin_for(0.5)

    assert planner.plans == []


def test_nothing_is_planned_while_disarmed(planner):
    FrameFeed(planner, range_m=1.0, armed=False)

    planner.spin_for(0.5)

    assert planner.plans == []


def test_a_clear_path_gives_ten_cruise_steps_of_200_ms(planner):
    FrameFeed(planner, range_m=1.0)

    assert planner.spin_until(lambda: planner.plans)

    plan = planner.plans[-1]
    assert plan.step_s == pytest.approx(0.2)
    assert _steps(plan) == [CRUISE] * 10


def test_plans_are_published_at_the_planner_rate(planner):
    FrameFeed(planner, range_m=1.0)

    planner.spin_for(1.0)

    assert 8 <= len(planner.plans) <= 25  # 20 Hz target


def test_an_obstacle_starts_a_stop_reverse_turn_manoeuvre(planner):
    FrameFeed(planner, range_m=0.1)

    assert planner.spin_until(lambda: planner.plans)

    steps = _steps(planner.plans[0])
    assert steps[0] == [0, 0, 0, 0]  # stop
    assert steps[1] == [-30, -30, -30, -30]  # reverse
    assert steps[3] == [-50, 50, -50, 50]  # then spin counter-clockwise


def test_a_stale_frame_turns_the_plan_into_a_stop(planner):
    feed = FrameFeed(planner, range_m=1.0)
    assert planner.spin_until(lambda: planner.plans)
    feed.stop()

    planner.spin_for(STALE_MS / 1000.0 + 0.3)

    assert _steps(planner.plans[-1]) == [[0, 0, 0, 0]] * 10


def test_an_invalid_range_gives_a_stop(planner):
    FrameFeed(planner, range_m=math.nan)

    assert planner.spin_until(lambda: planner.plans)

    assert _steps(planner.plans[-1]) == [[0, 0, 0, 0]] * 10
