import math

import pytest
from geometry_msgs.msg import Twist
from rclpy.parameter import Parameter

from picar.interface import CMD_VEL_TOPIC
from picar.planner_node import PlannerNode

from conftest import RangeFeed

STALE_MS = 300.0


@pytest.fixture
def planner(ros):
    ros.start(PlannerNode, drive=True, rate_hz=20.0, range_stale_ms=STALE_MS)
    ros.commands = ros.record(Twist, CMD_VEL_TOPIC)
    return ros


def _last(planner) -> tuple[float, float]:
    cmd = planner.commands[-1]
    return (cmd.linear.x, cmd.angular.z)


def test_nothing_is_commanded_before_the_first_range(planner):
    planner.spin_for(0.5)

    assert planner.commands == []


def test_nothing_is_commanded_while_drive_is_false(ros):
    ros.start(PlannerNode, drive=False, rate_hz=20.0)
    commands = ros.record(Twist, CMD_VEL_TOPIC)
    RangeFeed(ros, range_m=1.0)

    ros.spin_for(0.5)

    assert commands == []


def test_drive_can_be_switched_on_while_running(ros):
    ros.start(PlannerNode, drive=False, rate_hz=20.0)
    commands = ros.record(Twist, CMD_VEL_TOPIC)
    RangeFeed(ros, range_m=1.0)
    ros.spin_for(0.3)
    assert commands == []

    ros.node.set_parameters([Parameter("drive", value=True)])

    assert ros.spin_until(lambda: commands)


def test_a_clear_path_cruises(planner):
    RangeFeed(planner, range_m=1.0)

    assert planner.spin_until(lambda: planner.commands)

    assert _last(planner) == pytest.approx((0.5, 0.0))


def test_commands_are_published_at_the_planner_rate(planner):
    RangeFeed(planner, range_m=1.0)

    planner.spin_for(1.0)

    assert 8 <= len(planner.commands) <= 25  # 20 Hz target


def test_an_obstacle_starts_with_a_stop(planner):
    RangeFeed(planner, range_m=0.1)

    assert planner.spin_until(lambda: planner.commands)

    assert _last(planner) == (0.0, 0.0)


def test_a_stale_range_gives_a_stop(planner):
    feed = RangeFeed(planner, range_m=1.0)
    assert planner.spin_until(lambda: planner.commands)
    feed.stop()

    planner.spin_for(STALE_MS / 1000.0 + 0.3)

    assert _last(planner) == (0.0, 0.0)


def test_an_invalid_range_gives_a_stop(planner):
    RangeFeed(planner, range_m=math.nan)

    assert planner.spin_until(lambda: planner.commands)

    assert _last(planner) == (0.0, 0.0)

