import pytest
from geometry_msgs.msg import Twist
from std_msgs.msg import String

from picar_ros.planning_node import PlanningNode

from conftest import RangeFeed, call_arm, states

CLEAR_M = 1.0
OBSTACLE_M = 0.1  # under the 20 cm stop distance
PARAMS = dict(stop_distance_cm=20.0, cruise_speed=0.4, turn_duration_s=0.3, range_stale_ms=300.0)


@pytest.fixture
def planning(ros):
    ros.start(PlanningNode, **PARAMS)
    return ros


def test_boots_disarmed_and_stays_off_cmd_vel(planning):
    state = planning.record(String, "/robot_state")
    cmds = planning.record(Twist, "/cmd_vel")
    RangeFeed(planning, CLEAR_M)

    planning.spin_for(0.5)

    assert states(state) and set(states(state)) == {"DISARMED"}
    assert cmds == []  # teleop owns /cmd_vel while disarmed


def test_arm_with_clear_path_cruises_forward(planning):
    state = planning.record(String, "/robot_state")
    cmds = planning.record(Twist, "/cmd_vel")
    RangeFeed(planning, CLEAR_M)
    planning.spin_for(0.2)

    response = call_arm(planning, True)

    assert response.success
    assert response.message == "CRUISE"
    assert planning.spin_until(lambda: len(cmds) >= 3)
    assert states(state)[-1] == "CRUISE"
    assert cmds[-1].linear.x == pytest.approx(PARAMS["cruise_speed"])
    assert cmds[-1].angular.z == 0.0


def test_obstacle_runs_stop_reverse_turn_then_cruises_again(planning):
    state = planning.record(String, "/robot_state")
    cmds = planning.record(Twist, "/cmd_vel")
    feed = RangeFeed(planning, CLEAR_M)
    planning.spin_for(0.2)
    call_arm(planning, True)
    planning.spin_until(lambda: states(state)[-1:] == ["CRUISE"])

    feed.distance_m = OBSTACLE_M
    assert planning.spin_until(lambda: "TURN" in states(state))
    feed.distance_m = CLEAR_M
    assert planning.spin_until(lambda: states(state)[-1] == "CRUISE")

    seen = _collapse(states(state))
    assert seen[seen.index("STOP") :] == ["STOP", "REVERSE", "TURN", "CRUISE"]
    assert any(cmd.linear.x < 0.0 for cmd in cmds), "never reversed"
    assert any(cmd.angular.z != 0.0 for cmd in cmds), "never turned"


def test_stale_range_while_armed_faults_and_stops(planning):
    state = planning.record(String, "/robot_state")
    cmds = planning.record(Twist, "/cmd_vel")
    feed = RangeFeed(planning, CLEAR_M)
    planning.spin_for(0.2)
    call_arm(planning, True)
    planning.spin_until(lambda: states(state)[-1:] == ["CRUISE"])

    feed.stop()  # perception goes silent

    assert planning.spin_until(lambda: states(state)[-1] == "FAULT")
    planning.spin_for(0.1)
    assert cmds[-1].linear.x == 0.0 and cmds[-1].angular.z == 0.0


def test_invalid_range_while_armed_faults(planning):
    state = planning.record(String, "/robot_state")
    feed = RangeFeed(planning, CLEAR_M)
    planning.spin_for(0.2)
    call_arm(planning, True)
    planning.spin_until(lambda: states(state)[-1:] == ["CRUISE"])

    feed.distance_m = float("-inf")  # too close to measure

    assert planning.spin_until(lambda: states(state)[-1] == "FAULT")


def test_disarm_sends_one_stop_then_goes_quiet(planning):
    state = planning.record(String, "/robot_state")
    cmds = planning.record(Twist, "/cmd_vel")
    RangeFeed(planning, CLEAR_M)
    planning.spin_for(0.2)
    call_arm(planning, True)
    planning.spin_until(lambda: len(cmds) >= 3)

    response = call_arm(planning, False)
    planning.spin_for(0.1)  # let in-flight messages land
    count_after_disarm = len(cmds)
    planning.spin_for(0.4)

    assert response.success
    assert response.message == "DISARMED"
    assert cmds[-1].linear.x == 0.0 and cmds[-1].angular.z == 0.0
    assert len(cmds) == count_after_disarm
    assert states(state)[-1] == "DISARMED"


def _collapse(values: list[str]) -> list[str]:
    """Drop consecutive repeats: [A, A, B, A] -> [A, B, A]."""
    return [v for i, v in enumerate(values) if i == 0 or values[i - 1] != v]
