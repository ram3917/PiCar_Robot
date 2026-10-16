import pytest

from picar.planner import PlannerConfig, ReactivePlanner

# (linear.x, angular.z)
STOP = (0.0, 0.0)
CRUISE = (0.5, 0.0)
REVERSE = (-0.3, 0.0)
TURN = (0.0, 0.5)  # counter-clockwise


def planner(**overrides):
    return ReactivePlanner(PlannerConfig(**overrides))


def step(p, range_m, now_s=0.0, age_s=0.0):
    twist = p.step(range_m, age_s, now_s)
    return (twist.linear.x, twist.angular.z)


def test_a_clear_path_cruises():
    assert step(planner(), 1.0) == CRUISE


def test_an_open_path_with_infinite_range_cruises():
    assert step(planner(), float("inf")) == CRUISE


def test_an_obstacle_starts_stop_then_reverse_then_turn_then_cruise():
    p = planner()

    assert step(p, 0.1, now_s=0.0) == STOP  # 0.2 s stop
    assert step(p, 0.1, now_s=0.1) == STOP
    assert step(p, 0.1, now_s=0.2) == REVERSE  # 0.4 s reverse
    assert step(p, 0.1, now_s=0.5) == REVERSE
    assert step(p, 0.1, now_s=0.7) == TURN  # 1.0 s turn
    assert step(p, 0.1, now_s=1.5) == TURN


def test_the_manoeuvre_is_kept_even_if_the_path_looks_clear():
    p = planner()
    step(p, 0.1, now_s=0.0)

    assert step(p, 1.0, now_s=0.3) == REVERSE
    assert step(p, 1.0, now_s=0.8) == TURN


def test_cruise_resumes_once_the_manoeuvre_is_over():
    p = planner()
    step(p, 0.1, now_s=0.0)

    assert step(p, 1.0, now_s=1.7) == CRUISE


def test_a_second_obstacle_after_the_manoeuvre_starts_another():
    p = planner()
    step(p, 0.1, now_s=0.0)

    assert step(p, 0.1, now_s=1.7) == STOP


def test_a_stale_range_is_a_stop_and_cancels_the_manoeuvre():
    p = planner(range_stale_ms=300.0)
    step(p, 0.1, now_s=0.0)

    assert step(p, 0.1, now_s=0.3, age_s=0.4) == STOP
    assert step(p, 1.0, now_s=0.4) == CRUISE


def test_an_invalid_range_is_a_stop():
    assert step(planner(), None) == STOP


@pytest.mark.parametrize("distance_m", [0.2, 0.25])
def test_an_obstacle_at_or_beyond_the_stop_distance_is_not_an_obstacle(distance_m):
    assert step(planner(), distance_m) == CRUISE
