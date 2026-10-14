import math

import pytest

from picar_core import PLAN_STEPS, ZERO_POWER, PlannerConfig, ReactivePlanner, SensorFrame

CRUISE = (50, 50, 50, 50)
REVERSE = (-30, -30, -30, -30)
TURN = (-50, 50, -50, 50)  # counter-clockwise: left motors back, right forward


def frame(distance_cm, stamp_s=0.0, armed=True):
    return SensorFrame(stamp_s=stamp_s, distance_cm=distance_cm, armed=armed)


def planner(**overrides):
    return ReactivePlanner(PlannerConfig(**overrides))


def test_nothing_is_planned_while_disarmed():
    assert planner().plan(frame(100.0, armed=False), now_s=0.0) is None


def test_a_clear_path_is_ten_cruise_steps_covering_two_seconds():
    plan = planner().plan(frame(100.0), now_s=0.0)

    assert plan.steps == (CRUISE,) * PLAN_STEPS
    assert plan.horizon_s == pytest.approx(2.0)


def test_an_open_path_with_infinite_range_cruises():
    assert planner().plan(frame(math.inf), now_s=0.0).steps[0] == CRUISE


def test_an_obstacle_starts_stop_then_reverse_then_turn_then_cruise():
    plan = planner().plan(frame(10.0), now_s=0.0)

    # 0.2 s stop, 0.4 s reverse, 1.0 s turn, then cruise; steps are 0.2 s each.
    assert plan.steps == (
        (ZERO_POWER,) + (REVERSE,) * 2 + (TURN,) * 5 + (CRUISE,) * 2
    )


def test_the_manoeuvre_is_kept_on_replans_even_if_the_path_looks_clear():
    p = planner()
    p.plan(frame(10.0, stamp_s=0.0), now_s=0.0)

    plan = p.plan(frame(100.0, stamp_s=0.4), now_s=0.4)

    # 0.4 s in: reverse for its last 0.2 s, then 1.0 s of turn, then cruise.
    assert plan.steps == (REVERSE,) + (TURN,) * 5 + (CRUISE,) * 4


def test_cruise_resumes_once_the_manoeuvre_is_over():
    p = planner()
    p.plan(frame(10.0), now_s=0.0)

    plan = p.plan(frame(100.0, stamp_s=1.7), now_s=1.7)  # manoeuvre lasts 1.6 s

    assert plan.steps == (CRUISE,) * PLAN_STEPS


def test_a_new_obstacle_after_the_manoeuvre_starts_a_new_one():
    p = planner()
    p.plan(frame(10.0), now_s=0.0)

    plan = p.plan(frame(10.0, stamp_s=1.7), now_s=1.7)

    assert plan.steps[0] == ZERO_POWER


def test_an_invalid_distance_is_a_stop():
    plan = planner().plan(frame(None), now_s=0.0)

    assert plan.steps == (ZERO_POWER,) * PLAN_STEPS


def test_a_stale_frame_is_a_stop_even_with_a_clear_path():
    plan = planner(frame_stale_ms=500.0).plan(frame(100.0, stamp_s=0.0), now_s=0.6)

    assert plan.steps == (ZERO_POWER,) * PLAN_STEPS


def test_a_frame_just_inside_the_stale_limit_is_still_used():
    plan = planner(frame_stale_ms=500.0).plan(frame(100.0, stamp_s=0.0), now_s=0.5)

    assert plan.steps[0] == CRUISE


def test_disarming_forgets_a_manoeuvre_in_progress():
    p = planner()
    p.plan(frame(10.0), now_s=0.0)
    p.plan(frame(100.0, armed=False), now_s=0.2)

    plan = p.plan(frame(100.0, stamp_s=0.4), now_s=0.4)

    assert plan.steps == (CRUISE,) * PLAN_STEPS


@pytest.mark.parametrize(
    "overrides",
    [
        {"stop_distance_cm": 0.0},
        {"cruise_speed": 0.0},
        {"cruise_speed": 1.1},
        {"turn_rate": -0.1},
        {"turn_duration_s": 0.0},
        {"frame_stale_ms": 0.0},
    ],
)
def test_bad_config_is_rejected(overrides):
    with pytest.raises(ValueError):
        PlannerConfig(**overrides)
