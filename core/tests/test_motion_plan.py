import math

import pytest

from picar_core import MotionPlan

ONE = ((10, 20, 30, 40),)


def test_a_plan_keeps_its_times_and_steps_and_has_a_horizon():
    plan = MotionPlan(times_s=(0.2, 0.4), steps=((0, 0, 0, 0), (50, 50, 50, 50)))

    assert plan.times_s == (0.2, 0.4)
    assert plan.steps == ((0, 0, 0, 0), (50, 50, 50, 50))
    assert plan.horizon_s == pytest.approx(0.4)


def test_lists_are_normalised_to_tuples_of_floats():
    plan = MotionPlan(times_s=[0.5], steps=[[1, 2, 3, 4]])

    assert plan.times_s == (0.5,)
    assert plan.steps == ((1.0, 2.0, 3.0, 4.0),)


def test_a_plan_needs_at_least_one_point():
    with pytest.raises(ValueError):
        MotionPlan(times_s=(), steps=())


@pytest.mark.parametrize(
    "times, steps",
    [
        ((0.2,), ((1, 2, 3),)),  # a step needs four powers
        ((0.2, 0.4), ONE),  # one step per time
        ((0.4, 0.2), ((0, 0, 0, 0), (0, 0, 0, 0))),  # times must increase
        ((0.2, 0.2), ((0, 0, 0, 0), (0, 0, 0, 0))),
        ((-0.1,), ONE),
        ((math.nan,), ONE),
        ((0.2,), ((0, 0, math.nan, 0),)),
        ((0.2,), ((0, math.inf, 0, 0),)),
    ],
)
def test_malformed_plans_are_rejected(times, steps):
    with pytest.raises(ValueError):
        MotionPlan(times_s=times, steps=steps)
