import pytest

from picar.sweep import FORWARD_CONE_DEG, SweepPlan, in_forward_cone


def _angles_from(plan: SweepPlan, count: int) -> list[tuple[float, bool]]:
    return [plan.next_angle() for _ in range(count)]


def test_a_pass_visits_every_angle_once_and_ends_on_the_last_angle():
    plan = SweepPlan(-10, 10, 5)

    steps = _angles_from(plan, 5)

    assert steps == [(-10, False), (-5, False), (0, False), (5, False), (10, True)]


def test_the_return_pass_runs_back_and_ends_on_the_first_angle():
    plan = SweepPlan(-10, 10, 5)
    _angles_from(plan, 5)  # the outward pass

    steps = _angles_from(plan, 5)

    # The servo already points at 10, so the return pass starts there.
    assert steps == [(10, False), (5, False), (0, False), (-5, False), (-10, True)]


def test_the_sweep_repeats_forever():
    plan = SweepPlan(-10, 10, 5)

    angles = [angle for angle, _ in _angles_from(plan, 25)]

    assert angles[:5] == [-10, -5, 0, 5, 10]
    assert angles[5:10] == [10, 5, 0, -5, -10]
    assert angles[10:15] == [-10, -5, 0, 5, 10]


def test_the_full_sweep_is_the_default_minus_90_to_plus_90_in_37_angles():
    plan = SweepPlan(-90, 90, 5)

    assert len(plan.angles) == 37
    assert plan.angles[0] == -90 and plan.angles[-1] == 90


@pytest.mark.parametrize(
    "min_deg, max_deg, step_deg",
    [
        (-100, 90, 5),  # beyond the servo's travel
        (0, 90.5, 5),
        (10, 10, 5),  # no sweep at all
        (20, 10, 5),
        (-90, 90, 0),
        (-90, 90, -5),
        (-90, 90, 7),  # 180 is not a whole number of 7 degree steps
        (float("nan"), 90, 5),
    ],
)
def test_invalid_sweeps_are_rejected(min_deg, max_deg, step_deg):
    with pytest.raises(ValueError):
        SweepPlan(min_deg, max_deg, step_deg)


@pytest.mark.parametrize(
    "angle, expected",
    [(0, True), (FORWARD_CONE_DEG, True), (-FORWARD_CONE_DEG, True), (15.5, False), (-90, False)],
)
def test_only_readings_near_straight_ahead_count_as_forward(angle, expected):
    assert in_forward_cone(angle) is expected
