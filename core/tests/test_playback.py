import pytest

from picar_core import GovernorConfig, MotionPlan, MotorGovernor, ZERO_POWER


def governor(**overrides):
    # A long default, so tests that move the clock are not about the range going stale.
    overrides.setdefault("range_stale_ms", 10_000.0)
    g = MotorGovernor(GovernorConfig(**overrides))
    g.set_armed(True)
    g.update_range(100.0, now_s=0.0)
    return g


def plan(*steps, step_s=0.2):
    return MotionPlan(steps=steps, step_s=step_s)


def test_a_new_governor_outputs_zero_and_is_disarmed():
    g = MotorGovernor()

    assert g.output(0.0) == ZERO_POWER
    assert g.reason == "disarmed"


def test_armed_without_a_plan_outputs_zero():
    g = governor()

    assert g.output(0.0) == ZERO_POWER
    assert g.reason == "no_plan"


def test_the_first_step_plays_at_the_moment_the_plan_arrives():
    g = governor()
    g.set_plan(plan((40, 40, 40, 40), (80, 80, 80, 80)), now_s=0.0)

    assert g.output(0.0) == (40, 40, 40, 40)
    assert g.reason == "ok"


def test_power_ramps_linearly_between_steps():
    g = governor()
    g.set_plan(plan((0, 0, 0, 0), (100, 100, 100, 100)), now_s=0.0)

    assert g.output(0.05) == (25, 25, 25, 25)
    assert g.output(0.10) == (50, 50, 50, 50)


def test_ramp_goes_through_zero_when_the_direction_changes():
    g = governor()
    g.set_plan(plan((-40, 40, -40, 40), (40, -40, 40, -40)), now_s=0.0)

    assert g.output(0.10) == (0, 0, 0, 0)


def test_the_last_step_is_held_until_the_plan_horizon_ends():
    g = governor(plan_timeout_ms=5000.0)
    g.set_plan(plan((10, 10, 10, 10), (60, 60, 60, 60)), now_s=0.0)

    assert g.output(0.3) == (60, 60, 60, 60)


def test_output_is_zero_after_the_plan_horizon():
    g = governor(plan_timeout_ms=5000.0)
    g.set_plan(plan((60, 60, 60, 60), (60, 60, 60, 60)), now_s=0.0)

    assert g.output(0.4) == ZERO_POWER
    assert g.reason == "plan_done"


def test_no_new_plan_within_the_timeout_stops_the_motors():
    g = governor(plan_timeout_ms=500.0)
    g.set_plan(plan(*[(60, 60, 60, 60)] * 10), now_s=0.0)
    g.update_range(100.0, now_s=0.5)

    assert g.output(0.5) == (60, 60, 60, 60)
    g.update_range(100.0, now_s=0.6)
    assert g.output(0.6) == ZERO_POWER
    assert g.reason == "plan_timeout"


def test_a_new_plan_restarts_the_clock():
    g = governor(plan_timeout_ms=500.0)
    g.set_plan(plan(*[(60, 60, 60, 60)] * 10), now_s=0.0)
    g.update_range(100.0, now_s=0.6)
    g.set_plan(plan(*[(30, 30, 30, 30)] * 10), now_s=0.6)

    assert g.output(0.7) == (30, 30, 30, 30)


def test_disarming_stops_at_once_and_drops_the_plan():
    g = governor()
    g.set_plan(plan(*[(60, 60, 60, 60)] * 10), now_s=0.0)
    g.set_armed(False)

    assert g.output(0.01) == ZERO_POWER
    assert g.reason == "disarmed"

    g.set_armed(True)
    assert g.output(0.02) == ZERO_POWER
    assert g.reason == "no_plan"


def test_a_plan_that_arrives_while_disarmed_is_ignored():
    g = MotorGovernor()
    g.set_plan(plan(*[(60, 60, 60, 60)] * 10), now_s=0.0)
    g.set_armed(True)

    assert g.output(0.01) == ZERO_POWER
    assert g.reason == "no_plan"


def test_forward_drive_is_blocked_when_the_range_is_inside_the_hard_stop():
    g = governor(hard_stop_cm=10.0)
    g.set_plan(plan(*[(60, 60, 60, 60)] * 10), now_s=0.0)
    g.update_range(9.0, now_s=0.01)

    assert g.output(0.02) == ZERO_POWER
    assert g.reason == "range_blocked"


def test_forward_drive_is_allowed_exactly_at_the_hard_stop_distance():
    g = governor(hard_stop_cm=10.0)
    g.set_plan(plan(*[(60, 60, 60, 60)] * 10), now_s=0.0)
    g.update_range(10.0, now_s=0.01)

    assert g.output(0.02) == (60, 60, 60, 60)


def test_reversing_is_still_allowed_when_the_range_is_blocked():
    g = governor()
    g.set_plan(plan(*[(-30, -30, -30, -30)] * 10), now_s=0.0)
    g.update_range(5.0, now_s=0.01)

    assert g.output(0.02) == (-30, -30, -30, -30)


def test_spinning_in_place_is_still_allowed_when_the_range_is_blocked():
    g = governor()
    g.set_plan(plan(*[(-50, 50, -50, 50)] * 10), now_s=0.0)
    g.update_range(5.0, now_s=0.01)

    assert g.output(0.02) == (-50, 50, -50, 50)


def test_an_arc_with_net_forward_motion_loses_its_forward_part_when_blocked():
    g = governor()
    g.set_plan(plan(*[(-10, 60, -10, 60)] * 10), now_s=0.0)
    g.update_range(5.0, now_s=0.01)

    assert g.output(0.02) == (-10, 0, -10, 0)


def test_forward_drive_is_blocked_when_the_range_goes_stale():
    g = governor(range_stale_ms=150.0)
    g.set_plan(plan(*[(60, 60, 60, 60)] * 10), now_s=0.0)

    assert g.output(0.14) == (60, 60, 60, 60)
    assert g.output(0.16) == ZERO_POWER
    assert g.reason == "range_blocked"


def test_invalid_readings_do_not_refresh_the_range():
    g = governor(range_stale_ms=150.0)
    g.set_plan(plan(*[(60, 60, 60, 60)] * 10), now_s=0.0)
    g.update_range(None, now_s=0.14)
    g.update_range(float("nan"), now_s=0.15)

    assert g.output(0.16) == ZERO_POWER


def test_a_fresh_valid_reading_unblocks_forward_drive():
    g = governor(range_stale_ms=150.0)
    g.set_plan(plan(*[(60, 60, 60, 60)] * 10), now_s=0.0)
    assert g.output(0.2) == ZERO_POWER

    g.update_range(100.0, now_s=0.21)

    assert g.output(0.22) == (60, 60, 60, 60)


def test_forward_drive_is_blocked_until_the_first_valid_range():
    g = MotorGovernor()
    g.set_armed(True)
    g.set_plan(plan(*[(60, 60, 60, 60)] * 10), now_s=0.0)

    assert g.output(0.01) == ZERO_POWER


def test_an_open_path_with_infinite_range_does_not_block():
    g = governor()
    g.update_range(float("inf"), now_s=0.0)
    g.set_plan(plan(*[(60, 60, 60, 60)] * 10), now_s=0.0)

    assert g.output(0.01) == (60, 60, 60, 60)


@pytest.mark.parametrize(
    "overrides",
    [{"hard_stop_cm": 0.0}, {"range_stale_ms": -1.0}, {"plan_timeout_ms": 0.0}],
)
def test_bad_config_is_rejected(overrides):
    with pytest.raises(ValueError):
        GovernorConfig(**overrides)
