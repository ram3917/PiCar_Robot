import math

import pytest
from geometry_msgs.msg import Twist

from picar.governor import GovernorConfig, MotorGovernor, mix
from picar.hardware import ZERO_POWER


def cmd(linear=0.0, angular=0.0):
    twist = Twist()
    twist.linear.x = linear
    twist.angular.z = angular
    return twist


def governor(**overrides):
    # A long default, so tests that move the clock are not about the range going stale.
    overrides.setdefault("range_stale_ms", 10_000.0)
    g = MotorGovernor(GovernorConfig(**overrides))
    g.update_range(1.0, now_s=0.0)
    return g


# mix


def test_forward_drives_all_motors_forward():
    assert mix(cmd(linear=0.5)) == (50, 50, 50, 50)


def test_counter_clockwise_spin_is_left_back_and_right_forward():
    assert mix(cmd(angular=0.5)) == (-50, 50, -50, 50)


def test_a_curve_slows_the_inside_wheels():
    assert mix(cmd(linear=0.5, angular=0.2)) == (30, 70, 30, 70)


def test_powers_beyond_the_motor_limits_are_clamped():
    assert mix(cmd(linear=2.0, angular=-0.5)) == (100, 100, 100, 100)


# MotorGovernor


def test_a_new_governor_outputs_zero():
    g = MotorGovernor()

    assert g.output(0.0) == ZERO_POWER
    assert g.reason == "no_command"


def test_the_latest_command_is_applied():
    g = governor()
    g.set_command(cmd(linear=0.4), now_s=0.0)

    assert g.output(0.1) == (40, 40, 40, 40)
    assert g.reason == "ok"

    g.set_command(cmd(angular=0.3), now_s=0.1)
    assert g.output(0.2) == (-30, 30, -30, 30)


def test_output_is_zero_when_no_new_command_arrives():
    g = governor(cmd_timeout_ms=500.0)
    g.set_command(cmd(linear=0.4), now_s=0.0)

    assert g.output(0.5) == (40, 40, 40, 40)
    assert g.output(0.6) == ZERO_POWER
    assert g.reason == "command_timeout"


@pytest.mark.parametrize("bad", [math.nan, math.inf])
def test_a_non_finite_command_is_zero(bad):
    g = governor()
    g.set_command(cmd(linear=bad), now_s=0.0)

    assert g.output(0.1) == ZERO_POWER
    assert g.reason == "bad_command"


def test_forward_drive_is_blocked_inside_the_hard_stop_distance():
    g = governor(hard_stop_m=0.10)
    g.update_range(0.05, now_s=0.0)
    g.set_command(cmd(linear=0.5), now_s=0.0)

    assert g.output(0.1) == ZERO_POWER
    assert g.reason == "range_blocked"


def test_reversing_and_spinning_are_allowed_inside_the_hard_stop_distance():
    g = governor(hard_stop_m=0.10)
    g.update_range(0.05, now_s=0.0)

    g.set_command(cmd(linear=-0.3), now_s=0.0)
    assert g.output(0.1) == (-30, -30, -30, -30)

    g.set_command(cmd(angular=0.5), now_s=0.1)
    assert g.output(0.2) == (-50, 50, -50, 50)


def test_forward_drive_is_blocked_before_the_first_range():
    g = MotorGovernor()
    g.set_command(cmd(linear=0.5), now_s=0.0)

    assert g.output(0.1) == ZERO_POWER


def test_forward_drive_is_blocked_when_the_range_goes_stale():
    g = governor(range_stale_ms=150.0)
    g.set_command(cmd(linear=0.5), now_s=0.0)
    assert g.output(0.1) == (50, 50, 50, 50)

    g.set_command(cmd(linear=0.5), now_s=0.2)
    assert g.output(0.2) == ZERO_POWER


def test_an_invalid_reading_does_not_refresh_the_range():
    g = governor(range_stale_ms=150.0)
    g.set_command(cmd(linear=0.5), now_s=0.0)
    g.update_range(None, now_s=0.2)
    g.update_range(math.nan, now_s=0.2)

    g.set_command(cmd(linear=0.5), now_s=0.2)
    assert g.output(0.2) == ZERO_POWER


@pytest.mark.parametrize("name", ["hard_stop_m", "range_stale_ms", "cmd_timeout_ms"])
@pytest.mark.parametrize("value", [0.0, -1.0, math.nan, math.inf])
def test_a_limit_that_would_switch_a_rule_off_is_rejected(name, value):
    with pytest.raises(ValueError, match=name):
        GovernorConfig(**{name: value})
