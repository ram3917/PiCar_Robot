import math

import pytest

from picar_core import Command, command_from_twist, wheel_speeds


@pytest.mark.parametrize(
    "linear, angular, expected",
    [
        (0.5, 0.0, (0.5, 0.5)),  # straight ahead
        (-0.5, 0.0, (-0.5, -0.5)),  # straight back
        (0.0, 0.5, (-0.5, 0.5)),  # spin counter-clockwise: left back, right forward
        (0.0, -0.5, (0.5, -0.5)),  # spin clockwise
        (0.5, 0.5, (0.0, 1.0)),  # arc left: right wheel at the limit
    ],
)
def test_skid_steer_mapping(linear, angular, expected):
    assert wheel_speeds(Command(linear=linear, angular=angular)) == pytest.approx(expected)


def test_over_limit_mix_is_scaled_down_keeping_the_ratio():
    left, right = wheel_speeds(Command(linear=0.5, angular=1.0))

    assert max(abs(left), abs(right)) == pytest.approx(1.0)
    assert right / left == pytest.approx(-3.0)  # left = -0.5, right = 1.5 before scaling


def test_stop_gives_zero_wheel_speeds():
    assert wheel_speeds(Command(linear=0.0, angular=0.0)) == (0.0, 0.0)


@pytest.mark.parametrize("linear, angular", [(3.0, 0.0), (0.0, -2.5), (1.0, 1.0)])
def test_teleop_values_beyond_the_limit_are_clamped(linear, angular):
    command = command_from_twist(linear, angular)

    assert -1.0 <= command.linear <= 1.0
    assert -1.0 <= command.angular <= 1.0


def test_nan_from_a_twist_is_treated_as_zero():
    assert command_from_twist(math.nan, math.nan) == Command(linear=0.0, angular=0.0)


def test_in_range_values_pass_through_unchanged():
    assert command_from_twist(0.25, -0.75) == Command(linear=0.25, angular=-0.75)
