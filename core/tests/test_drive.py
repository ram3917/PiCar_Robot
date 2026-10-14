import pytest

from picar_core import Command, wheel_speeds


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
