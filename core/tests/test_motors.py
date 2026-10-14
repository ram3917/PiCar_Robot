import math

import pytest

from picar_core import (
    Command,
    MotionPlan,
    SensorFrame,
    ZERO_POWER,
    motor_powers_from_command,
    validate_motor_powers,
)


def test_straight_command_drives_all_four_motors_equally():
    assert motor_powers_from_command(Command(linear=0.5, angular=0.0)) == (50, 50, 50, 50)


def test_spin_drives_left_motors_back_and_right_motors_forward():
    # Order is left front, right front, left rear, right rear.
    assert motor_powers_from_command(Command(linear=0.0, angular=0.5)) == (-50, 50, -50, 50)


def test_stop_command_gives_zero_power():
    assert motor_powers_from_command(Command(linear=0.0, angular=0.0)) == ZERO_POWER


def test_valid_powers_are_returned_as_a_tuple():
    assert validate_motor_powers([1, -2, 100, -100]) == (1, -2, 100, -100)


@pytest.mark.parametrize(
    "powers",
    [
        (0, 0, 0),
        (0, 0, 0, 0, 0),
        (101, 0, 0, 0),
        (0, -101, 0, 0),
        (0.5, 0, 0, 0),
        (math.nan, 0, 0, 0),
        (True, 0, 0, 0),
    ],
)
def test_bad_powers_are_rejected(powers):
    with pytest.raises(ValueError):
        validate_motor_powers(powers)


def test_plan_horizon_is_steps_times_step_duration():
    plan = MotionPlan(steps=[ZERO_POWER] * 10, step_s=0.2)

    assert plan.horizon_s == pytest.approx(2.0)


def test_plan_normalises_steps_to_tuples():
    plan = MotionPlan(steps=[[1, 2, 3, 4]])

    assert plan.steps == ((1, 2, 3, 4),)


@pytest.mark.parametrize("step_s", [0.0, -0.2, math.nan, math.inf])
def test_plan_rejects_a_bad_step_duration(step_s):
    with pytest.raises(ValueError):
        MotionPlan(steps=[ZERO_POWER], step_s=step_s)


def test_plan_needs_at_least_one_step():
    with pytest.raises(ValueError):
        MotionPlan(steps=[])


def test_plan_rejects_an_out_of_range_step():
    with pytest.raises(ValueError):
        MotionPlan(steps=[(0, 0, 0, 101)])


def test_sensor_frame_defaults_describe_a_stopped_disarmed_robot():
    frame = SensorFrame(stamp_s=1.0, distance_cm=50.0)

    assert frame.greyscale == (0, 0, 0)
    assert frame.last_motor_power == ZERO_POWER
    assert frame.servo_angle_deg == 0.0
    assert not frame.armed


def test_sensor_frame_needs_three_greyscale_values():
    with pytest.raises(ValueError):
        SensorFrame(stamp_s=0.0, distance_cm=None, greyscale=(1, 2))
