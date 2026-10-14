import math

import pytest

from picar_core import FakeRobot, RobotHardware, SetMotorPower, SetServoAngle, Stop


def test_fake_robot_satisfies_the_hardware_protocol():
    assert isinstance(FakeRobot(), RobotHardware)


def test_scripted_distances_are_returned_in_order():
    robot = FakeRobot([50.0, 30.0, None, 12.5])

    assert [robot.read_distance_cm() for _ in range(4)] == [50.0, 30.0, None, 12.5]


def test_last_distance_repeats_once_the_script_runs_out():
    robot = FakeRobot([40.0, 20.0])

    assert [robot.read_distance_cm() for _ in range(4)] == [40.0, 20.0, 20.0, 20.0]


def test_a_trailing_invalid_reading_also_repeats():
    robot = FakeRobot([40.0, None])

    assert [robot.read_distance_cm() for _ in range(3)] == [40.0, None, None]


def test_unscripted_robot_has_no_reading():
    assert FakeRobot().read_distance_cm() is None


def test_script_distances_queues_after_existing_readings():
    robot = FakeRobot([40.0])
    robot.script_distances([10.0, 5.0])

    assert [robot.read_distance_cm() for _ in range(3)] == [40.0, 10.0, 5.0]


def test_greyscale_is_whatever_the_test_sets():
    robot = FakeRobot()
    assert robot.read_greyscale() == (0, 0, 0)

    robot.greyscale = (100, 2000, 300)

    assert robot.read_greyscale() == (100, 2000, 300)


def test_commands_are_recorded_in_call_order():
    robot = FakeRobot()

    robot.set_servo_angle(0.0)
    robot.set_motor_power([50, 50, 50, 50])
    robot.set_motor_power((-30, 30, -30, 30))
    robot.stop()

    assert robot.commands == [
        SetServoAngle(0.0),
        SetMotorPower((50, 50, 50, 50)),
        SetMotorPower((-30, 30, -30, 30)),
        Stop(),
    ]


def test_reading_sensors_is_not_recorded_as_a_command():
    robot = FakeRobot([10.0])

    robot.read_distance_cm()
    robot.read_greyscale()

    assert robot.commands == []


def test_motor_powers_track_the_latest_command():
    robot = FakeRobot()
    assert robot.is_stopped

    robot.set_motor_power((40, -40, 40, -40))
    assert robot.motor_powers == (40, -40, 40, -40)
    assert not robot.is_stopped

    robot.stop()
    assert robot.motor_powers == (0, 0, 0, 0)
    assert robot.is_stopped


def test_zero_powers_count_as_stopped():
    robot = FakeRobot()
    robot.set_motor_power((40, 40, 40, 40))

    robot.set_motor_power((0, 0, 0, 0))

    assert robot.is_stopped


def test_servo_angle_tracks_the_latest_command():
    robot = FakeRobot()
    assert robot.servo_angle is None

    robot.set_servo_angle(-30.0)

    assert robot.servo_angle == -30.0


@pytest.mark.parametrize("powers", [(100, 100, 100, 100), (-100, -100, -100, -100), (-100, 100, -100, 100)])
def test_powers_at_the_limits_are_accepted(powers):
    robot = FakeRobot()

    robot.set_motor_power(powers)

    assert robot.motor_powers == powers


@pytest.mark.parametrize(
    "powers",
    [(101, 0, 0, 0), (0, -101, 0, 0), (0, 0, 0), (0.5, 0, 0, 0), (math.nan, 0, 0, 0)],
)
def test_bad_powers_are_rejected_and_not_recorded(powers):
    robot = FakeRobot()

    with pytest.raises(ValueError):
        robot.set_motor_power(powers)

    assert robot.commands == []
    assert robot.is_stopped


@pytest.mark.parametrize("deg", [math.nan, math.inf])
def test_non_finite_servo_angle_is_rejected(deg):
    robot = FakeRobot()

    with pytest.raises(ValueError):
        robot.set_servo_angle(deg)

    assert robot.commands == []
