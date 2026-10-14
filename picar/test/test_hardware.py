import math
from types import SimpleNamespace

import pytest

from picar import hardware
from picar.hardware import (
    FakeRobot,
    Picar4WDHardware,
    RobotHardware,
    SimulatedRobot,
    make_hardware,
)


class FakeLibrary:
    """Stands in for the `picar_4wd` module, recording what the adapter asks of it."""

    def __init__(self, distance_cm=50.0, greyscale=(1, 2, 3)):
        self.calls = []
        self.distance_cm = distance_cm
        self.greyscale = greyscale
        self.us = SimpleNamespace(get_distance=lambda: self.distance_cm)
        self.servo = SimpleNamespace(set_angle=lambda angle: self.calls.append(("servo", angle)))

    def set_motor_power(self, motor, power):
        self.calls.append(("motor", motor, power))

    def stop(self):
        self.calls.append(("stop",))

    def get_grayscale_list(self):
        return list(self.greyscale)


def test_both_adapters_satisfy_the_hardware_protocol():
    assert isinstance(FakeRobot(), RobotHardware)
    assert isinstance(Picar4WDHardware(FakeLibrary()), RobotHardware)


# FakeRobot


def test_scripted_ranges_play_in_order_and_the_last_repeats():
    robot = FakeRobot([0.5, 0.3, None])

    assert [robot.read_range_m() for _ in range(5)] == [0.5, 0.3, None, None, None]


def test_an_unscripted_robot_has_no_reading():
    assert FakeRobot().read_range_m() is None


def test_commands_are_recorded_in_call_order():
    robot = FakeRobot()

    robot.set_servo_angle(0.0)
    robot.set_motor_power([50, 50, 50, 50])
    robot.stop()

    assert robot.commands == [("servo", 0.0), ("power", (50, 50, 50, 50)), ("stop",)]
    assert robot.motor_powers == (0, 0, 0, 0)


@pytest.mark.parametrize("deg", [math.nan, math.inf])
def test_non_finite_servo_angles_are_rejected(deg):
    for robot in (FakeRobot(), Picar4WDHardware(FakeLibrary())):
        with pytest.raises(ValueError):
            robot.set_servo_angle(deg)


# Picar4WDHardware


def test_each_power_drives_the_motor_with_the_same_number():
    fc = FakeLibrary()

    # Left front, right front, left rear, right rear = motors 1, 2, 3, 4.
    Picar4WDHardware(fc).set_motor_power((50, -25, 30, -10))

    assert fc.calls == [("motor", 1, 50), ("motor", 2, -25), ("motor", 3, 30), ("motor", 4, -10)]


def test_stop_delegates_to_the_library():
    fc = FakeLibrary()

    Picar4WDHardware(fc).stop()

    assert fc.calls == [("stop",)]


def test_a_valid_distance_is_returned_in_metres():
    assert Picar4WDHardware(FakeLibrary(distance_cm=42)).read_range_m() == pytest.approx(0.42)


@pytest.mark.parametrize("code", [-1, -2, -0.5, math.nan, math.inf])
def test_library_error_codes_and_non_finite_values_read_as_none(code):
    assert Picar4WDHardware(FakeLibrary(distance_cm=code)).read_range_m() is None


def test_greyscale_is_the_librarys_three_adc_values():
    assert Picar4WDHardware(FakeLibrary(greyscale=(100, 2000, 300))).read_greyscale() == (
        100,
        2000,
        300,
    )


# SimulatedRobot


class Clock:
    def __init__(self) -> None:
        self.now_s = 0.0

    def __call__(self) -> float:
        return self.now_s


def _sim(clock: Clock) -> SimulatedRobot:
    robot = SimulatedRobot(walls_m=[1.0, 0.6], speed_mps=0.5, turn_to_clear_s=0.25, clock=clock)
    assert robot.read_range_m() == pytest.approx(1.0)
    return robot


def test_sim_wall_closes_when_driving_forward_and_opens_in_reverse():
    clock = Clock()
    robot = _sim(clock)

    robot.set_motor_power([50, 50, 50, 50])
    clock.now_s += 0.2
    assert robot.read_range_m() == pytest.approx(0.95)  # 0.5 power * 0.5 m/s * 0.2 s

    robot.set_motor_power([-100, -100, -100, -100])
    clock.now_s += 0.1
    assert robot.read_range_m() == pytest.approx(1.0)


def test_sim_wall_holds_still_when_stopped():
    clock = Clock()
    robot = _sim(clock)

    clock.now_s += 0.2
    assert robot.read_range_m() == pytest.approx(1.0)


def test_sim_turning_in_place_faces_the_next_wall_cyclically():
    clock = Clock()
    robot = _sim(clock)
    robot.set_motor_power([-50, 50, -50, 50])

    distances = []
    for _ in range(8):  # turning at half power: a new wall every 0.5 s
        clock.now_s += 0.125
        distances.append(robot.read_range_m())

    assert distances == pytest.approx([1.0, 1.0, 1.0, 0.6, 0.6, 0.6, 0.6, 1.0])


def test_sim_never_reports_a_negative_range():
    clock = Clock()
    robot = _sim(clock)
    robot.set_motor_power([100, 100, 100, 100])

    for _ in range(50):
        clock.now_s += 0.2
        assert robot.read_range_m() >= 0.0


# make_hardware


def test_fake_plays_back_scripted_ranges():
    robot = make_hardware("fake", fake_ranges_m=[0.3, 0.1])

    assert isinstance(robot, FakeRobot)
    assert [robot.read_range_m() for _ in range(3)] == [0.3, 0.1, 0.1]


def test_sim_builds_the_simulated_wall():
    robot = make_hardware("sim", sim_walls_m=[0.7])

    assert isinstance(robot, SimulatedRobot)
    assert robot.read_range_m() == pytest.approx(0.7)


def test_picar4wd_builds_the_adapter(monkeypatch):
    # The real library needs a Raspberry Pi, so a stand-in replaces the import.
    monkeypatch.setattr(hardware, "_import_library", lambda: SimpleNamespace())

    assert isinstance(make_hardware("picar4wd"), Picar4WDHardware)


def test_unknown_name_is_rejected():
    with pytest.raises(ValueError, match="unknown hardware 'gopigo'"):
        make_hardware("gopigo")
