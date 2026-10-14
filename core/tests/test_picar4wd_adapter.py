import math
import sys
from types import SimpleNamespace

import pytest

from picar_core import RobotHardware
from picar_core.picar4wd_adapter import Picar4WDHardware


class FakeLibrary:
    """Stands in for the `picar_4wd` module, recording what the adapter asks of it."""

    def __init__(self, distance=50.0, greyscale=(1, 2, 3)):
        self.calls = []
        self.distance = distance
        self.greyscale = greyscale
        self.us = SimpleNamespace(get_distance=lambda: self.distance)
        self.servo = SimpleNamespace(set_angle=lambda angle: self.calls.append(("servo", angle)))

    def set_motor_power(self, motor, power):
        self.calls.append(("motor", motor, power))

    def stop(self):
        self.calls.append(("stop",))

    def get_grayscale_list(self):
        return list(self.greyscale)


def test_adapter_satisfies_the_hardware_protocol():
    assert isinstance(Picar4WDHardware(FakeLibrary()), RobotHardware)


def test_importing_picar_core_does_not_import_the_library():
    import picar_core  # noqa: F401

    assert "picar_4wd" not in sys.modules


def test_each_power_drives_the_motor_with_the_same_number():
    fc = FakeLibrary()

    # Left front, right front, left rear, right rear = motors 1, 2, 3, 4.
    Picar4WDHardware(fc).set_motor_power((50, -25, 30, -10))

    assert fc.calls == [
        ("motor", 1, 50),
        ("motor", 2, -25),
        ("motor", 3, 30),
        ("motor", 4, -10),
    ]


def test_full_power_maps_through_unchanged():
    fc = FakeLibrary()

    Picar4WDHardware(fc).set_motor_power((-100, 100, -100, 100))

    assert [c[2] for c in fc.calls] == [-100, 100, -100, 100]


def test_stop_delegates_to_the_library():
    fc = FakeLibrary()

    Picar4WDHardware(fc).stop()

    assert fc.calls == [("stop",)]


def test_a_valid_distance_is_returned_as_a_float():
    assert Picar4WDHardware(FakeLibrary(distance=42)).read_distance_cm() == 42.0


@pytest.mark.parametrize("code", [-1, -2, -0.5, math.nan, math.inf])
def test_library_error_codes_and_non_finite_values_read_as_none(code):
    assert Picar4WDHardware(FakeLibrary(distance=code)).read_distance_cm() is None


def test_greyscale_is_the_librarys_three_adc_values():
    assert Picar4WDHardware(FakeLibrary(greyscale=(100, 2000, 300))).read_greyscale() == (
        100,
        2000,
        300,
    )


def test_servo_angle_is_passed_to_the_library():
    fc = FakeLibrary()

    Picar4WDHardware(fc).set_servo_angle(0.0)

    assert fc.calls == [("servo", 0.0)]


@pytest.mark.parametrize("deg", [math.nan, math.inf])
def test_non_finite_servo_angle_is_rejected(deg):
    fc = FakeLibrary()

    with pytest.raises(ValueError):
        Picar4WDHardware(fc).set_servo_angle(deg)

    assert fc.calls == []
