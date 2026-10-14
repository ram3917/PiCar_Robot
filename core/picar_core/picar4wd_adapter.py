"""RobotHardware backed by SunFounder's picar-4wd library.

Only works on the robot. The library is imported when the adapter is created, never at
module import, so `picar_core` stays importable on machines without it.

The library does its hardware setup at import (HAT reset, GPIO claims, PWM writes) and is not
safe to use from two processes, so exactly one process may create this adapter.
"""

from __future__ import annotations

import math
from types import ModuleType
from typing import Sequence, Tuple

# Motor numbers as picar-4wd names them: 1 left front, 2 right front, 3 left rear, 4 right rear.
# This is also the order of `MotorPowers`, so power i drives motor i + 1.


def _import_library() -> ModuleType:
    import picar_4wd  # only needed on the robot, so imported here

    return picar_4wd


class Picar4WDHardware:
    """Drives the PiCar-4WD through the `picar_4wd` module.

    Args:
        fc: the `picar_4wd` module. Imported when not given; tests pass a stand-in.

    The library maps any non-zero power to a PWM duty of 50-100 %, so the lowest powers
    are not gentler than the smallest real one.
    """

    def __init__(self, fc: ModuleType | None = None) -> None:
        self._fc = fc if fc is not None else _import_library()

    def set_motor_power(self, powers: Sequence[int]) -> None:
        for motor, power in enumerate(powers, start=1):
            self._fc.set_motor_power(motor, power)

    def stop(self) -> None:
        self._fc.stop()

    def read_distance_cm(self) -> float | None:
        # On failure the library returns a negative code (-1: no echo, -2: echo never ends).
        distance = self._fc.us.get_distance()
        if distance is None or not math.isfinite(distance) or distance < 0:
            return None
        return float(distance)

    def read_greyscale(self) -> Tuple[int, int, int]:
        left, middle, right = self._fc.get_grayscale_list()
        return int(left), int(middle), int(right)

    def set_servo_angle(self, deg: float) -> None:
        if not math.isfinite(deg):
            raise ValueError(f"servo angle {deg!r} is not finite")
        self._fc.servo.set_angle(deg)
