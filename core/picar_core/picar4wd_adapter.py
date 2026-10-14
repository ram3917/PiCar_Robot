"""RobotHardware backed by SunFounder's picar-4wd library.

Only works on the robot. The library is imported when the adapter is created, never at
module import, so `picar_core` stays importable on machines without it.
"""

from __future__ import annotations

import math
from types import ModuleType

# Motor numbers as picar-4wd names them: 1 left front, 2 right front, 3 left rear, 4 right rear.
LEFT_MOTORS = (1, 3)
RIGHT_MOTORS = (2, 4)
MAX_POWER = 100


def _import_library() -> ModuleType:
    import picar_4wd  # only needed on the robot, so imported here

    return picar_4wd


class Picar4WDHardware:
    """Drives the PiCar-4WD through the `picar_4wd` module.

    Args:
        fc: the `picar_4wd` module. Imported when not given; tests pass a stand-in.

    The library maps any non-zero power to a PWM duty of 50-100 %, so the lowest speeds
    are not gentler than the smallest real one. Tune `cruise_speed` on the robot.
    """

    def __init__(self, fc: ModuleType | None = None) -> None:
        self._fc = fc if fc is not None else _import_library()

    def set_wheel_speeds(self, left: float, right: float) -> None:
        for name, value in (("left", left), ("right", right)):
            # Written so that NaN fails the check too.
            if not -1.0 <= value <= 1.0:
                raise ValueError(f"{name} wheel speed {value!r} is outside [-1.0, 1.0]")
        left_power = round(left * MAX_POWER)
        right_power = round(right * MAX_POWER)
        for motor in LEFT_MOTORS:
            self._fc.set_motor_power(motor, left_power)
        for motor in RIGHT_MOTORS:
            self._fc.set_motor_power(motor, right_power)

    def stop(self) -> None:
        self._fc.stop()

    def read_distance_cm(self) -> float | None:
        # On failure the library returns a negative code (-1: no echo, -2: echo never ends).
        distance = self._fc.us.get_distance()
        if distance is None or not math.isfinite(distance) or distance < 0:
            return None
        return float(distance)

    def set_servo_angle(self, deg: float) -> None:
        if not math.isfinite(deg):
            raise ValueError(f"servo angle {deg!r} is not finite")
        self._fc.servo.set_angle(deg)
