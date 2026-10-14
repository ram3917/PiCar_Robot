"""The robot hardware: the interface, an in-memory fake, and the SunFounder picar-4wd adapter.

Motor order is picar-4wd's, everywhere: left front, right front, left rear, right rear.
"""

from __future__ import annotations

import math
from collections import deque
from types import ModuleType
from typing import Iterable, Protocol, Sequence, Tuple, runtime_checkable

MAX_POWER = 100
MotorPowers = Tuple[int, int, int, int]
ZERO_POWER: MotorPowers = (0, 0, 0, 0)


@runtime_checkable
class RobotHardware(Protocol):
    """Motors, ultrasonic sensor, greyscale sensors and sensor servo of a 4-motor robot.

    Implementations must not raise for a bad sensor reading; they report it as ``None``.
    """

    def set_motor_power(self, powers: Sequence[int]) -> None:
        """Drive the four motors. Each power is an int in ``[-100, 100]``; negative is reverse."""
        ...

    def stop(self) -> None:
        """Stop all motors."""
        ...

    def read_range_m(self) -> float | None:
        """Distance to the nearest obstacle in metres, or ``None`` if the reading is invalid."""
        ...

    def read_greyscale(self) -> Tuple[int, int, int]:
        """The three downward greyscale ADC readings."""
        ...

    def set_servo_angle(self, deg: float) -> None:
        """Point the ultrasonic sensor; ``0`` is straight ahead, positive is to the left."""
        ...


class FakeRobot:
    """An in-memory robot for tests and for running without hardware.

    Each `read_range_m` call consumes the next scripted distance. Once the script runs out the
    last value repeats; with no script every reading is ``None``. Every command is appended to
    `commands` as a tuple: ``("power", powers)``, ``("stop",)`` or ``("servo", deg)``.
    """

    def __init__(self, ranges_m: Iterable[float | None] = ()) -> None:
        self._ranges: deque[float | None] = deque(ranges_m)
        self._last_range: float | None = None
        self.commands: list[tuple] = []
        self.motor_powers: MotorPowers = ZERO_POWER
        self.servo_angle: float | None = None
        self.greyscale: Tuple[int, int, int] = (0, 0, 0)

    def set_motor_power(self, powers: Sequence[int]) -> None:
        self.motor_powers = tuple(powers)  # type: ignore[assignment]
        self.commands.append(("power", self.motor_powers))

    def stop(self) -> None:
        self.motor_powers = ZERO_POWER
        self.commands.append(("stop",))

    def read_range_m(self) -> float | None:
        if self._ranges:
            self._last_range = self._ranges.popleft()
        return self._last_range

    def read_greyscale(self) -> Tuple[int, int, int]:
        return self.greyscale

    def set_servo_angle(self, deg: float) -> None:
        if not math.isfinite(deg):
            raise ValueError(f"servo angle {deg!r} is not finite")
        self.servo_angle = deg
        self.commands.append(("servo", deg))


def _import_library() -> ModuleType:
    import picar_4wd  # only on the robot, and it resets the HAT and claims GPIO when imported

    return picar_4wd


class Picar4WDHardware:
    """Drives the PiCar-4WD through SunFounder's `picar_4wd` module.

    Exactly one process may create this: the library is not safe to use from two. Motor ``i``
    of the power order is picar-4wd motor ``i + 1``. The library maps any non-zero power to a
    PWM duty of 50-100 %, so the lowest powers are not gentler than the smallest real one.

    Args:
        fc: the `picar_4wd` module. Imported when not given; tests pass a stand-in.
    """

    def __init__(self, fc: ModuleType | None = None) -> None:
        self._fc = fc if fc is not None else _import_library()

    def set_motor_power(self, powers: Sequence[int]) -> None:
        for motor, power in enumerate(powers, start=1):
            self._fc.set_motor_power(motor, power)

    def stop(self) -> None:
        self._fc.stop()

    def read_range_m(self) -> float | None:
        # On failure the library returns a negative code (-1: no echo, -2: echo never ends).
        distance_cm = self._fc.us.get_distance()
        if distance_cm is None or not math.isfinite(distance_cm) or distance_cm < 0:
            return None
        return float(distance_cm) / 100.0

    def read_greyscale(self) -> Tuple[int, int, int]:
        left, middle, right = self._fc.get_grayscale_list()
        return int(left), int(middle), int(right)

    def set_servo_angle(self, deg: float) -> None:
        if not math.isfinite(deg):
            raise ValueError(f"servo angle {deg!r} is not finite")
        self._fc.servo.set_angle(deg)


HARDWARE_NAMES = ("fake", "picar4wd")


def make_hardware(name: str, fake_ranges_m: Iterable[float] = ()) -> RobotHardware:
    """Build the adapter named by the `hardware` parameter: ``fake`` or ``picar4wd``."""
    if name == "fake":
        return FakeRobot(fake_ranges_m)
    if name == "picar4wd":
        return Picar4WDHardware()
    raise ValueError(f"unknown hardware {name!r}; expected one of {HARDWARE_NAMES}")
