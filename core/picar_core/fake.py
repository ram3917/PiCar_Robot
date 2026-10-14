"""An in-memory `RobotHardware` for tests and for running without a robot."""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass
from typing import Iterable, Sequence, Tuple, Union

from picar_core.motors import ZERO_POWER, MotorPowers


@dataclass(frozen=True)
class SetMotorPower:
    powers: MotorPowers


@dataclass(frozen=True)
class Stop:
    pass


@dataclass(frozen=True)
class SetServoAngle:
    deg: float


class FakeRobot:
    """Plays back scripted distances and records every command it is given.

    Each `read_distance_cm` call consumes the next scripted value. Once the script runs
    out the last value repeats, so a simulated loop can keep running; with no script at
    all every reading is ``None``. `greyscale` is whatever the test sets it to.
    """

    def __init__(self, distances: Iterable[float | None] = ()) -> None:
        self._distances: deque[float | None] = deque(distances)
        self._last_distance: float | None = None
        self.commands: list[Union[SetMotorPower, Stop, SetServoAngle]] = []
        self.motor_powers: MotorPowers = ZERO_POWER
        self.servo_angle: float | None = None
        self.greyscale: Tuple[int, int, int] = (0, 0, 0)

    def script_distances(self, distances: Iterable[float | None]) -> None:
        """Queue more readings after the ones already scripted."""
        self._distances.extend(distances)

    def set_motor_power(self, powers: Sequence[int]) -> None:
        self.motor_powers = tuple(powers)  # type: ignore[assignment]
        self.commands.append(SetMotorPower(self.motor_powers))

    def stop(self) -> None:
        self.motor_powers = ZERO_POWER
        self.commands.append(Stop())

    def read_distance_cm(self) -> float | None:
        if self._distances:
            self._last_distance = self._distances.popleft()
        return self._last_distance

    def read_greyscale(self) -> Tuple[int, int, int]:
        return self.greyscale

    def set_servo_angle(self, deg: float) -> None:
        if not math.isfinite(deg):
            raise ValueError(f"servo angle {deg!r} is not finite")
        self.servo_angle = deg
        self.commands.append(SetServoAngle(deg))

    @property
    def is_stopped(self) -> bool:
        return self.motor_powers == ZERO_POWER
