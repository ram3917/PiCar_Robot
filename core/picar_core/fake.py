"""An in-memory `RobotHardware` for tests and for running without a robot."""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass
from typing import Iterable, Union


@dataclass(frozen=True)
class SetWheelSpeeds:
    left: float
    right: float


@dataclass(frozen=True)
class Stop:
    pass


@dataclass(frozen=True)
class SetServoAngle:
    deg: float


HardwareCall = Union[SetWheelSpeeds, Stop, SetServoAngle]


class FakeRobot:
    """Plays back scripted distances and records every command it is given.

    Each `read_distance_cm` call consumes the next scripted value. Once the script runs
    out the last value repeats, so a simulated loop can keep running; with no script at
    all every reading is ``None``.
    """

    def __init__(self, distances: Iterable[float | None] = ()) -> None:
        self._distances: deque[float | None] = deque(distances)
        self._last_distance: float | None = None
        self.commands: list[HardwareCall] = []
        self.wheel_speeds: tuple[float, float] = (0.0, 0.0)
        self.servo_angle: float | None = None

    def script_distances(self, distances: Iterable[float | None]) -> None:
        """Queue more readings after the ones already scripted."""
        self._distances.extend(distances)

    def set_wheel_speeds(self, left: float, right: float) -> None:
        for name, value in (("left", left), ("right", right)):
            # Written so that NaN fails the check too.
            if not -1.0 <= value <= 1.0:
                raise ValueError(f"{name} wheel speed {value!r} is outside [-1.0, 1.0]")
        self.wheel_speeds = (left, right)
        self.commands.append(SetWheelSpeeds(left, right))

    def stop(self) -> None:
        self.wheel_speeds = (0.0, 0.0)
        self.commands.append(Stop())

    def read_distance_cm(self) -> float | None:
        if self._distances:
            self._last_distance = self._distances.popleft()
        return self._last_distance

    def set_servo_angle(self, deg: float) -> None:
        if not math.isfinite(deg):
            raise ValueError(f"servo angle {deg!r} is not finite")
        self.servo_angle = deg
        self.commands.append(SetServoAngle(deg))

    @property
    def is_stopped(self) -> bool:
        return self.wheel_speeds == (0.0, 0.0)
