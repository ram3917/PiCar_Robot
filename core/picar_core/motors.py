"""Per-motor power: what the planner outputs and the controller applies."""

from __future__ import annotations

from typing import Iterable, Tuple

from picar_core.command import Command
from picar_core.drive import wheel_speeds

MAX_POWER = 100
MOTOR_COUNT = 4

# Power for the four motors in picar-4wd's order: left front, right front, left rear, right rear.
MotorPowers = Tuple[int, int, int, int]
ZERO_POWER: MotorPowers = (0, 0, 0, 0)


def validate_motor_powers(powers: Iterable[int]) -> MotorPowers:
    """The powers as a 4-tuple of ints in ``[-MAX_POWER, MAX_POWER]``.

    Raises:
        ValueError: if there are not four values, or any is not an int in range
    """
    values = tuple(powers)
    if len(values) != MOTOR_COUNT:
        raise ValueError(f"expected {MOTOR_COUNT} motor powers, got {len(values)}")
    for value in values:
        if isinstance(value, bool) or not isinstance(value, int):
            raise ValueError(f"motor power {value!r} is not an int")
        if not -MAX_POWER <= value <= MAX_POWER:
            raise ValueError(f"motor power {value!r} is outside [{-MAX_POWER}, {MAX_POWER}]")
    return values  # type: ignore[return-value]


def motor_powers_from_command(command: Command) -> MotorPowers:
    """Skid-steer mapping: the left motors get the left wheel speed, the right ones the right."""
    left, right = wheel_speeds(command)
    left_power = round(left * MAX_POWER)
    right_power = round(right * MAX_POWER)
    return (left_power, right_power, left_power, right_power)
