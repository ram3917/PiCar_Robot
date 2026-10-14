"""Turning a velocity command into wheel speeds for a skid-steer robot."""

from __future__ import annotations

import math

from picar_core.command import Command


def command_from_twist(linear: float, angular: float) -> Command:
    """Build a Command from raw Twist values.

    Teleop can send values beyond ``[-1.0, 1.0]``, so they are clamped. NaN is treated as zero.
    """
    return Command(linear=_clamp(linear), angular=_clamp(angular))


def wheel_speeds(command: Command) -> tuple[float, float]:
    """Left and right wheel speeds, each in ``[-1.0, 1.0]``.

    Positive angular turns counter-clockwise, so the right wheel runs faster. If either wheel
    would exceed the limit, both are scaled down together, which keeps the turn ratio.
    """
    left = command.linear - command.angular
    right = command.linear + command.angular
    peak = max(abs(left), abs(right), 1.0)
    return left / peak, right / peak


def _clamp(value: float) -> float:
    if math.isnan(value):
        return 0.0
    return max(-1.0, min(1.0, value))
