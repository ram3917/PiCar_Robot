"""Turning a velocity command into wheel speeds for a skid-steer robot."""

from __future__ import annotations

from picar_core.command import Command


def wheel_speeds(command: Command) -> tuple[float, float]:
    """Left and right wheel speeds, each in ``[-1.0, 1.0]``.

    Positive angular turns counter-clockwise, so the right wheel runs faster. If either wheel
    would exceed the limit, both are scaled down together, which keeps the turn ratio.
    """
    left = command.linear - command.angular
    right = command.linear + command.angular
    peak = max(abs(left), abs(right), 1.0)
    return left / peak, right / peak
