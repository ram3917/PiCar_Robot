"""Per-motor power: what the planner outputs and the controller applies."""

from __future__ import annotations

from typing import Tuple

MAX_POWER = 100

# Power for the four motors in picar-4wd's order: left front, right front, left rear, right rear.
MotorPowers = Tuple[int, int, int, int]
ZERO_POWER: MotorPowers = (0, 0, 0, 0)
