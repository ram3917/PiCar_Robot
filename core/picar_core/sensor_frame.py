"""What the reactive planner looks at."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class SensorFrame:
    """The latest forward range and arming state.

    Args:
        stamp_s: when the range arrived, in seconds on the planner's clock
        distance_cm: forward ultrasonic distance, or None if the reading is invalid
        armed: whether the robot is armed
    """

    stamp_s: float
    distance_cm: float | None
    armed: bool = False
