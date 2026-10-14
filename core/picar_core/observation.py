"""Sensor observation fed into the state machine."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class Observation:
    """Snapshot of sensor readings.

    Args:
        distance_cm: distance to the nearest obstacle, or None if invalid/unavailable
    """

    distance_cm: float | None
