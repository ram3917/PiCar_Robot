"""The plan the planner hands to the controller."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Tuple

import numpy as np


@dataclass(frozen=True)
class MotionPlan:
    """Motor powers to reach at given times, like a ``trajectory_msgs/JointTrajectory``.

    The controller ramps linearly between points, holds the first one before its time, and stops
    once the last time has passed. Times count from the moment the plan arrives.

    Args:
        times_s: seconds after arrival at which each point's powers are reached, increasing
        steps: the four motor powers (see ``MotorPowers``) at each point; need not be ints
    """

    times_s: Tuple[float, ...]
    steps: Tuple[Tuple[float, float, float, float], ...]

    def __post_init__(self) -> None:
        times = np.asarray(self.times_s, dtype=float)
        steps = np.asarray(self.steps, dtype=float)
        if times.ndim != 1 or times.size == 0 or steps.shape != (times.size, 4):
            raise ValueError(f"need one 4-power step per time, got {times.shape} and {steps.shape}")
        if not (np.isfinite(times).all() and np.isfinite(steps).all()):
            raise ValueError("times and powers must be finite")
        if times[0] < 0 or (np.diff(times) <= 0).any():
            raise ValueError("times must be non-negative and strictly increasing")
        object.__setattr__(self, "times_s", tuple(times.tolist()))
        object.__setattr__(self, "steps", tuple(map(tuple, steps.tolist())))

    @property
    def horizon_s(self) -> float:
        return self.times_s[-1]
