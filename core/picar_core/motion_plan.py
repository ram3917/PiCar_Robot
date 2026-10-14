"""The plan the planner hands to the controller."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Tuple

from picar_core.motors import MotorPowers, validate_motor_powers

PLAN_STEPS = 10
STEP_S = 0.2  # 10 steps x 200 ms = a 2 s plan


@dataclass(frozen=True)
class MotionPlan:
    """A timed sequence of motor powers.

    Each step is a keyframe at the start of its ``step_s`` slot. The controller ramps linearly
    from one step to the next and holds the last step until the plan runs out.

    Args:
        steps: motor powers per step, each a 4-tuple (see ``MotorPowers``)
        step_s: duration of one step in seconds
    """

    steps: Tuple[MotorPowers, ...]
    step_s: float = STEP_S

    def __post_init__(self) -> None:
        if not self.steps:
            raise ValueError("a plan needs at least one step")
        if not math.isfinite(self.step_s) or self.step_s <= 0:
            raise ValueError(f"step_s {self.step_s!r} must be positive")
        # Normalises lists to tuples and rejects bad values.
        object.__setattr__(self, "steps", tuple(validate_motor_powers(s) for s in self.steps))

    @property
    def horizon_s(self) -> float:
        return len(self.steps) * self.step_s
