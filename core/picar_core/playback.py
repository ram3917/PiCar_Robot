"""Plays a motion plan back at the controller's motor tick, behind the safety rules.

The safety rules live here, not in the planner, so they hold whatever produced the plan.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from picar_core.motion_plan import MotionPlan
from picar_core.motors import MAX_POWER, ZERO_POWER, MotorPowers


@dataclass(frozen=True)
class GovernorConfig:
    """Safety limits for ``MotorGovernor``.

    Args:
        hard_stop_cm: forward drive stops when the forward range is closer than this
        range_stale_ms: forward drive stops when there is no valid range for this long
        plan_timeout_ms: everything stops when no new plan arrives for this long
    """

    hard_stop_cm: float = 10.0
    range_stale_ms: float = 150.0
    plan_timeout_ms: float = 500.0

    def __post_init__(self) -> None:
        for name in ("hard_stop_cm", "range_stale_ms", "plan_timeout_ms"):
            value = getattr(self, name)
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} {value!r} must be positive")


class MotorGovernor:
    """Decides, on every tick, what power the four motors get.

    Plan timing runs from the moment a plan *arrives* here, not from the planner's clock, so it
    is safe against clock differences between machines. Not thread-safe; the caller serialises.

    Output is all zeros when disarmed, when there is no plan, when no new plan has arrived for
    ``plan_timeout_ms``, or when the plan's horizon has ended. Otherwise it ramps linearly between
    plan steps. On top of that, forward drive is blocked while the forward range is closer than
    ``hard_stop_cm`` or has been invalid for ``range_stale_ms``: every positive power is dropped to
    zero and negative power is kept, so the robot can still reverse and spin in place.
    """

    def __init__(self, config: GovernorConfig | None = None) -> None:
        self.config = config or GovernorConfig()
        self._armed = False
        self._plan: MotionPlan | None = None
        self._plan_received_s = 0.0
        self._last_valid_range_s: float | None = None
        self._last_distance_cm = math.inf
        self.last_output: MotorPowers = ZERO_POWER
        self.reason = "disarmed"

    @property
    def armed(self) -> bool:
        return self._armed

    def set_armed(self, armed: bool) -> None:
        self._armed = armed
        if not armed:
            self._plan = None

    def set_plan(self, plan: MotionPlan, now_s: float) -> None:
        """Start playing ``plan`` from ``now_s``. Ignored while disarmed."""
        if not self._armed:
            return
        self._plan = plan
        self._plan_received_s = now_s

    def update_range(self, distance_cm: float | None, now_s: float) -> None:
        """Feed the latest forward range; ``None`` means the reading was invalid."""
        if distance_cm is None or math.isnan(distance_cm):
            return
        self._last_valid_range_s = now_s
        self._last_distance_cm = distance_cm

    def output(self, now_s: float) -> MotorPowers:
        """The motor powers to apply now."""
        powers, self.reason = self._plan_power(now_s)
        if sum(powers) > 0 and self._forward_blocked(now_s):
            powers = tuple(min(p, 0) for p in powers)  # type: ignore[assignment]
            self.reason = "range_blocked"
        self.last_output = powers
        return powers

    def _plan_power(self, now_s: float) -> tuple[MotorPowers, str]:
        if not self._armed:
            return ZERO_POWER, "disarmed"
        plan = self._plan
        if plan is None:
            return ZERO_POWER, "no_plan"
        elapsed_s = now_s - self._plan_received_s
        if elapsed_s * 1000.0 > self.config.plan_timeout_ms:
            return ZERO_POWER, "plan_timeout"
        if elapsed_s >= plan.horizon_s:
            return ZERO_POWER, "plan_done"
        return _ramp(plan, max(elapsed_s, 0.0)), "ok"

    def _forward_blocked(self, now_s: float) -> bool:
        if self._last_valid_range_s is None:
            return True
        if (now_s - self._last_valid_range_s) * 1000.0 > self.config.range_stale_ms:
            return True
        return self._last_distance_cm < self.config.hard_stop_cm


def _ramp(plan: MotionPlan, elapsed_s: float) -> MotorPowers:
    """Linear interpolation between the plan's keyframes; the last step is held."""
    index = int(elapsed_s / plan.step_s)
    if index >= len(plan.steps) - 1:
        return plan.steps[-1]
    fraction = (elapsed_s - index * plan.step_s) / plan.step_s
    start, end = plan.steps[index], plan.steps[index + 1]
    mixed = tuple(round(a + (b - a) * fraction) for a, b in zip(start, end))
    return tuple(max(-MAX_POWER, min(MAX_POWER, p)) for p in mixed)  # type: ignore[return-value]
