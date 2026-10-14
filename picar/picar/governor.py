"""Turns drive commands into motor powers, behind the safety rules.

The safety rules live here, not in the planner, so they hold whatever sent the command.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from geometry_msgs.msg import Twist

from picar.hardware import MAX_POWER, ZERO_POWER, MotorPowers


@dataclass(frozen=True)
class GovernorConfig:
    """Safety limits for ``MotorGovernor``.

    Args:
        hard_stop_m: forward drive stops when the forward range is closer than this
        range_stale_ms: forward drive stops when there is no valid range for this long
        cmd_timeout_ms: everything stops when no new command arrives for this long
    """

    hard_stop_m: float = 0.10
    range_stale_ms: float = 150.0
    cmd_timeout_ms: float = 500.0

    def __post_init__(self) -> None:
        # A zero or negative limit would silently switch a safety rule off.
        for name in ("hard_stop_m", "range_stale_ms", "cmd_timeout_ms"):
            value = getattr(self, name)
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} {value!r} must be positive")


def mix(cmd: Twist) -> MotorPowers:
    """Differential-drive mix of a normalized Twist into four motor powers, clamped to ±100."""
    left = cmd.linear.x - cmd.angular.z
    right = cmd.linear.x + cmd.angular.z
    power = [max(-MAX_POWER, min(MAX_POWER, round(MAX_POWER * v))) for v in (left, right, left, right)]
    return tuple(power)  # type: ignore[return-value]


class MotorGovernor:
    """Decides, on every tick, what power the four motors get.

    Command timing runs from the moment a command *arrives* here, not from the sender's clock,
    so it is safe against clock differences between machines. Not thread-safe; the caller
    serialises.

    Output is all zeros when there is no command or no new one has arrived for
    ``cmd_timeout_ms``. Otherwise it is the mixed command. On top of that, forward drive is
    blocked while the forward range is closer than ``hard_stop_m`` or has been invalid for
    ``range_stale_ms``: every positive power is dropped to zero and negative power is kept, so
    the robot can still reverse and spin in place.
    """

    def __init__(self, config: GovernorConfig | None = None) -> None:
        self.config = config or GovernorConfig()
        self._cmd: Twist | None = None
        self._cmd_received_s = 0.0
        self._last_valid_range_s: float | None = None
        self._last_range_m = math.inf
        self.reason = "no_command"

    def set_command(self, cmd: Twist, now_s: float) -> None:
        """Use ``cmd`` from ``now_s`` until the next one or the timeout."""
        self._cmd = cmd
        self._cmd_received_s = now_s

    def update_range(self, range_m: float | None, now_s: float) -> None:
        """Feed the latest forward range; ``None`` means the reading was invalid."""
        if range_m is None or math.isnan(range_m):
            return
        self._last_valid_range_s = now_s
        self._last_range_m = range_m

    def output(self, now_s: float) -> MotorPowers:
        """The motor powers to apply now."""
        powers, self.reason = self._command_power(now_s)
        if sum(powers) > 0 and self._forward_blocked(now_s):
            powers = tuple(min(p, 0) for p in powers)  # type: ignore[assignment]
            self.reason = "range_blocked"
        return powers

    def _command_power(self, now_s: float) -> tuple[MotorPowers, str]:
        if self._cmd is None:
            return ZERO_POWER, "no_command"
        if (now_s - self._cmd_received_s) * 1000.0 > self.config.cmd_timeout_ms:
            return ZERO_POWER, "command_timeout"
        if not all(math.isfinite(v) for v in (self._cmd.linear.x, self._cmd.angular.z)):
            return ZERO_POWER, "bad_command"
        return mix(self._cmd), "ok"

    def _forward_blocked(self, now_s: float) -> bool:
        if self._last_valid_range_s is None:
            return True
        if (now_s - self._last_valid_range_s) * 1000.0 > self.config.range_stale_ms:
            return True
        return self._last_range_m < self.config.hard_stop_m
