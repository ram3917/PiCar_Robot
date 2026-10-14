"""The v0.2 planner: reactive obstacle avoidance expressed as a 2 s motor plan."""

from __future__ import annotations

from dataclasses import dataclass

from picar_core.command import Command
from picar_core.motion_plan import PLAN_STEPS, STEP_S, MotionPlan
from picar_core.motors import ZERO_POWER, MotorPowers, motor_powers_from_command
from picar_core.sensor_frame import SensorFrame


_BOUNDARY_TOLERANCE_S = 1e-9


@dataclass(frozen=True)
class PlannerConfig:
    """Parameters for the reactive planner.

    Args:
        stop_distance_cm: an obstacle closer than this starts stop -> reverse -> turn
        cruise_speed: forward speed while cruising, in (0.0, 1.0]
        reverse_speed: backward speed while reversing, in (0.0, 1.0]
        turn_rate: spin rate while turning, in (0.0, 1.0]
        stop_duration_s: how long the robot holds still before reversing
        reverse_duration_s: how long it reverses
        turn_duration_s: how long it turns
        frame_stale_ms: a frame older than this is a fault and the plan is a stop
    """

    stop_distance_cm: float = 20.0
    cruise_speed: float = 0.5
    reverse_speed: float = 0.3
    turn_rate: float = 0.5
    stop_duration_s: float = 0.2
    reverse_duration_s: float = 0.4
    turn_duration_s: float = 1.0
    frame_stale_ms: float = 500.0

    def __post_init__(self) -> None:
        for name in ("stop_distance_cm", "stop_duration_s", "reverse_duration_s",
                     "turn_duration_s", "frame_stale_ms"):
            if getattr(self, name) <= 0:
                raise ValueError(f"{name} {getattr(self, name)!r} must be positive")
        for name in ("cruise_speed", "reverse_speed", "turn_rate"):
            if not 0.0 < getattr(self, name) <= 1.0:
                raise ValueError(f"{name} {getattr(self, name)!r} must be in (0.0, 1.0]")


class ReactivePlanner:
    """Turns the latest sensor frame into a 10-step, 2 s motor plan.

    While the path is clear the plan is ten cruise steps. An obstacle closer than
    ``stop_distance_cm`` starts a manoeuvre (stop, reverse, turn) that the planner then keeps to
    on every replan, whatever the sensor says, so replanning does not make it flap. A frame that
    is stale or has no valid distance gives a stop. Nothing is planned while disarmed.
    """

    def __init__(self, config: PlannerConfig | None = None) -> None:
        self.config = config or PlannerConfig()
        c = self.config
        self._cruise = motor_powers_from_command(Command(linear=c.cruise_speed, angular=0.0))
        self._reverse = motor_powers_from_command(Command(linear=-c.reverse_speed, angular=0.0))
        self._turn = motor_powers_from_command(Command(linear=0.0, angular=c.turn_rate))
        self._reverse_from_s = c.stop_duration_s
        self._turn_from_s = self._reverse_from_s + c.reverse_duration_s
        self._manoeuvre_s = self._turn_from_s + c.turn_duration_s
        self._manoeuvre_start_s: float | None = None

    def plan(self, frame: SensorFrame, now_s: float) -> MotionPlan | None:
        """The plan for the next 2 s, or None while disarmed.

        Args:
            frame: the latest sensor frame
            now_s: current time in seconds, on the same clock as ``frame.stamp_s``
        """
        if not frame.armed:
            self._manoeuvre_start_s = None
            return None

        stale = (now_s - frame.stamp_s) * 1000.0 > self.config.frame_stale_ms
        if stale or frame.distance_cm is None:
            self._manoeuvre_start_s = None
            return MotionPlan(steps=(ZERO_POWER,) * PLAN_STEPS)

        if self._manoeuvre_start_s is not None and (
            now_s - self._manoeuvre_start_s >= self._manoeuvre_s
        ):
            self._manoeuvre_start_s = None
        if self._manoeuvre_start_s is None and frame.distance_cm < self.config.stop_distance_cm:
            self._manoeuvre_start_s = now_s

        elapsed_s = 0.0 if self._manoeuvre_start_s is None else now_s - self._manoeuvre_start_s
        in_manoeuvre = self._manoeuvre_start_s is not None
        steps = tuple(
            self._power_at(elapsed_s + i * STEP_S) if in_manoeuvre else self._cruise
            for i in range(PLAN_STEPS)
        )
        return MotionPlan(steps=steps)

    def _power_at(self, since_start_s: float) -> MotorPowers:
        """The power ``since_start_s`` seconds into a manoeuvre (cruise once it is over)."""
        # A step that starts exactly on a phase boundary belongs to the next phase; the tolerance
        # keeps float error in `i * STEP_S` from putting it in the previous one.
        t = since_start_s + _BOUNDARY_TOLERANCE_S
        if t < self._reverse_from_s:
            return ZERO_POWER
        if t < self._turn_from_s:
            return self._reverse
        if t < self._manoeuvre_s:
            return self._turn
        return self._cruise
