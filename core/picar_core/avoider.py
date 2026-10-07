"""Reactive obstacle-avoidance state machine."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

from picar_core.command import Command
from picar_core.observation import Observation


class State(Enum):
    """Avoidance state machine states."""

    DISARMED = "DISARMED"
    CRUISE = "CRUISE"
    STOP = "STOP"
    REVERSE = "REVERSE"
    TURN = "TURN"
    FAULT = "FAULT"


@dataclass(frozen=True)
class AvoiderConfig:
    """Parameters for the avoidance state machine.

    Args:
        stop_distance_cm: obstacle closer than this triggers stop/reverse/turn
        cruise_speed: forward velocity in [-1.0, 1.0]
        turn_duration_s: how long a turn lasts
        range_stale_ms: range older than this is treated as a fault
    """

    stop_distance_cm: float
    cruise_speed: float
    turn_duration_s: float
    range_stale_ms: float

    def __post_init__(self) -> None:
        if self.stop_distance_cm <= 0:
            raise ValueError(f"stop_distance_cm {self.stop_distance_cm!r} must be positive")
        if not 0.0 < self.cruise_speed <= 1.0:
            raise ValueError(f"cruise_speed {self.cruise_speed!r} must be in (0.0, 1.0]")
        if self.turn_duration_s <= 0:
            raise ValueError(f"turn_duration_s {self.turn_duration_s!r} must be positive")
        if self.range_stale_ms <= 0:
            raise ValueError(f"range_stale_ms {self.range_stale_ms!r} must be positive")


@dataclass(frozen=True)
class AvoiderSnapshot:
    """State and command at a point in time."""

    state: State
    command: Command
    range_timestamp_ms: float  # when the last non-None distance was observed


class Avoider:
    """Reactive obstacle-avoidance state machine.

    Boots DISARMED. Call ``step(observation, now)`` repeatedly to update the state and
    generate motor commands.
    """

    def __init__(self, config: AvoiderConfig) -> None:
        self.config = config
        self._state = State.DISARMED
        self._command = Command(linear=0.0, angular=0.0)
        self._range_timestamp_ms = 0.0
        self._state_start_ms = 0.0

    def step(self, observation: Observation, now_ms: float) -> AvoiderSnapshot:
        """Run one cycle of the state machine.

        Args:
            observation: sensor readings
            now_ms: current time in milliseconds

        Returns:
            AvoiderSnapshot with the new state and command
        """
        # Track range timestamp if we got a valid reading
        if observation.distance_cm is not None:
            self._range_timestamp_ms = now_ms

        # Check if range is stale or invalid
        range_valid = (
            observation.distance_cm is not None
            and (now_ms - self._range_timestamp_ms) <= self.config.range_stale_ms
        )

        if not range_valid and self._state != State.DISARMED:
            self._state = State.FAULT
            self._command = Command(linear=0.0, angular=0.0)
            return AvoiderSnapshot(
                state=self._state,
                command=self._command,
                range_timestamp_ms=self._range_timestamp_ms,
            )

        # State transitions and command generation
        if self._state == State.DISARMED:
            self._command = Command(linear=0.0, angular=0.0)
        elif self._state == State.CRUISE:
            if observation.distance_cm is not None and observation.distance_cm < self.config.stop_distance_cm:
                self._state = State.STOP
                self._state_start_ms = now_ms
                self._command = Command(linear=0.0, angular=0.0)
            else:
                self._command = Command(linear=self.config.cruise_speed, angular=0.0)
        elif self._state == State.STOP:
            self._state = State.REVERSE
            self._state_start_ms = now_ms
            self._command = Command(linear=-0.3, angular=0.0)
        elif self._state == State.REVERSE:
            self._state = State.TURN
            self._state_start_ms = now_ms
            self._command = Command(linear=0.0, angular=0.5)
        elif self._state == State.TURN:
            elapsed_ms = now_ms - self._state_start_ms
            if elapsed_ms >= self.config.turn_duration_s * 1000:
                self._state = State.CRUISE
                self._command = Command(linear=self.config.cruise_speed, angular=0.0)
            else:
                self._command = Command(linear=0.0, angular=0.5)
        elif self._state == State.FAULT:
            self._command = Command(linear=0.0, angular=0.0)

        return AvoiderSnapshot(
            state=self._state,
            command=self._command,
            range_timestamp_ms=self._range_timestamp_ms,
        )

    def arm(self) -> None:
        """Transition from DISARMED to CRUISE."""
        if self._state == State.DISARMED:
            self._state = State.CRUISE
            self._state_start_ms = 0.0

    def disarm(self) -> None:
        """Transition any state to DISARMED with zero command."""
        self._state = State.DISARMED
        self._command = Command(linear=0.0, angular=0.0)
