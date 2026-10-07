"""Command watchdog: ensures motors stop if no command is received."""

from __future__ import annotations

from typing import Callable

from picar_core.command import Command


class Watchdog:
    """Passes through the latest command, or outputs stop if it has gone stale.

    Args:
        timeout_ms: no command for this many milliseconds → output stop
        clock: callable returning the current time in seconds; defaults to ``time.time``
    """

    def __init__(
        self,
        timeout_ms: float = 500.0,
        clock: Callable[[], float] | None = None,
    ) -> None:
        if timeout_ms <= 0:
            raise ValueError(f"timeout_ms {timeout_ms!r} must be positive")
        self.timeout_s = timeout_ms / 1000.0
        self.clock = clock or __import__("time").time
        self._command: Command | None = None
        self._command_time: float | None = None

    def feed(self, command: Command) -> None:
        """Accept a new command from the planner."""
        self._command = command
        self._command_time = self.clock()

    def output(self) -> Command:
        """The current motor command: pass-through or stop if stale."""
        if self._command_time is None or (self.clock() - self._command_time) > self.timeout_s:
            return Command(linear=0.0, angular=0.0)
        assert self._command is not None  # guaranteed by feed()
        return self._command
