from picar_core import Command, Watchdog


class ManualClock:
    def __init__(self, now: float = 100.0) -> None:
        self.now = now

    def __call__(self) -> float:
        return self.now

    def advance(self, seconds: float) -> None:
        self.now += seconds


def test_fresh_command_passes_through():
    clock = ManualClock()
    watchdog = Watchdog(clock=clock)

    watchdog.feed(Command(linear=0.3, angular=0.1))
    clock.advance(0.1)

    assert watchdog.output() == Command(linear=0.3, angular=0.1)


def test_stale_command_returns_stop():
    clock = ManualClock()
    watchdog = Watchdog(timeout_ms=200, clock=clock)

    watchdog.feed(Command(linear=0.3, angular=0.1))
    clock.advance(0.25)

    assert watchdog.output() == Command(linear=0.0, angular=0.0)


def test_recovery_after_fresh_command():
    clock = ManualClock()
    watchdog = Watchdog(timeout_ms=200, clock=clock)

    watchdog.feed(Command(linear=0.3, angular=0.1))
    clock.advance(0.25)
    assert watchdog.output() == Command(linear=0.0, angular=0.0)

    watchdog.feed(Command(linear=0.5, angular=-0.2))
    clock.advance(0.1)

    assert watchdog.output() == Command(linear=0.5, angular=-0.2)
