"""Chooses the RobotHardware implementation from the `hardware` parameter."""

from typing import Iterable

from picar_core import FakeRobot, RobotHardware
from picar_core.picar4wd_adapter import Picar4WDHardware

HARDWARE_NAMES = ("fake", "picar4wd")


def make_hardware(name: str, fake_distances_cm: Iterable[float] = ()) -> RobotHardware:
    """Build the hardware adapter named by the `hardware` parameter.

    Args:
        name: ``fake`` (in memory) or ``picar4wd`` (the real robot)
        fake_distances_cm: scripted readings for ``fake``; the last one repeats
    """
    if name == "fake":
        return FakeRobot(fake_distances_cm)
    if name == "picar4wd":
        return Picar4WDHardware()
    raise ValueError(f"unknown hardware {name!r}; expected one of {HARDWARE_NAMES}")
