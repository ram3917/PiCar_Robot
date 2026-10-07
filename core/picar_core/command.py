"""The command: linear and angular velocities for skid-steer driving."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class Command:
    """Linear and angular velocity in normalised units.

    Both are in ``[-1.0, 1.0]`` where negative is backwards / counter-clockwise.
    """

    linear: float
    angular: float

    def __post_init__(self) -> None:
        for name, value in (("linear", self.linear), ("angular", self.angular)):
            if not -1.0 <= value <= 1.0:
                raise ValueError(f"{name} velocity {value!r} is outside [-1.0, 1.0]")

    def is_zero(self) -> bool:
        return self.linear == 0.0 and self.angular == 0.0
