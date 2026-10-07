"""The hardware boundary: everything the robot logic may ask of the physical robot."""

from __future__ import annotations

from typing import Protocol, runtime_checkable


@runtime_checkable
class RobotHardware(Protocol):
    """Motors, ultrasonic sensor and sensor servo of a skid-steer robot.

    Implementations must not raise for a bad sensor reading; they report it as ``None``.
    """

    def set_wheel_speeds(self, left: float, right: float) -> None:
        """Drive each side at a normalised speed in ``[-1.0, 1.0]``; negative is reverse."""
        ...

    def stop(self) -> None:
        """Stop all motors."""
        ...

    def read_distance_cm(self) -> float | None:
        """Distance to the nearest obstacle in centimetres, or ``None`` if the reading is invalid."""
        ...

    def set_servo_angle(self, deg: float) -> None:
        """Point the ultrasonic sensor; ``0`` is straight ahead, positive is to the left."""
        ...
