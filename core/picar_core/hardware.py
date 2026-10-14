"""The hardware boundary: everything the robot logic may ask of the physical robot."""

from __future__ import annotations

from typing import Protocol, Sequence, Tuple, runtime_checkable


@runtime_checkable
class RobotHardware(Protocol):
    """Motors, ultrasonic sensor, greyscale sensors and sensor servo of a 4-motor robot.

    Implementations must not raise for a bad sensor reading; they report it as ``None``.
    """

    def set_motor_power(self, powers: Sequence[int]) -> None:
        """Drive the four motors (left front, right front, left rear, right rear).

        Each power is an int in ``[-100, 100]``; negative is reverse.
        """
        ...

    def stop(self) -> None:
        """Stop all motors."""
        ...

    def read_distance_cm(self) -> float | None:
        """Distance to the nearest obstacle in centimetres, or ``None`` if the reading is invalid."""
        ...

    def read_greyscale(self) -> Tuple[int, int, int]:
        """The three downward greyscale ADC readings."""
        ...

    def set_servo_angle(self, deg: float) -> None:
        """Point the ultrasonic sensor; ``0`` is straight ahead, positive is to the left."""
        ...
