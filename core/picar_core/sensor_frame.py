"""One snapshot of the onboard sensors: the planner's input."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Tuple

from picar_core.motors import MotorPowers, ZERO_POWER, validate_motor_powers


@dataclass(frozen=True)
class SensorFrame:
    """Everything the planner may look at.

    Args:
        stamp_s: when the frame was taken, in seconds on the reader's clock
        distance_cm: forward ultrasonic distance, or None if the reading is invalid
        servo_angle_deg: ultrasonic servo angle; 0 is straight ahead
        greyscale: the three downward ADC readings
        last_motor_power: the motor powers the controller last applied
        armed: whether the robot is armed
    """

    stamp_s: float
    distance_cm: float | None
    servo_angle_deg: float = 0.0
    greyscale: Tuple[int, int, int] = (0, 0, 0)
    last_motor_power: MotorPowers = ZERO_POWER
    armed: bool = False

    def __post_init__(self) -> None:
        if len(self.greyscale) != 3:
            raise ValueError(f"expected 3 greyscale values, got {len(self.greyscale)}")
        object.__setattr__(self, "greyscale", tuple(self.greyscale))
        object.__setattr__(self, "last_motor_power", validate_motor_powers(self.last_motor_power))
