"""Manual hardware check for the PiCar-4WD. Run on the robot with the wheels off the ground.

    PYTHONPATH=core python3 scripts/hw_check.py

Drives each side briefly in turn, then prints 20 ultrasonic readings to compare with a ruler.
Note which values were wrong and how they were mapped in the PR (MVP-06).
"""

import time

from picar_core.picar4wd_adapter import Picar4WDHardware

SPEED = 0.3
DRIVE_S = 1.0
READINGS = 20

MOVES = [
    ("left side forward", SPEED, 0.0),
    ("right side forward", 0.0, SPEED),
    ("left side backward", -SPEED, 0.0),
    ("right side backward", 0.0, -SPEED),
    ("both sides forward", SPEED, SPEED),
]


def drive(hw: Picar4WDHardware, label: str, left: float, right: float) -> None:
    print(f"{label}: left={left} right={right} for {DRIVE_S}s")
    hw.set_wheel_speeds(left, right)
    time.sleep(DRIVE_S)
    hw.stop()
    time.sleep(0.5)


def main() -> None:
    input("Wheels OFF the ground and the area clear? Press Enter to start...")
    hw = Picar4WDHardware()
    hw.set_servo_angle(0.0)
    try:
        for label, left, right in MOVES:
            drive(hw, label, left, right)

        print("Distance readings (cm), compare with a ruler:")
        for i in range(1, READINGS + 1):
            distance = hw.read_distance_cm()
            shown = "invalid (None)" if distance is None else f"{distance:.1f}"
            print(f"{i:2d}: {shown}")
            time.sleep(0.2)
    finally:
        hw.stop()


if __name__ == "__main__":
    main()
