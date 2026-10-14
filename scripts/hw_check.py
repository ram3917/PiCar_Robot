"""Manual hardware check for the PiCar-4WD. Run on the robot with the wheels off the ground.

    PYTHONPATH=core python3 scripts/hw_check.py

Drives each side briefly in turn, then prints 20 ultrasonic readings to compare with a ruler
and the three greyscale values. Stop the container first (`docker compose stop picar`): only
one process may own the hardware. Note which values were wrong and how they were mapped in the PR.
"""

import time

from picar_core.picar4wd_adapter import Picar4WDHardware

POWER = 30
DRIVE_S = 1.0
READINGS = 20

# Motor order: left front, right front, left rear, right rear.
MOVES = [
    ("left side forward", (POWER, 0, POWER, 0)),
    ("right side forward", (0, POWER, 0, POWER)),
    ("left side backward", (-POWER, 0, -POWER, 0)),
    ("right side backward", (0, -POWER, 0, -POWER)),
    ("both sides forward", (POWER, POWER, POWER, POWER)),
]


def drive(hw: Picar4WDHardware, label: str, powers: tuple) -> None:
    print(f"{label}: powers={powers} for {DRIVE_S}s")
    hw.set_motor_power(powers)
    time.sleep(DRIVE_S)
    hw.stop()
    time.sleep(0.5)


def main() -> None:
    input("Wheels OFF the ground and the area clear? Press Enter to start...")
    hw = Picar4WDHardware()
    hw.set_servo_angle(0.0)
    try:
        for label, powers in MOVES:
            drive(hw, label, powers)

        print("Distance readings (cm), compare with a ruler:")
        for i in range(1, READINGS + 1):
            distance = hw.read_distance_cm()
            shown = "invalid (None)" if distance is None else f"{distance:.1f}"
            print(f"{i:2d}: {shown}   greyscale={hw.read_greyscale()}")
            time.sleep(0.2)
    finally:
        hw.stop()


if __name__ == "__main__":
    main()
