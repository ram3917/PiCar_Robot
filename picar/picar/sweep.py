"""The ultrasonic servo sweep: which angle to point at on each sensor tick.

The sweep runs from ``min_deg`` to ``max_deg`` and back, forever. Each pass visits every angle once
and ends on the angle it started from the other side. Positive angles are to the left, as in
`picar.hardware.RobotHardware.set_servo_angle`.

Pure logic, no ROS, so it is tested on its own.
"""

from __future__ import annotations

import math
from collections import deque

FORWARD_CONE_DEG = 15.0  # the HC-SR04 beam is about 15 degrees wide; readings this close are "ahead"


def in_forward_cone(angle_deg: float) -> bool:
    """True if a reading taken at ``angle_deg`` is a reading of what is ahead of the robot."""
    return abs(angle_deg) <= FORWARD_CONE_DEG


class SweepPlan:
    """Hands out the servo angle for each tick, and says when a pass has finished.

    Args:
        min_deg: one end of the sweep, in degrees, within [-90, 90]
        max_deg: the other end, greater than ``min_deg``
        step_deg: the angle change per tick; ``max_deg - min_deg`` must be a whole number of steps
    """

    def __init__(self, min_deg: float, max_deg: float, step_deg: float) -> None:
        for name, value in (("min_deg", min_deg), ("max_deg", max_deg), ("step_deg", step_deg)):
            if not math.isfinite(value):
                raise ValueError(f"{name} {value!r} is not finite")
        if not -90.0 <= min_deg < max_deg <= 90.0:
            raise ValueError(f"sweep needs -90 <= min_deg < max_deg <= 90, got {min_deg}..{max_deg}")
        if step_deg <= 0:
            raise ValueError(f"step_deg {step_deg!r} must be positive")
        steps = (max_deg - min_deg) / step_deg
        if abs(steps - round(steps)) > 1e-9:
            raise ValueError(f"the sweep {min_deg}..{max_deg} is not a whole number of {step_deg} steps")

        self._angles = [min_deg + i * step_deg for i in range(round(steps) + 1)]
        self._outward = True
        self._queue: deque[float] = deque(self._angles)

    @property
    def angles(self) -> list[float]:
        """Every angle of one pass, in ascending order."""
        return list(self._angles)

    def next_angle(self) -> tuple[float, bool]:
        """The angle to point at now, and whether it is the last of its pass.

        Each pass visits every angle in `angles` once. The return pass runs the other way.
        """
        if not self._queue:
            self._outward = not self._outward
            self._queue = deque(self._angles if self._outward else reversed(self._angles))
        angle = self._queue.popleft()
        return angle, not self._queue
