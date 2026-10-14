"""Conversion between the core's centimetres and `sensor_msgs/Range` metres (REP 117)."""

from __future__ import annotations

import math


def range_m_from_cm(distance_cm: float | None, min_range_m: float, max_range_m: float) -> float:
    """The value to publish in a Range message.

    ``None`` (invalid reading) becomes NaN. A distance outside the sensor's range becomes
    -inf (too close) or +inf (nothing within max range), as REP 117 specifies.
    """
    if distance_cm is None or math.isnan(distance_cm):
        return math.nan
    distance_m = distance_cm / 100.0
    if distance_m < min_range_m:
        return -math.inf
    if distance_m > max_range_m:
        return math.inf
    return distance_m


def distance_cm_from_range_m(range_m: float) -> float | None:
    """The distance to feed the core from a Range message, or ``None`` if unusable.

    +inf means no echo within max range, which is a clear path, so it stays infinite.
    -inf (too close) and NaN are invalid readings.
    """
    if math.isnan(range_m) or range_m == -math.inf:
        return None
    return range_m * 100.0
