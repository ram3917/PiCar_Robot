import math

import pytest
from sensor_msgs.msg import Range

from picar_core import SetServoAngle
from picar_ros.perception_node import PerceptionNode


def test_publishes_range_in_metres_with_sensor_metadata(ros):
    ros.start(PerceptionNode, hardware="fake", fake_distances_cm=[100.0])
    ranges = ros.record(Range, "/ultrasonic/range")

    assert ros.spin_until(lambda: len(ranges) >= 2)
    msg = ranges[-1]
    assert msg.range == pytest.approx(1.0)
    assert msg.header.frame_id == "ultrasonic_link"
    assert msg.radiation_type == Range.ULTRASOUND
    assert msg.min_range == pytest.approx(0.02)
    assert msg.max_range == pytest.approx(4.0)
    assert msg.field_of_view > 0.0


@pytest.mark.parametrize(
    "distance_cm, expected_m",
    [
        (1.0, -math.inf),  # closer than min_range
        (500.0, math.inf),  # beyond max_range: no echo, clear path
        (math.nan, math.nan),  # invalid reading
    ],
    ids=["too-close", "out-of-range", "invalid"],
)
def test_out_of_range_readings_follow_rep_117(ros, distance_cm, expected_m):
    ros.start(PerceptionNode, hardware="fake", fake_distances_cm=[distance_cm])
    ranges = ros.record(Range, "/ultrasonic/range")

    assert ros.spin_until(lambda: len(ranges) >= 1)
    if math.isnan(expected_m):
        assert math.isnan(ranges[-1].range)
    else:
        assert ranges[-1].range == expected_m


def test_publishes_at_about_10_hz(ros):
    ros.start(PerceptionNode, hardware="fake")
    ranges = ros.record(Range, "/ultrasonic/range")
    assert ros.spin_until(lambda: len(ranges) >= 1)

    count_before = len(ranges)
    ros.spin_for(1.0)

    assert 7 <= len(ranges) - count_before <= 13


def test_locks_the_servo_forward(ros):
    node = ros.start(PerceptionNode, hardware="fake")

    assert node._hardware.commands[0] == SetServoAngle(0.0)
