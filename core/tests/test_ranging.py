import math

import pytest

from picar_core import distance_cm_from_range_m, range_m_from_cm

MIN_M = 0.02
MAX_M = 4.0


def test_a_distance_in_range_is_published_in_metres():
    assert range_m_from_cm(50.0, MIN_M, MAX_M) == pytest.approx(0.5)


def test_an_invalid_reading_is_published_as_nan():
    assert math.isnan(range_m_from_cm(None, MIN_M, MAX_M))


def test_closer_than_the_minimum_range_is_negative_infinity():
    assert range_m_from_cm(1.0, MIN_M, MAX_M) == -math.inf


def test_beyond_the_maximum_range_is_positive_infinity():
    assert range_m_from_cm(500.0, MIN_M, MAX_M) == math.inf


def test_nan_and_negative_infinity_are_invalid_readings():
    assert distance_cm_from_range_m(math.nan) is None
    assert distance_cm_from_range_m(-math.inf) is None


def test_positive_infinity_is_a_clear_path_not_an_invalid_reading():
    assert distance_cm_from_range_m(math.inf) == math.inf


def test_a_finite_range_is_converted_to_centimetres():
    assert distance_cm_from_range_m(0.25) == pytest.approx(25.0)


@pytest.mark.parametrize("distance_cm", [2.0, 25.0, 399.0])
def test_round_trip_within_the_sensor_range(distance_cm):
    published = range_m_from_cm(distance_cm, MIN_M, MAX_M)

    assert distance_cm_from_range_m(published) == pytest.approx(distance_cm)
