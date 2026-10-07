import pytest

from picar_core import Avoider, AvoiderConfig, Command, Observation, State


DEFAULT_CONFIG = AvoiderConfig(
    stop_distance_cm=20.0,
    cruise_speed=0.5,
    turn_duration_s=1.0,
    range_stale_ms=500,
)


def test_boots_disarmed():
    avoider = Avoider(DEFAULT_CONFIG)
    snap = avoider.step(Observation(distance_cm=100.0), now_ms=0.0)

    assert snap.state == State.DISARMED
    assert snap.command.is_zero()


def test_arm_transitions_to_cruise():
    avoider = Avoider(DEFAULT_CONFIG)
    avoider.arm()
    snap = avoider.step(Observation(distance_cm=100.0), now_ms=0.0)

    assert snap.state == State.CRUISE
    assert snap.command == Command(linear=0.5, angular=0.0)


def test_cruise_forward_when_clear():
    avoider = Avoider(DEFAULT_CONFIG)
    avoider.arm()
    snap = avoider.step(Observation(distance_cm=50.0), now_ms=0.0)

    assert snap.state == State.CRUISE
    assert snap.command == Command(linear=0.5, angular=0.0)


def test_cruise_to_stop_when_obstacle_detected():
    avoider = Avoider(DEFAULT_CONFIG)
    avoider.arm()
    avoider.step(Observation(distance_cm=100.0), now_ms=0.0)
    snap = avoider.step(Observation(distance_cm=15.0), now_ms=10.0)

    assert snap.state == State.STOP
    assert snap.command.is_zero()


def test_stop_to_reverse():
    avoider = Avoider(DEFAULT_CONFIG)
    avoider.arm()
    avoider.step(Observation(distance_cm=100.0), now_ms=0.0)
    avoider.step(Observation(distance_cm=15.0), now_ms=10.0)  # CRUISE -> STOP
    snap = avoider.step(Observation(distance_cm=15.0), now_ms=20.0)

    assert snap.state == State.REVERSE
    assert snap.command == Command(linear=-0.3, angular=0.0)


def test_reverse_to_turn():
    avoider = Avoider(DEFAULT_CONFIG)
    avoider.arm()
    avoider.step(Observation(distance_cm=100.0), now_ms=0.0)
    avoider.step(Observation(distance_cm=15.0), now_ms=10.0)  # CRUISE -> STOP
    avoider.step(Observation(distance_cm=15.0), now_ms=20.0)  # STOP -> REVERSE
    snap = avoider.step(Observation(distance_cm=15.0), now_ms=30.0)

    assert snap.state == State.TURN
    assert snap.command == Command(linear=0.0, angular=0.5)


def test_turn_to_cruise_after_duration():
    avoider = Avoider(DEFAULT_CONFIG)
    avoider.arm()
    avoider.step(Observation(distance_cm=100.0), now_ms=0.0)
    avoider.step(Observation(distance_cm=15.0), now_ms=10.0)  # CRUISE -> STOP
    avoider.step(Observation(distance_cm=15.0), now_ms=20.0)  # STOP -> REVERSE
    avoider.step(Observation(distance_cm=15.0), now_ms=30.0)  # REVERSE -> TURN
    snap = avoider.step(Observation(distance_cm=50.0), now_ms=1100.0)  # after 1000ms turn

    assert snap.state == State.CRUISE
    assert snap.command == Command(linear=0.5, angular=0.0)


def test_disarm_from_cruise():
    avoider = Avoider(DEFAULT_CONFIG)
    avoider.arm()
    avoider.step(Observation(distance_cm=100.0), now_ms=0.0)
    avoider.disarm()
    snap = avoider.step(Observation(distance_cm=50.0), now_ms=10.0)

    assert snap.state == State.DISARMED
    assert snap.command.is_zero()


def test_disarm_from_any_state():
    """Disarm wins from STOP, REVERSE, TURN and FAULT."""
    avoider = Avoider(DEFAULT_CONFIG)
    avoider.arm()
    avoider.step(Observation(distance_cm=100.0), now_ms=0.0)
    avoider.step(Observation(distance_cm=15.0), now_ms=10.0)  # CRUISE -> STOP
    avoider.step(Observation(distance_cm=15.0), now_ms=20.0)  # STOP -> REVERSE
    avoider.disarm()
    snap = avoider.step(Observation(distance_cm=50.0), now_ms=30.0)

    assert snap.state == State.DISARMED
    assert snap.command.is_zero()


def test_invalid_range_triggers_fault():
    avoider = Avoider(DEFAULT_CONFIG)
    avoider.arm()
    avoider.step(Observation(distance_cm=100.0), now_ms=0.0)
    avoider.step(Observation(distance_cm=100.0), now_ms=10.0)
    snap = avoider.step(Observation(distance_cm=None), now_ms=520.0)  # > 500ms stale

    assert snap.state == State.FAULT
    assert snap.command.is_zero()


def test_fault_while_disarmed_is_ignored():
    """Disarmed state cannot enter FAULT; invalid range is just ignored."""
    avoider = Avoider(DEFAULT_CONFIG)
    snap = avoider.step(Observation(distance_cm=None), now_ms=1000.0)

    assert snap.state == State.DISARMED
    assert snap.command.is_zero()


def test_stale_range_tracked_across_reads():
    """Range timestamp is updated only on valid readings."""
    avoider = Avoider(DEFAULT_CONFIG)
    avoider.arm()
    avoider.step(Observation(distance_cm=100.0), now_ms=0.0)
    avoider.step(Observation(distance_cm=100.0), now_ms=100.0)
    avoider.step(Observation(distance_cm=None), now_ms=200.0)  # invalid read
    snap = avoider.step(Observation(distance_cm=None), now_ms=600.0)  # > 500ms since valid

    assert snap.state == State.FAULT


def test_range_recovery_from_fault():
    """FAULT continues until disarmed."""
    avoider = Avoider(DEFAULT_CONFIG)
    avoider.arm()
    avoider.step(Observation(distance_cm=100.0), now_ms=0.0)
    avoider.step(Observation(distance_cm=None), now_ms=520.0)  # FAULT
    snap = avoider.step(Observation(distance_cm=100.0), now_ms=530.0)

    # Still in FAULT; disarm to leave
    assert snap.state == State.FAULT
    avoider.disarm()
    snap = avoider.step(Observation(distance_cm=100.0), now_ms=540.0)

    assert snap.state == State.DISARMED
