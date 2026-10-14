"""Runs the avoider against FakeRobot in a fixed-rate loop, the way the planning node will."""

from picar_core import Avoider, AvoiderConfig, Command, FakeRobot, Observation, State, Stop

CONFIG = AvoiderConfig(
    stop_distance_cm=20.0,
    cruise_speed=0.5,
    turn_duration_s=1.0,
    range_stale_ms=500,
)
DT_MS = 100.0


def apply(robot: FakeRobot, command: Command) -> None:
    """Skid-steer the command onto the robot, as the action node does."""
    if command.is_zero():
        robot.stop()
    else:
        robot.set_wheel_speeds(command.linear - command.angular, command.linear + command.angular)


def run_loop(avoider: Avoider, robot: FakeRobot, ticks: int) -> list[State]:
    states = []
    for tick in range(ticks):
        distance = robot.read_distance_cm()
        snap = avoider.step(Observation(distance_cm=distance), now_ms=tick * DT_MS)
        apply(robot, snap.command)
        states.append(snap.state)
    return states


def test_obstacle_cycle_runs_stop_reverse_turn_then_cruise():
    # Clear, then an obstacle for 13 ticks (1.3 s), then clear again.
    robot = FakeRobot([100.0] + [15.0] * 13 + [100.0] * 10)
    avoider = Avoider(CONFIG)
    avoider.arm()

    states = run_loop(avoider, robot, ticks=24)

    expected = [State.CRUISE, State.STOP, State.REVERSE] + [State.TURN] * 10 + [State.CRUISE] * 11
    assert states == expected


def test_robot_is_stopped_while_stopped_and_moves_otherwise():
    robot = FakeRobot([100.0, 15.0, 15.0, 15.0])
    avoider = Avoider(CONFIG)
    avoider.arm()

    run_loop(avoider, robot, ticks=2)  # CRUISE, then STOP
    assert robot.is_stopped

    run_loop(avoider, robot, ticks=1)  # REVERSE
    assert robot.wheel_speeds[0] < 0 and robot.wheel_speeds[1] < 0


def test_sensor_dropout_mid_cruise_stops_the_robot_and_holds_fault():
    robot = FakeRobot([100.0] * 3 + [None] * 10)
    avoider = Avoider(CONFIG)
    avoider.arm()

    states = run_loop(avoider, robot, ticks=8)

    assert states[:3] == [State.CRUISE] * 3
    assert states[3:] == [State.FAULT] * 5
    assert robot.is_stopped


def test_disarm_while_moving_stops_the_robot_and_stays_silent():
    robot = FakeRobot([100.0] * 20)
    avoider = Avoider(CONFIG)
    avoider.arm()
    run_loop(avoider, robot, ticks=3)
    assert robot.wheel_speeds == (0.5, 0.5)

    avoider.disarm()
    states = run_loop(avoider, robot, ticks=3)
    commands_after_disarm = len(robot.commands)

    assert states == [State.DISARMED] * 3
    assert robot.is_stopped
    # Stop is sent once per tick while disarmed; no movement commands follow.
    run_loop(avoider, robot, ticks=3)
    assert all(isinstance(c, Stop) for c in robot.commands[commands_after_disarm:])
