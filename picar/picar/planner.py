"""Reactive obstacle avoidance: cruise until something is close, then stop, reverse and turn."""

from __future__ import annotations

from dataclasses import dataclass

from geometry_msgs.msg import Twist


@dataclass(frozen=True)
class PlannerConfig:
    """Parameters for the reactive planner. Speeds are normalized, in (0, 1].

    Args:
        stop_distance_m: an obstacle closer than this starts stop -> reverse -> turn
        cruise_speed: forward speed while cruising
        reverse_speed: backward speed while reversing
        turn_speed: spin speed while turning counter-clockwise
        stop_duration_s: how long the robot holds still before reversing
        reverse_duration_s: how long it reverses
        turn_duration_s: how long it turns
        range_stale_ms: a range older than this is a fault and the command is a stop
    """

    stop_distance_m: float = 0.20
    cruise_speed: float = 0.5
    reverse_speed: float = 0.3
    turn_speed: float = 0.5
    stop_duration_s: float = 0.2
    reverse_duration_s: float = 0.4
    turn_duration_s: float = 1.0
    range_stale_ms: float = 500.0


class ReactivePlanner:
    """Turns the latest range into a drive command.

    While the path is clear it cruises. An obstacle closer than ``stop_distance_m`` starts a
    manoeuvre (stop, reverse, turn) that the planner then keeps to whatever the sensor says, so
    it does not flap. A range that is stale or invalid gives a stop.
    """

    def __init__(self, config: PlannerConfig | None = None) -> None:
        self.config = config or PlannerConfig()
        c = self.config
        self._reverse_from_s = c.stop_duration_s
        self._turn_from_s = self._reverse_from_s + c.reverse_duration_s
        self._manoeuvre_s = self._turn_from_s + c.turn_duration_s
        self._manoeuvre_start_s: float | None = None

    def step(self, range_m: float | None, range_age_s: float, now_s: float) -> Twist:
        """The command to send now.

        Args:
            range_m: the latest forward distance, or None if the reading was invalid
            range_age_s: how long ago that range arrived
            now_s: current time in seconds, on any steadily running clock
        """
        if range_age_s * 1000.0 > self.config.range_stale_ms or range_m is None:
            self._manoeuvre_start_s = None
            return Twist()

        if self._manoeuvre_start_s is not None and (
            now_s - self._manoeuvre_start_s >= self._manoeuvre_s
        ):
            self._manoeuvre_start_s = None
        if self._manoeuvre_start_s is None and range_m < self.config.stop_distance_m:
            self._manoeuvre_start_s = now_s

        if self._manoeuvre_start_s is None:
            return _twist(linear=self.config.cruise_speed)
        since_start_s = now_s - self._manoeuvre_start_s
        if since_start_s < self._reverse_from_s:
            return Twist()
        if since_start_s < self._turn_from_s:
            return _twist(linear=-self.config.reverse_speed)
        return _twist(angular=self.config.turn_speed)


def _twist(linear: float = 0.0, angular: float = 0.0) -> Twist:
    twist = Twist()
    twist.linear.x = linear
    twist.angular.z = angular
    return twist
