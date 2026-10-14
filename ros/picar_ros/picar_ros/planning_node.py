"""Planning: runs the avoidance state machine.

Subscribes `/ultrasonic/range`, publishes `/cmd_vel` and `/robot_state`, and serves `/arm`.
Publishes nothing on `/cmd_vel` while DISARMED so teleop can drive the robot.
"""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import String
from std_srvs.srv import SetBool

from picar_core import (
    Avoider,
    AvoiderConfig,
    Command,
    Observation,
    State,
    distance_cm_from_range_m,
)

TICK_HZ = 20.0
ZERO = Command(linear=0.0, angular=0.0)


class PlanningNode(Node):
    def __init__(self) -> None:
        super().__init__("planning")
        self.declare_parameter("stop_distance_cm", 20.0)
        self.declare_parameter("cruise_speed", 0.5)
        self.declare_parameter("turn_duration_s", 1.0)
        self.declare_parameter("range_stale_ms", 500.0)

        self._avoider = Avoider(
            AvoiderConfig(
                stop_distance_cm=self.get_parameter("stop_distance_cm").value,
                cruise_speed=self.get_parameter("cruise_speed").value,
                turn_duration_s=self.get_parameter("turn_duration_s").value,
                range_stale_ms=self.get_parameter("range_stale_ms").value,
            )
        )
        # None when the last message was unusable; _range_at_ms is the node-clock time it arrived.
        self._range_cm: float | None = None
        self._range_at_ms = 0.0

        self._cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self._state_pub = self.create_publisher(String, "/robot_state", 10)
        self.create_subscription(Range, "/ultrasonic/range", self._on_range, 10)
        self.create_service(SetBool, "/arm", self._on_arm)
        self.create_timer(1.0 / TICK_HZ, self._tick)

    def _now_ms(self) -> float:
        return self.get_clock().now().nanoseconds / 1e6

    def _on_range(self, msg: Range) -> None:
        self._range_cm = distance_cm_from_range_m(msg.range)
        self._range_at_ms = self._now_ms()

    def _tick(self) -> None:
        now_ms = self._now_ms()
        # A range older than range_stale_ms counts as no reading, so the core goes to FAULT.
        fresh = now_ms - self._range_at_ms <= self._avoider.config.range_stale_ms
        distance = self._range_cm if fresh else None

        snap = self._avoider.step(Observation(distance_cm=distance), now_ms)
        self._state_pub.publish(String(data=snap.state.value))
        if snap.state is not State.DISARMED:
            self._publish_command(snap.command)

    def _publish_command(self, command: Command) -> None:
        msg = Twist()
        msg.linear.x = command.linear
        msg.angular.z = command.angular
        self._cmd_pub.publish(msg)

    def _on_arm(self, request: SetBool.Request, response: SetBool.Response) -> SetBool.Response:
        if request.data:
            self._avoider.arm()
            response.success = self._avoider.state not in (State.DISARMED, State.FAULT)
        else:
            self._avoider.disarm()
            self._publish_command(ZERO)  # one stop on the way out, then silence
            response.success = True
        response.message = self._avoider.state.value
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PlanningNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()  # SIGINT has usually shut the context down already
