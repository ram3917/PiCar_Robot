"""Planner: turns the latest ultrasonic range into a /cmd_vel drive command.

Pure software, no hardware access. To swap in another planner (an ML model, say), replace this
node with one that subscribes to the controller's range topic and publishes a
`geometry_msgs/Twist` on /cmd_vel (see `picar.interface`); the controller enforces the safety
limits whatever it sends.

The `drive` parameter is the on switch. While it is false nothing is published, so the controller
stops the motors when its command timeout runs out. Flip it live with
``ros2 param set /planner drive true``.
"""

from __future__ import annotations

import dataclasses
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Range

from picar.interface import CMD_VEL_TOPIC, RANGE_TOPIC, valid_range_m
from picar.planner import PlannerConfig, ReactivePlanner


class PlannerNode(Node):
    def __init__(self) -> None:
        super().__init__("planner")
        self.declare_parameter("drive", False)
        self.declare_parameter("rate_hz", 10.0)
        fields = dataclasses.fields(PlannerConfig)
        for field in fields:
            self.declare_parameter(field.name, field.default)

        self._planner = ReactivePlanner(
            PlannerConfig(**{f.name: self.get_parameter(f.name).value for f in fields})
        )
        self._range_m: float | None = None
        self._range_received_s: float | None = None

        self._pub = self.create_publisher(Twist, CMD_VEL_TOPIC, 10)
        self.create_subscription(Range, RANGE_TOPIC, self._on_range, 10)
        self.create_timer(1.0 / self.get_parameter("rate_hz").value, self._publish_command)

    def _on_range(self, msg: Range) -> None:
        # Timed on arrival, so the age of the range is measured on this node's clock.
        self._range_m = valid_range_m(msg)
        self._range_received_s = time.monotonic()

    def _publish_command(self) -> None:
        if self._range_received_s is None or not self.get_parameter("drive").value:
            return
        now_s = time.monotonic()
        self._pub.publish(self._planner.step(self._range_m, now_s - self._range_received_s, now_s))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PlannerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()  # SIGINT has usually shut the context down already
