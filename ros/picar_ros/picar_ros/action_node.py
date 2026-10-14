"""Action: turns /cmd_vel into wheel speeds and drives the motors behind the command watchdog.

If no /cmd_vel arrives for cmd_timeout_ms, the motors stop, so a dead planning node cannot
leave the robot running.
"""

import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

from picar_core import Watchdog, command_from_twist, wheel_speeds
from picar_ros.hardware import make_hardware

TICK_HZ = 50.0


class ActionNode(Node):
    def __init__(self) -> None:
        super().__init__("action")
        self.declare_parameter("hardware", "fake")
        self.declare_parameter("cmd_timeout_ms", 500.0)

        self._hardware = make_hardware(self.get_parameter("hardware").value)
        self._watchdog = Watchdog(
            timeout_ms=self.get_parameter("cmd_timeout_ms").value,
            clock=time.monotonic,
        )

        self.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel, 10)
        self.create_timer(1.0 / TICK_HZ, self._drive)

    def _on_cmd_vel(self, msg: Twist) -> None:
        self._watchdog.feed(command_from_twist(msg.linear.x, msg.angular.z))

    def _drive(self) -> None:
        left, right = wheel_speeds(self._watchdog.output())
        self._hardware.set_wheel_speeds(left, right)

    def stop_motors(self) -> None:
        self._hardware.stop()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ActionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_motors()
        node.destroy_node()
        rclpy.shutdown()
