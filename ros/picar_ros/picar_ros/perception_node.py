"""Perception: reads the ultrasonic sensor and publishes `/ultrasonic/range`."""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Range

from picar_core import range_m_from_cm
from picar_ros.hardware import make_hardware

RATE_HZ = 10.0
MIN_RANGE_M = 0.02
MAX_RANGE_M = 4.0
FIELD_OF_VIEW_RAD = 0.26  # about 15 degrees, the HC-SR04 beam
FRAME_ID = "ultrasonic_link"


class PerceptionNode(Node):
    def __init__(self) -> None:
        super().__init__("perception")
        self.declare_parameter("hardware", "fake")
        self.declare_parameter("fake_distances_cm", [100.0])

        self._hardware = make_hardware(
            self.get_parameter("hardware").value,
            fake_distances_cm=self.get_parameter("fake_distances_cm").value,
        )
        self._hardware.set_servo_angle(0.0)  # the sensor stays locked forward in the MVP

        self._pub = self.create_publisher(Range, "/ultrasonic/range", 10)
        self.create_timer(1.0 / RATE_HZ, self._publish_range)

    def _publish_range(self) -> None:
        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = FRAME_ID
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = FIELD_OF_VIEW_RAD
        msg.min_range = MIN_RANGE_M
        msg.max_range = MAX_RANGE_M
        msg.range = range_m_from_cm(self._hardware.read_distance_cm(), MIN_RANGE_M, MAX_RANGE_M)
        self._pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()  # SIGINT has usually shut the context down already
