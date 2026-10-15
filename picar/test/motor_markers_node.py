"""Draws the four applied motor powers as RViz bars, for watching the robot by hand.

Reads /joint_states (the applied power in percent is the effort) and publishes a MarkerArray in
the ultrasonic frame: one bar per motor in the layout of the car, green forward and red reverse,
with a text label. The sensor is at the front, so the car extends toward -x from the frame.
"""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker, MarkerArray

from picar.controller_node import FRAME_ID
from picar.interface import JOINT_STATES_TOPIC, MOTOR_JOINTS

MARKERS_TOPIC = "/picar/motor_markers"
BAR_M_AT_FULL_POWER = 0.4
# (x, y) of each motor from the ultrasonic frame, left and front positive.
MOTOR_XY = {
    "left_front": (-0.05, 0.12),
    "right_front": (-0.05, -0.12),
    "left_rear": (-0.25, 0.12),
    "right_rear": (-0.25, -0.12),
}


def _marker(marker_id: int, kind: int, name: str) -> Marker:
    marker = Marker(id=marker_id, type=kind, action=Marker.ADD, ns=name)
    marker.header.frame_id = FRAME_ID
    return marker


class MotorMarkersNode(Node):
    def __init__(self) -> None:
        super().__init__("motor_markers")
        self._pub = self.create_publisher(MarkerArray, MARKERS_TOPIC, 10)
        self.create_subscription(JointState, JOINT_STATES_TOPIC, self._on_joints, 10)

    def _on_joints(self, msg: JointState) -> None:
        powers = dict(zip(msg.name, msg.effort))
        markers = MarkerArray()
        for index, joint in enumerate(MOTOR_JOINTS):
            power = powers.get(joint, math.nan)
            if math.isnan(power):
                power = 0.0
            x, y = MOTOR_XY[joint]
            height = max(abs(power) / 100.0 * BAR_M_AT_FULL_POWER, 0.005)

            bar = _marker(index, Marker.CUBE, "bar")
            bar.header.stamp = msg.header.stamp
            bar.pose.position.x, bar.pose.position.y = x, y
            bar.pose.position.z = height / 2.0 if power >= 0 else -height / 2.0
            bar.pose.orientation.w = 1.0
            bar.scale.x = bar.scale.y = 0.08
            bar.scale.z = height
            bar.color.a = 0.9
            bar.color.g, bar.color.r = (1.0, 0.0) if power >= 0 else (0.0, 1.0)
            markers.markers.append(bar)

            label = _marker(index, Marker.TEXT_VIEW_FACING, "label")
            label.header.stamp = msg.header.stamp
            label.pose.position.x, label.pose.position.y = x, y
            label.pose.position.z = 0.5
            label.pose.orientation.w = 1.0
            label.scale.z = 0.05
            label.color.a = label.color.r = label.color.g = label.color.b = 1.0
            label.text = f"{joint}\n{power:.0f}%"
            markers.markers.append(label)
        self._pub.publish(markers)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MotorMarkersNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
