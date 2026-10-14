"""MotionPlanner: turns the latest ultrasonic range into a 2 s motor plan.

Pure software, no hardware access. To swap in another planner (an ML model, say), replace this
node with one that subscribes to the controller's topics and publishes a
`trajectory_msgs/JointTrajectory` on `/planner/motion_plan` (see `picar_ros.interface`); the
controller enforces the safety limits whatever it sends.
"""

from __future__ import annotations

import dataclasses
import time

import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from picar_core import (
    MotionPlan,
    PlannerConfig,
    ReactivePlanner,
    SensorFrame,
    distance_cm_from_range_m,
)
from picar_ros.interface import ARMED_QOS, ARMED_TOPIC, MOTOR_JOINTS, PLAN_TOPIC, RANGE_TOPIC


class MotionPlannerNode(Node):
    def __init__(self) -> None:
        super().__init__("motion_planner")
        self.declare_parameter("rate_hz", 10.0)
        fields = dataclasses.fields(PlannerConfig)
        for field in fields:
            self.declare_parameter(field.name, field.default)

        self._planner = ReactivePlanner(
            PlannerConfig(**{f.name: self.get_parameter(f.name).value for f in fields})
        )
        self._distance_cm: float | None = None
        self._range_received_s: float | None = None
        self._armed = False

        self._pub = self.create_publisher(JointTrajectory, PLAN_TOPIC, 10)
        self.create_subscription(Range, RANGE_TOPIC, self._on_range, 10)
        self.create_subscription(Bool, ARMED_TOPIC, self._on_armed, ARMED_QOS)
        self.create_timer(1.0 / self.get_parameter("rate_hz").value, self._publish_plan)

    def _on_range(self, msg: Range) -> None:
        # Timed on arrival, so the age of the range is measured on this node's clock.
        self._distance_cm = distance_cm_from_range_m(msg.range)
        self._range_received_s = time.monotonic()

    def _on_armed(self, msg: Bool) -> None:
        self._armed = msg.data

    def _publish_plan(self) -> None:
        if self._range_received_s is None:
            return
        frame = SensorFrame(self._range_received_s, self._distance_cm, self._armed)
        plan = self._planner.plan(frame, time.monotonic())
        if plan is not None:
            self._pub.publish(self._to_msg(plan))

    def _to_msg(self, plan: MotionPlan) -> JointTrajectory:
        msg = JointTrajectory(
            joint_names=list(MOTOR_JOINTS),
            points=[
                JointTrajectoryPoint(
                    positions=list(powers), time_from_start=Duration(seconds=t).to_msg()
                )
                for t, powers in zip(plan.times_s, plan.steps)
            ],
        )
        msg.header.stamp = self.get_clock().now().to_msg()
        return msg


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MotionPlannerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()  # SIGINT has usually shut the context down already
