"""MotionPlanner: turns the latest `SensorFrame` into a 2 s `MotionPlan`.

Pure software, no hardware access. To swap in another planner (an ML model, say), replace this
node with one that subscribes to `/controller/sensor_frame` and publishes `/planner/motion_plan`
with the same message types; the controller enforces the safety limits whatever it sends.
"""

from __future__ import annotations

import time

import rclpy
from picar_interfaces.msg import MotionPlan as MotionPlanMsg
from picar_interfaces.msg import MotorPower
from picar_interfaces.msg import SensorFrame as SensorFrameMsg
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from picar_core import (
    MotionPlan,
    PlannerConfig,
    ReactivePlanner,
    SensorFrame,
    distance_cm_from_range_m,
)


class MotionPlannerNode(Node):
    def __init__(self) -> None:
        super().__init__("motion_planner")
        defaults = PlannerConfig()
        self.declare_parameter("rate_hz", 10.0)
        self.declare_parameter("stop_distance_cm", defaults.stop_distance_cm)
        self.declare_parameter("cruise_speed", defaults.cruise_speed)
        self.declare_parameter("reverse_speed", defaults.reverse_speed)
        self.declare_parameter("turn_rate", defaults.turn_rate)
        self.declare_parameter("stop_duration_s", defaults.stop_duration_s)
        self.declare_parameter("reverse_duration_s", defaults.reverse_duration_s)
        self.declare_parameter("turn_duration_s", defaults.turn_duration_s)
        self.declare_parameter("frame_stale_ms", defaults.frame_stale_ms)

        self._planner = ReactivePlanner(
            PlannerConfig(
                stop_distance_cm=self.get_parameter("stop_distance_cm").value,
                cruise_speed=self.get_parameter("cruise_speed").value,
                reverse_speed=self.get_parameter("reverse_speed").value,
                turn_rate=self.get_parameter("turn_rate").value,
                stop_duration_s=self.get_parameter("stop_duration_s").value,
                reverse_duration_s=self.get_parameter("reverse_duration_s").value,
                turn_duration_s=self.get_parameter("turn_duration_s").value,
                frame_stale_ms=self.get_parameter("frame_stale_ms").value,
            )
        )
        self._frame: SensorFrame | None = None

        self._pub = self.create_publisher(MotionPlanMsg, "/planner/motion_plan", 10)
        self.create_subscription(SensorFrameMsg, "/controller/sensor_frame", self._on_frame, 10)
        self.create_timer(1.0 / self.get_parameter("rate_hz").value, self._publish_plan)

    def _on_frame(self, msg: SensorFrameMsg) -> None:
        # Stamped with the local arrival time, so frame age is measured on this node's clock.
        self._frame = SensorFrame(
            stamp_s=time.monotonic(),
            distance_cm=distance_cm_from_range_m(msg.range_m),
            servo_angle_deg=msg.servo_angle_deg,
            greyscale=tuple(int(v) for v in msg.greyscale),
            last_motor_power=tuple(int(v) for v in msg.last_motor_power),
            armed=msg.armed,
        )

    def _publish_plan(self) -> None:
        if self._frame is None:
            return
        plan = self._planner.plan(self._frame, time.monotonic())
        if plan is not None:
            self._pub.publish(self._to_msg(plan))

    def _to_msg(self, plan: MotionPlan) -> MotionPlanMsg:
        msg = MotionPlanMsg()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.step_s = float(plan.step_s)
        for powers in plan.steps:
            step = MotorPower()
            step.power = list(powers)
            msg.steps.append(step)
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
