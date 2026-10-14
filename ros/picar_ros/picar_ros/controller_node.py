"""Controller: the one node that owns the robot hardware.

It reads the sensors, publishes a `SensorFrame` every sensor_period_ms, plays the latest
`MotionPlan` onto the motors every motor_tick_ms, and owns arming and every safety limit
(see `picar_core.MotorGovernor`). Nothing else may touch the hardware: the picar-4wd library
resets the HAT and claims the GPIO lines when it is imported, so a second owner breaks the first.

Two worker threads do the timed work, because an ultrasonic read blocks for up to ~20 ms:

- the sensor thread reads the ultrasonic sensor and the greyscale ADC and publishes the frame;
- the motor thread asks the governor for the powers and writes them when they change.

The ultrasonic read is plain GPIO and runs unlocked. The greyscale ADC and the motors share the
HAT's I2C address, so those calls take ``_i2c_lock`` and can never interleave.
"""

from __future__ import annotations

import threading
import time

import rclpy
from picar_interfaces.msg import MotionPlan as MotionPlanMsg
from picar_interfaces.msg import SensorFrame as SensorFrameMsg
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_srvs.srv import SetBool

from picar_core import (
    ZERO_POWER,
    GovernorConfig,
    MotionPlan,
    MotorGovernor,
    MotorPowers,
    RobotHardware,
    distance_cm_from_range_m,
    range_m_from_cm,
)
from picar_ros.hardware import make_hardware

MIN_RANGE_M = 0.02
MAX_RANGE_M = 4.0
FRAME_ID = "ultrasonic_link"
MAX_ADC = 65535  # SensorFrame.greyscale is uint16


class ControllerNode(Node):
    def __init__(self, hardware: RobotHardware | None = None) -> None:
        super().__init__("controller")
        self.declare_parameter("hardware", "fake")
        self.declare_parameter("fake_distances_cm", [100.0])
        self.declare_parameter("sensor_period_ms", 30.0)
        self.declare_parameter("motor_tick_ms", 10.0)
        self.declare_parameter("hard_stop_cm", 10.0)
        self.declare_parameter("range_stale_ms", 150.0)
        self.declare_parameter("plan_timeout_ms", 500.0)
        self.declare_parameter("slow_write_ms", 100.0)

        self._hardware = hardware or make_hardware(
            self.get_parameter("hardware").value,
            fake_distances_cm=self.get_parameter("fake_distances_cm").value,
        )
        self._sensor_period_s = self.get_parameter("sensor_period_ms").value / 1000.0
        self._motor_tick_s = self.get_parameter("motor_tick_ms").value / 1000.0
        self._slow_write_s = self.get_parameter("slow_write_ms").value / 1000.0
        self._governor = MotorGovernor(
            GovernorConfig(
                hard_stop_cm=self.get_parameter("hard_stop_cm").value,
                range_stale_ms=self.get_parameter("range_stale_ms").value,
                plan_timeout_ms=self.get_parameter("plan_timeout_ms").value,
            )
        )

        self._state_lock = threading.Lock()  # the governor and _applied
        self._i2c_lock = threading.Lock()  # greyscale reads and motor writes
        # What was last written; None until the first write, which makes the motor thread stop
        # the motors explicitly at startup.
        self._applied: MotorPowers | None = None
        self._stopping = threading.Event()

        self._hardware.set_servo_angle(0.0)  # the sensor stays locked forward in v0.2
        self._frame_pub = self.create_publisher(SensorFrameMsg, "/controller/sensor_frame", 10)
        self.create_subscription(MotionPlanMsg, "/planner/motion_plan", self._on_plan, 10)
        self.create_service(SetBool, "/arm", self._on_arm)

        self._threads = [
            threading.Thread(target=self._sensor_loop, name="sensors", daemon=True),
            threading.Thread(target=self._motor_loop, name="motors", daemon=True),
        ]
        for thread in self._threads:
            thread.start()

    # ROS callbacks (executor thread)

    def _on_arm(self, request: SetBool.Request, response: SetBool.Response) -> SetBool.Response:
        with self._state_lock:
            self._governor.set_armed(request.data)
        response.success = True
        response.message = "ARMED" if request.data else "DISARMED"
        self.get_logger().info(response.message)
        return response

    def _on_plan(self, msg: MotionPlanMsg) -> None:
        try:
            plan = MotionPlan(steps=[tuple(step.power) for step in msg.steps], step_s=msg.step_s)
        except ValueError as error:
            self.get_logger().warning(f"ignored a bad motion plan: {error}")
            return
        with self._state_lock:
            self._governor.set_plan(plan, time.monotonic())

    # Worker threads

    def _sensor_loop(self) -> None:
        next_at = time.monotonic()
        while not self._stopping.is_set():
            try:
                self._sense_once()
            except Exception as error:  # a failed read must not kill the thread
                # No update reaches the governor, so forward drive is blocked once the range
                # goes stale.
                self.get_logger().error(f"sensor read failed: {error!r}", throttle_duration_sec=1.0)
            next_at = self._sleep_until(next_at + self._sensor_period_s)

    def _sense_once(self) -> None:
        distance_cm = self._hardware.read_distance_cm()
        with self._i2c_lock:
            greyscale = self._hardware.read_greyscale()
        with self._state_lock:
            self._governor.update_range(distance_cm, time.monotonic())
            armed = self._governor.armed
            applied = self._applied or ZERO_POWER
        self._publish_frame(distance_cm, greyscale, applied, armed)

    def _publish_frame(self, distance_cm, greyscale, applied: MotorPowers, armed: bool) -> None:
        range_m = range_m_from_cm(distance_cm, MIN_RANGE_M, MAX_RANGE_M)
        msg = SensorFrameMsg()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = FRAME_ID
        msg.range_m = float(range_m)
        msg.range_valid = distance_cm_from_range_m(range_m) is not None
        msg.servo_angle_deg = 0.0
        msg.greyscale = [max(0, min(MAX_ADC, int(value))) for value in greyscale]
        msg.last_motor_power = list(applied)
        msg.armed = armed
        self._frame_pub.publish(msg)

    def _motor_loop(self) -> None:
        next_at = time.monotonic()
        while not self._stopping.is_set():
            try:
                self._drive_once()
            except Exception as error:
                self.get_logger().error(f"motor write failed: {error!r}", throttle_duration_sec=1.0)
                self._try_stop()
            next_at = self._sleep_until(next_at + self._motor_tick_s)
        self._try_stop()

    def _drive_once(self) -> None:
        with self._state_lock:
            powers = self._governor.output(time.monotonic())
            if powers == self._applied:
                return
        started = time.monotonic()
        with self._i2c_lock:
            if powers == ZERO_POWER:
                self._hardware.stop()
            else:
                self._hardware.set_motor_power(powers)
        elapsed = time.monotonic() - started
        if elapsed > self._slow_write_s:
            # The library retries a failed I2C write after a 1 s HAT reset.
            self.get_logger().warning(f"motor write took {elapsed * 1000:.0f} ms")
        with self._state_lock:
            self._applied = powers

    def _try_stop(self) -> None:
        try:
            with self._i2c_lock:
                self._hardware.stop()
            with self._state_lock:
                self._applied = ZERO_POWER
        except Exception as error:
            self.get_logger().error(f"stopping the motors failed: {error!r}")

    def _sleep_until(self, deadline: float) -> float:
        """Sleep to ``deadline``; if already late, resync to now instead of bursting."""
        now = time.monotonic()
        if deadline > now:
            self._stopping.wait(deadline - now)
            return deadline
        return now

    # Lifecycle

    def destroy_node(self) -> bool:
        self._stopping.set()
        for thread in self._threads:
            thread.join(timeout=2.0)
        self._try_stop()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ControllerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()  # SIGINT has usually shut the context down already
