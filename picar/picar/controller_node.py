"""Controller: the one node that owns the robot hardware.

It reads the sensors every sensor_period_ms, applies the latest /cmd_vel to the motors every
motor_tick_ms, and owns every safety limit (see `picar.governor.MotorGovernor`). Nothing
else may touch the hardware: the picar-4wd library resets the HAT and claims the GPIO lines when
it is imported, so a second owner breaks the first.

The ultrasonic read blocks for up to ~20 ms, so the sensor timer, the motor timer and the command
callback sit in separate callback groups; run under a MultiThreadedExecutor, they execute in
parallel. The ultrasonic read is plain GPIO and runs unlocked. The greyscale ADC and the motors
share the HAT's I2C address, so those calls take ``_i2c_lock`` and never interleave.
"""

from __future__ import annotations

import dataclasses
import math
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState, Range
from std_msgs.msg import UInt16MultiArray

from picar.governor import GovernorConfig, MotorGovernor
from picar.hardware import ZERO_POWER, MotorPowers, RobotHardware, make_hardware
from picar.interface import (
    CMD_VEL_TOPIC,
    GREYSCALE_TOPIC,
    JOINT_STATES_TOPIC,
    MOTOR_JOINTS,
    RANGE_TOPIC,
    SERVO_JOINT,
)

MIN_RANGE_M = 0.02
MAX_RANGE_M = 4.0
FIELD_OF_VIEW_RAD = 0.26  # about 15 degrees, the HC-SR04 beam
FRAME_ID = "ultrasonic_link"


def _range_value(range_m: float | None) -> float:
    """The value to publish in a Range message: NaN if invalid, -inf if too close, +inf if clear."""
    if range_m is None or math.isnan(range_m):
        return math.nan
    if range_m < MIN_RANGE_M:
        return -math.inf
    if range_m > MAX_RANGE_M:
        return math.inf
    return range_m


class ControllerNode(Node):
    def __init__(self, hardware: RobotHardware | None = None) -> None:
        super().__init__("controller")
        self.declare_parameter("hardware", "fake")
        self.declare_parameter("fake_ranges_m", [1.0])
        self.declare_parameter("sim_walls_m", [1.0, 0.6, 1.5])
        self.declare_parameter("sim_speed_mps", 0.3)
        self.declare_parameter("sensor_period_ms", 30.0)
        self.declare_parameter("motor_tick_ms", 10.0)
        self.declare_parameter("slow_write_ms", 100.0)
        governor_fields = dataclasses.fields(GovernorConfig)
        for field in governor_fields:
            self.declare_parameter(field.name, field.default)

        self._hardware = hardware or make_hardware(
            self.get_parameter("hardware").value,
            fake_ranges_m=self.get_parameter("fake_ranges_m").value,
            sim_walls_m=self.get_parameter("sim_walls_m").value,
            sim_speed_mps=self.get_parameter("sim_speed_mps").value,
        )
        self._slow_write_s = self.get_parameter("slow_write_ms").value / 1000.0
        self._governor = MotorGovernor(
            GovernorConfig(**{f.name: self.get_parameter(f.name).value for f in governor_fields})
        )

        self._lock = threading.Lock()  # the governor and _applied
        self._i2c_lock = threading.Lock()  # greyscale reads and motor writes
        # What was last written; None until the first write, which makes the motor timer stop
        # the motors explicitly at startup.
        self._applied: MotorPowers | None = None

        self._hardware.set_servo_angle(0.0)  # the sensor stays locked forward in v0.2
        self._range_pub = self.create_publisher(Range, RANGE_TOPIC, 10)
        self._greyscale_pub = self.create_publisher(UInt16MultiArray, GREYSCALE_TOPIC, 10)
        self._joint_pub = self.create_publisher(JointState, JOINT_STATES_TOPIC, 10)

        sensors, motors, requests = (MutuallyExclusiveCallbackGroup() for _ in range(3))
        self.create_timer(
            self.get_parameter("sensor_period_ms").value / 1000.0, self._sense, sensors
        )
        self.create_timer(
            self.get_parameter("motor_tick_ms").value / 1000.0, self._drive, motors
        )
        self.create_subscription(Twist, CMD_VEL_TOPIC, self._on_cmd_vel, 10, callback_group=requests)

    def _on_cmd_vel(self, msg: Twist) -> None:
        with self._lock:
            self._governor.set_command(msg, time.monotonic())

    # Timers

    def _sense(self) -> None:
        try:
            range_m = self._hardware.read_range_m()
            with self._i2c_lock:
                greyscale = self._hardware.read_greyscale()
        except Exception as error:  # a failed read must not stop the node
            # No update reaches the governor, so forward drive is blocked once the range goes
            # stale.
            self.get_logger().error(f"sensor read failed: {error!r}", throttle_duration_sec=1.0)
            return
        with self._lock:
            self._governor.update_range(range_m, time.monotonic())
            applied = self._applied or ZERO_POWER
        stamp = self.get_clock().now().to_msg()

        range_msg = Range(
            radiation_type=Range.ULTRASOUND,
            field_of_view=FIELD_OF_VIEW_RAD,
            min_range=MIN_RANGE_M,
            max_range=MAX_RANGE_M,
            range=_range_value(range_m),
        )
        range_msg.header.stamp = stamp
        range_msg.header.frame_id = FRAME_ID
        self._range_pub.publish(range_msg)

        self._greyscale_pub.publish(UInt16MultiArray(data=[int(value) for value in greyscale]))

        joints = JointState(
            name=[SERVO_JOINT, *MOTOR_JOINTS],
            position=[0.0] + [math.nan] * len(MOTOR_JOINTS),
            effort=[math.nan, *map(float, applied)],
        )
        joints.header.stamp = stamp
        self._joint_pub.publish(joints)

    def _drive(self) -> None:
        try:
            with self._lock:
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
            with self._lock:
                self._applied = powers
        except Exception as error:
            self.get_logger().error(f"motor write failed: {error!r}", throttle_duration_sec=1.0)
            self._try_stop()

    def _try_stop(self) -> None:
        try:
            with self._i2c_lock:
                self._hardware.stop()
            with self._lock:
                self._applied = ZERO_POWER
        except Exception as error:
            self.get_logger().error(f"stopping the motors failed: {error!r}")

    def destroy_node(self) -> bool:
        self._try_stop()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ControllerNode()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()  # SIGINT has usually shut the context down already
