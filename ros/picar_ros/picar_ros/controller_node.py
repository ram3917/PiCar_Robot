"""Controller: the one node that owns the robot hardware.

It reads the sensors every sensor_period_ms, plays the latest plan onto the motors every
motor_tick_ms, and owns arming and every safety limit (see `picar_core.MotorGovernor`). Nothing
else may touch the hardware: the picar-4wd library resets the HAT and claims the GPIO lines when
it is imported, so a second owner breaks the first.

The ultrasonic read blocks for up to ~20 ms, so the sensor timer, the motor timer and the plan and
arm callbacks sit in separate callback groups; run under a MultiThreadedExecutor, they execute in
parallel. The ultrasonic read is plain GPIO and runs unlocked. The greyscale ADC and the motors
share the HAT's I2C address, so those calls take ``_i2c_lock`` and never interleave.
"""

from __future__ import annotations

import dataclasses
import math
import threading
import time

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState, Range
from std_msgs.msg import Bool, UInt16MultiArray
from std_srvs.srv import SetBool
from trajectory_msgs.msg import JointTrajectory

from picar_core import (
    ZERO_POWER,
    GovernorConfig,
    MotionPlan,
    MotorGovernor,
    MotorPowers,
    RobotHardware,
    range_m_from_cm,
)
from picar_ros.hardware import make_hardware
from picar_ros.interface import (
    ARM_SERVICE,
    ARMED_QOS,
    ARMED_TOPIC,
    GREYSCALE_TOPIC,
    JOINT_STATES_TOPIC,
    MOTOR_JOINTS,
    PLAN_TOPIC,
    RANGE_TOPIC,
    SERVO_JOINT,
)

MIN_RANGE_M = 0.02
MAX_RANGE_M = 4.0
FIELD_OF_VIEW_RAD = 0.26  # about 15 degrees, the HC-SR04 beam
FRAME_ID = "ultrasonic_link"


class ControllerNode(Node):
    def __init__(self, hardware: RobotHardware | None = None) -> None:
        super().__init__("controller")
        self.declare_parameter("hardware", "fake")
        self.declare_parameter("fake_distances_cm", [100.0])
        self.declare_parameter("sensor_period_ms", 30.0)
        self.declare_parameter("motor_tick_ms", 10.0)
        self.declare_parameter("slow_write_ms", 100.0)
        governor_fields = dataclasses.fields(GovernorConfig)
        for field in governor_fields:
            self.declare_parameter(field.name, field.default)

        self._hardware = hardware or make_hardware(
            self.get_parameter("hardware").value,
            fake_distances_cm=self.get_parameter("fake_distances_cm").value,
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
        self._armed_pub = self.create_publisher(Bool, ARMED_TOPIC, ARMED_QOS)
        self._publish_armed()

        sensors, motors, requests = (MutuallyExclusiveCallbackGroup() for _ in range(3))
        self.create_timer(
            self.get_parameter("sensor_period_ms").value / 1000.0, self._sense, sensors
        )
        self.create_timer(
            self.get_parameter("motor_tick_ms").value / 1000.0, self._drive, motors
        )
        self.create_subscription(JointTrajectory, PLAN_TOPIC, self._on_plan, 10, callback_group=requests)
        self.create_service(SetBool, ARM_SERVICE, self._on_arm, callback_group=requests)

    # Requests

    def _on_arm(self, request: SetBool.Request, response: SetBool.Response) -> SetBool.Response:
        with self._lock:
            self._governor.set_armed(request.data)
        self._publish_armed()
        response.success = True
        response.message = "ARMED" if request.data else "DISARMED"
        self.get_logger().info(response.message)
        return response

    def _on_plan(self, msg: JointTrajectory) -> None:
        try:
            if tuple(msg.joint_names) != MOTOR_JOINTS:
                raise ValueError(f"joint_names {list(msg.joint_names)} are not {list(MOTOR_JOINTS)}")
            plan = MotionPlan(
                times_s=[Duration.from_msg(p.time_from_start).nanoseconds / 1e9 for p in msg.points],
                steps=[list(p.positions) for p in msg.points],
            )
        except ValueError as error:
            self.get_logger().warning(f"ignored a bad motion plan: {error}")
            return
        with self._lock:
            self._governor.set_plan(plan, time.monotonic())

    def _publish_armed(self) -> None:
        with self._lock:
            armed = self._governor.armed
        self._armed_pub.publish(Bool(data=armed))

    # Timers

    def _sense(self) -> None:
        try:
            distance_cm = self._hardware.read_distance_cm()
            with self._i2c_lock:
                greyscale = self._hardware.read_greyscale()
        except Exception as error:  # a failed read must not stop the node
            # No update reaches the governor, so forward drive is blocked once the range goes
            # stale.
            self.get_logger().error(f"sensor read failed: {error!r}", throttle_duration_sec=1.0)
            return
        with self._lock:
            self._governor.update_range(distance_cm, time.monotonic())
            applied = self._applied or ZERO_POWER
        stamp = self.get_clock().now().to_msg()

        range_msg = Range(
            radiation_type=Range.ULTRASOUND,
            field_of_view=FIELD_OF_VIEW_RAD,
            min_range=MIN_RANGE_M,
            max_range=MAX_RANGE_M,
            range=float(range_m_from_cm(distance_cm, MIN_RANGE_M, MAX_RANGE_M)),
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
