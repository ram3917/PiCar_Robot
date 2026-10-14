"""Shared fixtures for the picar_ros tests.

These tests need ROS 2 (rclpy, launch_testing) and run in the dev container with the ROS
Python, not the uv venv. See scripts/test_ros.sh. The pure-Python core tests live in core/tests.
"""

import math
import os
import time
from typing import Callable

import pytest
import rclpy
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from picar_ros.interface import ARMED_QOS, ARMED_TOPIC, MOTOR_JOINTS, RANGE_TOPIC

# Keep test traffic off the network and away from a robot stack running on the default domain.
# Set before any rclpy.init, and inherited by the processes the launch test starts.
os.environ["ROS_DOMAIN_ID"] = os.environ.get("PICAR_TEST_DOMAIN_ID", "87")
os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"


def _ros_args(params: dict) -> list[str]:
    args = ["--ros-args"]
    for name, value in params.items():
        args += ["-p", f"{name}:={_yaml(value)}"]
    return args


def _yaml(value) -> str:
    if isinstance(value, (list, tuple)):
        return "[" + ", ".join(_yaml(v) for v in value) + "]"
    if isinstance(value, float) and math.isnan(value):
        return ".nan"
    return str(value)


class RosHarness:
    """Runs one node under test next to a probe node that plays the rest of the graph.

    The probe publishes and subscribes on the node's topics and records what it receives.
    """

    def __init__(self) -> None:
        self.context: rclpy.context.Context | None = None
        self.executor: SingleThreadedExecutor | None = None
        self.node: Node | None = None
        self.probe: Node | None = None
        self.received: dict[str, list] = {}

    def start(self, node_cls: Callable[[], Node], **params) -> Node:
        """Create the node under test, with ``params`` as parameter overrides."""
        # Overrides go in as global --ros-args, since the nodes take no constructor arguments.
        rclpy.init(args=_ros_args(params))
        self.context = rclpy.get_default_context()
        self.node = node_cls()
        self.probe = rclpy.create_node("probe")
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.executor.add_node(self.probe)
        return self.node

    def record(self, msg_type, topic: str, qos=10) -> list:
        """Subscribe the probe to ``topic``; received messages are appended to the returned list."""
        messages = self.received.setdefault(topic, [])
        self.probe.create_subscription(msg_type, topic, messages.append, qos)
        return messages

    def publisher(self, msg_type, topic: str, qos=10):
        return self.probe.create_publisher(msg_type, topic, qos)

    def spin_for(self, seconds: float) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.executor.spin_once(timeout_sec=0.01)

    def spin_until(self, predicate: Callable[[], bool], timeout_s: float = 3.0) -> bool:
        end = time.monotonic() + timeout_s
        while time.monotonic() < end:
            if predicate():
                return True
            self.executor.spin_once(timeout_sec=0.01)
        return predicate()

    def close(self) -> None:
        if self.executor is not None:
            self.executor.shutdown()
        for node in (self.node, self.probe):
            if node is not None:
                node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


@pytest.fixture
def ros():
    harness = RosHarness()
    yield harness
    harness.close()


class RangeFeed:
    """Publishes the controller's range and armed topics from the probe at a steady rate.

    ``range_m`` and ``armed`` can change while it runs.
    """

    def __init__(
        self, harness: RosHarness, range_m: float, armed: bool = True, rate_hz: float = 33.0
    ) -> None:
        self.range_m = range_m
        self.armed = armed
        self._range_pub = harness.publisher(Range, RANGE_TOPIC)
        self._armed_pub = harness.publisher(Bool, ARMED_TOPIC, ARMED_QOS)
        self._timer = harness.probe.create_timer(1.0 / rate_hz, self._publish)

    def _publish(self) -> None:
        self._range_pub.publish(Range(range=float(self.range_m)))
        self._armed_pub.publish(Bool(data=self.armed))

    def stop(self) -> None:
        self._timer.cancel()


def plan_msg(*steps: tuple[float, float, float, float], step_s: float = 0.2) -> JointTrajectory:
    """A plan whose point ``i`` reaches ``steps[i]`` at ``(i + 1) * step_s`` seconds."""
    return JointTrajectory(
        joint_names=list(MOTOR_JOINTS),
        points=[
            JointTrajectoryPoint(
                positions=[float(p) for p in powers],
                time_from_start=Duration(seconds=(i + 1) * step_s).to_msg(),
            )
            for i, powers in enumerate(steps)
        ],
    )


def call_arm(harness: RosHarness, armed: bool) -> SetBool.Response:
    client = harness.probe.create_client(SetBool, "/arm")
    assert client.wait_for_service(timeout_sec=3.0), "/arm service not available"
    future = client.call_async(SetBool.Request(data=armed))
    assert harness.spin_until(future.done), "/arm call timed out"
    return future.result()


