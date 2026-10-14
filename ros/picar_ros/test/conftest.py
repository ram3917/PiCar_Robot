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
from geometry_msgs.msg import Twist
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import String
from std_srvs.srv import SetBool

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

    def record(self, msg_type, topic: str) -> list:
        """Subscribe the probe to ``topic``; received messages are appended to the returned list."""
        messages = self.received.setdefault(topic, [])
        self.probe.create_subscription(msg_type, topic, messages.append, 10)
        return messages

    def publisher(self, msg_type, topic: str):
        return self.probe.create_publisher(msg_type, topic, 10)

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
    """Publishes /ultrasonic/range from the probe at a steady rate; ``distance_m`` can change."""

    def __init__(self, harness: RosHarness, distance_m: float, rate_hz: float = 20.0) -> None:
        self.distance_m = distance_m
        self._pub = harness.publisher(Range, "/ultrasonic/range")
        self._timer = harness.probe.create_timer(1.0 / rate_hz, self._publish)

    def _publish(self) -> None:
        msg = Range()
        msg.radiation_type = Range.ULTRASOUND
        msg.min_range = 0.02
        msg.max_range = 4.0
        msg.range = self.distance_m
        self._pub.publish(msg)

    def stop(self) -> None:
        self._timer.cancel()


def call_arm(harness: RosHarness, armed: bool) -> SetBool.Response:
    client = harness.probe.create_client(SetBool, "/arm")
    assert client.wait_for_service(timeout_sec=3.0), "/arm service not available"
    future = client.call_async(SetBool.Request(data=armed))
    assert harness.spin_until(future.done), "/arm call timed out"
    return future.result()


def twist(linear: float = 0.0, angular: float = 0.0) -> Twist:
    msg = Twist()
    msg.linear.x = linear
    msg.angular.z = angular
    return msg


def states(messages: list[String]) -> list[str]:
    return [msg.data for msg in messages]
