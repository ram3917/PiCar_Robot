"""End to end: picar.launch.py with fake hardware brings up both nodes, and arming drives.

Needs the packages built and sourced (ros/install/setup.bash), since it launches the installed
entry points.
"""

import os
import time
import unittest

import launch_testing
import launch_testing.asserts
import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from picar_interfaces.msg import MotionPlan, SensorFrame
from std_srvs.srv import SetBool


@pytest.mark.launch_test
def generate_test_description():
    launch_file = os.path.join(
        get_package_share_directory("picar_ros"), "launch", "picar.launch.py"
    )
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(launch_file),
                launch_arguments={"hardware": "fake"}.items(),
            ),
            launch_testing.actions.ReadyToTest(),
        ]
    )


class TestRunningStack(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("launch_probe")
        cls.frames, cls.plans = [], []
        cls.node.create_subscription(SensorFrame, "/controller/sensor_frame", cls.frames.append, 10)
        cls.node.create_subscription(MotionPlan, "/planner/motion_plan", cls.plans.append, 10)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _spin_until(self, predicate, timeout_s=10.0):
        end = time.monotonic() + timeout_s
        while time.monotonic() < end and not predicate():
            rclpy.spin_once(self.node, timeout_sec=0.05)
        return predicate()

    def test_1_both_nodes_come_up(self):
        self.assertTrue(
            self._spin_until(
                lambda: {"controller", "motion_planner"} <= set(self.node.get_node_names())
            ),
            f"nodes up: {self.node.get_node_names()}",
        )

    def test_2_controller_publishes_the_fake_range(self):
        self.assertTrue(self._spin_until(lambda: self.frames))
        self.assertAlmostEqual(self.frames[-1].range_m, 1.0, places=3)  # params.yaml: 100 cm

    def test_3_the_robot_boots_disarmed_and_nothing_is_planned(self):
        self.assertTrue(self._spin_until(lambda: self.frames))
        self.assertFalse(self.frames[-1].armed)
        self.assertEqual(self.plans, [])

    def test_4_arming_starts_cruising(self):
        client = self.node.create_client(SetBool, "/arm")
        self.assertTrue(client.wait_for_service(timeout_sec=10.0))
        future = client.call_async(SetBool.Request(data=True))
        self.assertTrue(self._spin_until(future.done))
        self.assertTrue(future.result().success, future.result().message)

        self.assertTrue(self._spin_until(lambda: self.plans))
        self.assertTrue(
            self._spin_until(lambda: self.frames and self.frames[-1].last_motor_power[0] > 0)
        )
        self.assertTrue(self.frames[-1].armed)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_nodes_exit_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
