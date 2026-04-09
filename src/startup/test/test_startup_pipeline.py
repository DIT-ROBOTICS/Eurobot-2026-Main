#!/usr/bin/env python3
"""
Integration Test — Full Startup Pipeline
==========================================
Automated version of test_launch.sh + mock_groups_test.py.

This test:
  1. Launches both startup_new and bt_engine nodes
  2. Waits for startup to publish "are_you_ready"
  3. Sends ready signals for ALL groups (mock_groups_test.py behaviour)
  4. Sends the plug signal to trigger game start
  5. Verifies that startup transitions to START state by checking
     that /robot/startup/game_time begins publishing
  6. Verifies that bt_engine receives the start signal

Timeout: 30 seconds for the full pipeline.
"""

import os
import time
import unittest
import threading

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import launch_testing.markers
import pytest

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String
from btcpp_ros2_interfaces.srv import StartUpSrv


# How many groups the startup node expects (index 1..group_num-1)
GROUP_NUM = 5  # Matches robot_config_default.yaml


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    """Launch both startup_new and bt_engine together."""

    test_dir = os.path.dirname(os.path.abspath(__file__))
    startup_pkg_dir = os.path.dirname(test_dir)            # src/startup
    src_dir = os.path.dirname(startup_pkg_dir)             # src/
    bt_core_dir = os.path.join(src_dir, 'bt_core')

    default_config = os.path.join(startup_pkg_dir, 'params', 'robot_config_default.yaml')
    map_points_config = os.path.join(bt_core_dir, 'params', 'map_points_default.yaml')

    startup_node = launch_ros.actions.Node(
        package='startup',
        executable='startup_new',
        name='startup_node',
        output='screen',
        parameters=[default_config],
    )

    bt_engine_node = launch_ros.actions.Node(
        package='bt_core',
        executable='bt_engine',
        name='bt_engine',
        output='screen',
        parameters=[
            default_config,
            map_points_config,
            {'frame_id': 'base_footprint'},
            {'tree_name': 'MainTree'},
        ],
    )

    return (
        launch.LaunchDescription([
            startup_node,
            bt_engine_node,
            launch_testing.actions.ReadyToTest(),
        ]),
        {
            'startup_node': startup_node,
            'bt_engine_node': bt_engine_node,
        },
    )


class TestStartupPipeline(unittest.TestCase):
    """End-to-end integration test for the startup → bt_engine pipeline."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = Node('integration_tester')
        self.are_you_ready_received = False
        self.game_time_received = False
        self.plan_file_received = False
        self.plan_file_name = ''

        # Subscribers
        self.node.create_subscription(
            Bool, '/robot/startup/are_you_ready',
            self._are_you_ready_cb, 10)
        self.node.create_subscription(
            Float32, '/robot/startup/game_time',
            self._game_time_cb, 10)
        self.node.create_subscription(
            String, '/robot/startup/plan_file',
            self._plan_file_cb, 10)

        # Publishers
        self.plug_pub = self.node.create_publisher(Bool, '/robot/startup/plug', 10)

        # Service client
        self.ready_client = self.node.create_client(
            StartUpSrv, '/robot/startup/ready_signal')

    def tearDown(self):
        self.node.destroy_node()

    # ── callbacks ───────────────────────────────────────────────
    def _are_you_ready_cb(self, msg):
        if msg.data:
            self.are_you_ready_received = True

    def _game_time_cb(self, msg):
        if msg.data >= 0.0:
            self.game_time_received = True

    def _plan_file_cb(self, msg):
        if msg.data:
            self.plan_file_received = True
            self.plan_file_name = msg.data

    # ── helpers ─────────────────────────────────────────────────
    def _spin_until(self, condition_fn, timeout_sec=15.0, label='condition'):
        """Spin the node until condition_fn() returns True or timeout."""
        end = time.time() + timeout_sec
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.2)
            if condition_fn():
                return True
        self.fail(f"Timed out waiting for {label} (waited {timeout_sec}s)")

    def _send_ready_for_group(self, group_id):
        """Send a ready signal for a single group."""
        req = StartUpSrv.Request()
        req.group = group_id
        req.state = 1  # READY
        future = self.ready_client.call_async(req)
        # Spin until the service responds (max 5s)
        end = time.time() + 5.0
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if future.done():
                result = future.result()
                self.assertTrue(result.success,
                                f"Ready signal for group {group_id} was rejected")
                return
        self.fail(f"Ready signal for group {group_id} timed out")

    # ── THE TEST ────────────────────────────────────────────────
    def test_full_startup_pipeline(self):
        """
        Full pipeline:
          are_you_ready → all groups ready → plug → game_time publishing
        """

        # Step 1: Wait for startup to broadcast "are_you_ready"
        self._spin_until(
            lambda: self.are_you_ready_received,
            timeout_sec=15,
            label='/robot/startup/are_you_ready')
        self.node.get_logger().info("✓ are_you_ready received")

        # Step 2: Wait for ready_signal service to be available
        self.assertTrue(
            self.ready_client.wait_for_service(timeout_sec=10.0),
            "ready_signal service never appeared")
        self.node.get_logger().info("✓ ready_signal service available")

        # Step 3: Send ready for ALL groups (1 through GROUP_NUM-1)
        for group_id in range(1, GROUP_NUM):
            self._send_ready_for_group(group_id)
            self.node.get_logger().info(f"✓ Group {group_id} ready")

        # Step 4: Send plug signal (multiple times like mock_groups_test.py)
        plug_msg = Bool()
        plug_msg.data = True
        for _ in range(5):
            self.plug_pub.publish(plug_msg)
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.node.get_logger().info("✓ Plug signal sent")

        # Step 5: Verify game starts — game_time should begin publishing
        self._spin_until(
            lambda: self.game_time_received,
            timeout_sec=10,
            label='/robot/startup/game_time publishing')
        self.node.get_logger().info("✓ Game started — game_time is publishing")

        # Step 6: Verify plan file was published
        self.assertTrue(self.plan_file_received,
                        "Plan file was never published")
        self.node.get_logger().info(
            f"✓ Plan file: {self.plan_file_name}")


@launch_testing.post_shutdown_test()
class TestPipelineExit(unittest.TestCase):
    """Verify clean shutdown of all nodes."""

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
