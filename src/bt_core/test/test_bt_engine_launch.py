#!/usr/bin/env python3
"""
Launch Test — bt_engine node
================================
Verifies that the bt_engine node:
  1. Starts without crashing (process stays alive for 5s)
  2. Advertises its expected topics and services

Note: bt_engine will sit in a spin loop waiting for a plan_file message,
so it won't fully initialise the behaviour tree — but it should NOT crash.
"""

import os
import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import launch_testing.markers
import pytest

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    """Launch the bt_engine node with default config for testing."""

    # Locate config files (relative to this test file)
    test_dir = os.path.dirname(os.path.abspath(__file__))
    pkg_dir = os.path.dirname(test_dir)   # src/bt_core
    src_dir = os.path.dirname(pkg_dir)    # src/

    map_points_config = os.path.join(pkg_dir, 'params', 'map_points_default.yaml')
    robot_config = os.path.join(src_dir, 'startup', 'params', 'robot_config_default.yaml')

    # Override pkg_share_dir so bt_engine finds BT XMLs at the correct install path
    # (robot_config_default.yaml hardcodes a path for the real robot)
    bt_core_share_dir = get_package_share_directory('bt_core')

    bt_engine_node = launch_ros.actions.Node(
        package='bt_core',
        executable='bt_engine',
        name='bt_engine',
        output='screen',
        parameters=[
            robot_config,
            map_points_config,
            {'frame_id': 'base_footprint'},
            {'tree_name': 'MainTree'},
            {'pkg_share_dir': bt_core_share_dir},
        ],
    )

    return (
        launch.LaunchDescription([
            bt_engine_node,
            launch_testing.actions.ReadyToTest(),
        ]),
        {'bt_engine_node': bt_engine_node},
    )


class TestBTEngineAlive(unittest.TestCase):
    """Check the process is still alive after a short delay."""

    def test_process_starts_and_stays_alive(self, proc_info, bt_engine_node):
        """bt_engine must not crash within the first 5 seconds."""
        proc_info.assertWaitForStartup(process=bt_engine_node, timeout=10)
        time.sleep(5)


class TestBTEngineTopics(unittest.TestCase):
    """After the node is running, verify expected topics/services exist."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = Node('test_bt_engine_topics')

    def tearDown(self):
        self.node.destroy_node()

    def test_start_signal_service_exists(self):
        """/robot/start_signal service must be advertised by bt_engine."""
        assert self._wait_for_service('/robot/start_signal'), \
            "Service /robot/start_signal not found"

    def test_subscribes_to_are_you_ready(self):
        """bt_engine must subscribe to /robot/startup/are_you_ready."""
        assert self._wait_for_topic('/robot/startup/are_you_ready'), \
            "Topic /robot/startup/are_you_ready not found"

    def test_subscribes_to_plan_file(self):
        """bt_engine must subscribe to /robot/startup/plan_file."""
        assert self._wait_for_topic('/robot/startup/plan_file'), \
            "Topic /robot/startup/plan_file not found"

    # ── helpers ─────────────────────────────────────────────────
    def _wait_for_topic(self, topic_name, timeout_sec=10.0):
        end = time.time() + timeout_sec
        while time.time() < end:
            topics = self.node.get_topic_names_and_types()
            if any(t[0] == topic_name for t in topics):
                return True
            rclpy.spin_once(self.node, timeout_sec=0.5)
        return False

    def _wait_for_service(self, service_name, timeout_sec=10.0):
        end = time.time() + timeout_sec
        while time.time() < end:
            services = self.node.get_service_names_and_types()
            if any(s[0] == service_name for s in services):
                return True
            rclpy.spin_once(self.node, timeout_sec=0.5)
        return False


@launch_testing.post_shutdown_test()
class TestBTEngineExit(unittest.TestCase):
    """Verify clean shutdown."""

    def test_exit_code(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
