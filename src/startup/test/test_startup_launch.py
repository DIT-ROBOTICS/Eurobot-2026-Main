#!/usr/bin/env python3
"""
Launch Test — startup_new node
================================
Verifies that the startup_new node:
  1. Starts without crashing (process stays alive for 5s)
  2. Advertises its expected topics and services
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


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    """Launch the startup_new node with default config for testing."""

    # Locate the default robot config (relative to this test file)
    test_dir = os.path.dirname(os.path.abspath(__file__))
    pkg_dir = os.path.dirname(test_dir)  # src/startup
    default_config = os.path.join(pkg_dir, 'params', 'robot_config_default.yaml')

    startup_node = launch_ros.actions.Node(
        package='startup',
        executable='startup_new',
        name='startup_node',
        output='screen',
        parameters=[default_config],
    )

    return (
        launch.LaunchDescription([
            startup_node,
            launch_testing.actions.ReadyToTest(),
        ]),
        {'startup_node': startup_node},
    )


class TestStartupNodeAlive(unittest.TestCase):
    """Check the process is still alive after a short delay."""

    def test_process_starts_and_stays_alive(self, proc_info, startup_node):
        """startup_new must not crash within the first 5 seconds."""
        proc_info.assertWaitForStartup(process=startup_node, timeout=10)
        # Give it a few seconds to run its INIT → READY transition
        time.sleep(5)


class TestStartupTopics(unittest.TestCase):
    """After the node is running, verify expected topics/services exist."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = Node('test_startup_topics')

    def tearDown(self):
        self.node.destroy_node()

    def test_are_you_ready_topic_exists(self):
        """startup_node must publish /robot/startup/are_you_ready."""
        assert self._wait_for_topic('/robot/startup/are_you_ready'), \
            "Topic /robot/startup/are_you_ready not found"

    def test_game_time_topic_exists(self):
        """startup_node must publish /robot/startup/game_time."""
        assert self._wait_for_topic('/robot/startup/game_time'), \
            "Topic /robot/startup/game_time not found"

    def test_plan_file_topic_exists(self):
        """startup_node must publish /robot/startup/plan_file."""
        assert self._wait_for_topic('/robot/startup/plan_file'), \
            "Topic /robot/startup/plan_file not found"

    def test_ready_signal_service_exists(self):
        """startup_node must advertise /robot/startup/ready_signal service."""
        assert self._wait_for_service('/robot/startup/ready_signal'), \
            "Service /robot/startup/ready_signal not found"

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
class TestStartupNodeExit(unittest.TestCase):
    """Verify clean shutdown."""

    def test_exit_code(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
