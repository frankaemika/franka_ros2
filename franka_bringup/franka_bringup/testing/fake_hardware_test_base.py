# Copyright 2026 Franka Robotics GmbH
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Base class and utilities for fake-hardware integration tests."""
# Provides FakeHardwareTestBase: handles ROS lifecycle, node management,
# ControllerServiceClient, joint_state waiting, and error assertion.
# See franka_bringup/test/README.md for full architecture documentation.

import time
import unittest

from franka_bringup.testing.controller_service_client import ControllerServiceClient

import rclpy
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import JointState


JOINT_STATE_BROADCASTER = 'joint_state_broadcaster'

# Default timeouts (tuned for CI environments)
SERVICE_DISCOVERY_TIMEOUT = 20.0
CONTROLLER_STATE_TIMEOUT = 30.0
JOINT_STATE_TIMEOUT = 10.0

# Errors that are expected on a clean launch_testing shutdown (nodes tearing
# down while peers are already gone) and are not indicative of a test failure:
#   - "service call timed out": a peer node is already gone.
#   - "pal_statistics": controller_manager's statistics publisher thread races
#     the rclpy context teardown ("context cannot be slept with ... invalid").
DEFAULT_SHUTDOWN_IGNORE_PATTERNS = [
    'service call timed out',
    'pal_statistics',
]


def collect_unexpected_error_lines(proc_output, ignore_patterns=None):
    """Return the [ERROR] log lines in proc_output not matching ignore_patterns."""
    # Intended for @launch_testing.post_shutdown_test() classes so the complete
    # process output (including the shutdown phase) is inspected, rather than the
    # partial output an active test method sees. Ignore matching is case-insensitive.
    ignore_patterns = [p.lower() for p in (ignore_patterns or [])]
    error_lines = []
    for event in proc_output:
        text = getattr(event, 'text', None)
        if text is None:
            text = str(event)
        elif isinstance(text, bytes):
            text = text.decode('utf-8', errors='replace')
        for line in text.splitlines():
            if '[ERROR]' not in line:
                continue
            if any(pattern in line.lower() for pattern in ignore_patterns):
                continue
            error_lines.append(line)
    return error_lines


class FakeHardwareTestBase(unittest.TestCase):
    """Base class for fake-hardware integration tests."""

    # Subclasses should override NODE_NAME for unique ROS node naming.

    NODE_NAME = 'fake_hardware_test'

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node(self.NODE_NAME)
        self.controller_client = ControllerServiceClient(self.node)

    def tearDown(self):
        self.controller_client.destroy()
        self.node.destroy_node()

    def wait_for_stack_ready(self):
        """Wait for controller_manager services and joint_state_broadcaster."""
        self.assertTrue(
            self.controller_client.wait_for_services(
                timeout_sec=SERVICE_DISCOVERY_TIMEOUT
            ),
            'controller_manager services did not become available',
        )
        self.assertTrue(
            self.controller_client.wait_for_controller_state(
                JOINT_STATE_BROADCASTER, ['active'],
                timeout_sec=CONTROLLER_STATE_TIMEOUT,
            ),
            'joint_state_broadcaster did not become active',
        )

    def wait_for_joint_states(
        self,
        *,
        min_joints=1,
        timeout_sec=JOINT_STATE_TIMEOUT,
        topic='/joint_states',
    ):
        """Wait for a JointState topic with at least `min_joints` joints published."""
        received = []

        def callback(msg):
            received.append(msg)

        sub = self.node.create_subscription(
            JointState, topic, callback, qos_profile_sensor_data,
        )

        try:
            deadline = time.monotonic() + timeout_sec
            while time.monotonic() < deadline:
                rclpy.spin_once(self.node, timeout_sec=0.2)
                if received and len(received[-1].name) >= min_joints:
                    return received[-1]
        finally:
            self.node.destroy_subscription(sub)

        self.fail(
            f'Did not receive {topic} with >= {min_joints} joints '
            f'within {timeout_sec}s'
        )

    def assert_no_errors_in_output(self, proc_output, *, ignore_patterns=None):
        """Assert no unexpected [ERROR] messages appear in launch output."""
        # Prefer a @launch_testing.post_shutdown_test() class (see
        # collect_unexpected_error_lines) so the full output is inspected; an
        # active test method only sees output captured so far.
        error_lines = collect_unexpected_error_lines(proc_output, ignore_patterns)
        self.assertEqual(
            error_lines, [],
            f'Found unexpected [ERROR] log messages: {error_lines}',
        )
