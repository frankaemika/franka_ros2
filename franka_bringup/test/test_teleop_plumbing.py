#  Copyright (c) 2026 Franka Robotics GmbH
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.

"""Teleop plumbing test for the joystick to cmd_vel pipeline."""

import time
import unittest

from geometry_msgs.msg import Twist

from launch import LaunchDescription
from launch.actions import TimerAction

from launch_ros.actions import Node

import launch_testing.actions

import rclpy

from sensor_msgs.msg import Joy


class TestTeleopPlumbing(unittest.TestCase):
    """Verify joystick input is translated into a forward cmd_vel command."""

    @classmethod
    def setUpClass(cls):
        """Initialize the ROS context."""
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """Shutdown the ROS context."""
        rclpy.shutdown()

    def setUp(self):
        """Create a ROS node for the teleop plumbing test."""
        self.node = rclpy.create_node('teleop_plumbing_test')

    def tearDown(self):
        """Destroy the ROS node."""
        self.node.destroy_node()

    def test_joy_message_produces_cmd_vel(self):
        """Verify forward joystick command produces positive velocity."""
        received_messages = []

        def twist_callback(msg):
            received_messages.append(msg)

        joy_publisher = self.node.create_publisher(Joy, '/joy', 10)
        cmd_vel_subscription = self.node.create_subscription(
            Twist, '/cmd_vel', twist_callback, 10
        )

        try:
            deadline = time.monotonic() + 5.0
            while (time.monotonic() < deadline
                   and joy_publisher.get_subscription_count() == 0):
                rclpy.spin_once(self.node, timeout_sec=0.1)

            while time.monotonic() < deadline and not received_messages:
                joy_publisher.publish(
                    Joy(
                        axes=[0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
                        buttons=[1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                    )
                )
                rclpy.spin_once(self.node, timeout_sec=0.1)

            self.assertTrue(
                received_messages,
                'No /cmd_vel message received within 5s',
            )
            self.assertGreater(
                received_messages[-1].linear.x,
                0.0,
                'Expected a positive forward cmd_vel command',
            )
        finally:
            self.node.destroy_subscription(cmd_vel_subscription)


def generate_test_description():
    """Create the joy + teleop_twist_joy launch for plumbing tests."""
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
    )
    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        parameters=[
            {
                'axis_linear.x': 1,
                'scale_linear.x': 0.5,
                'enable_button': 0,
            }
        ],
        output='screen',
    )

    return (
        LaunchDescription(
            [
                joy_node,
                teleop_node,
                TimerAction(
                    period=1.0,
                    actions=[launch_testing.actions.ReadyToTest()],
                ),
            ]
        ),
        {},
    )
