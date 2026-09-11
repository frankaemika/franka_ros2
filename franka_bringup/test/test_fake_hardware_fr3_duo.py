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

"""Fake-hardware integration test for FR3 Duo (dual-arm) configuration."""

import unittest

from franka_bringup.testing.fake_hardware_test_base import (
    collect_unexpected_error_lines,
    CONTROLLER_STATE_TIMEOUT,
    DEFAULT_SHUTDOWN_IGNORE_PATTERNS,
    FakeHardwareTestBase,
)

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare

import launch_testing
import launch_testing.actions


CONTROLLER_NAME = 'fr3_duo_joint_impedance_example_controller'
EXPECTED_JOINT_NAMES = {
    f'{arm_prefix}_fr3v2_joint{joint_index}'
    for arm_prefix in ('left', 'right')
    for joint_index in range(1, 8)
}


class TestFakeHardwareFr3Duo(FakeHardwareTestBase):
    """Verify FR3 Duo stack activates with fake hardware."""

    NODE_NAME = 'fake_hardware_fr3_duo_test'

    def test_duo_controller_activates(self):
        """Verify dual-arm controller activates and both arms publish."""
        self.wait_for_stack_ready()
        self.assertTrue(
            self.controller_client.wait_for_controller_state(
                CONTROLLER_NAME, ['active'],
                timeout_sec=CONTROLLER_STATE_TIMEOUT,
            ),
            f'{CONTROLLER_NAME} did not reach active state',
        )

        joint_state = self.wait_for_joint_states(min_joints=14)
        self.assertTrue(
            EXPECTED_JOINT_NAMES.issubset(set(joint_state.name))
        )


@launch_testing.post_shutdown_test()
class TestFakeHardwareFr3DuoShutdown(unittest.TestCase):
    """Verify the FR3 Duo launch exits without unexpected errors."""

    def test_has_no_error(self, proc_output):
        """Check no unexpected [ERROR] messages appear across the full run."""
        error_lines = collect_unexpected_error_lines(
            proc_output, DEFAULT_SHUTDOWN_IGNORE_PATTERNS
        )
        self.assertEqual(
            error_lines, [],
            f'Found unexpected [ERROR] log messages: {error_lines}',
        )


def generate_test_description():
    """Create the FR3 Duo fake-hardware launch for integration tests."""
    fr3_duo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare('franka_bringup'),
                        'launch',
                        'fr3_duo.launch.py',
                    ]
                )
            ]
        ),
        launch_arguments={
            'robot_config_file': PathJoinSubstitution(
                [
                    FindPackageShare('franka_bringup'),
                    'test',
                    'config',
                    'test_fake_hardware_fr3_duo.config.yaml',
                ]
            ),
            'controller_name': CONTROLLER_NAME,
        }.items(),
    )

    return (
        LaunchDescription(
            [
                fr3_duo_launch,
                launch_testing.actions.ReadyToTest(),
            ]
        ),
        {},
    )
