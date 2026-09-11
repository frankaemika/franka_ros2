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

"""Fake-hardware integration test for Mobile FR3 Duo configuration."""
# The swerve_drive_controller needs real TMR cartesian_pose_state interfaces,
# so it cannot activate with mock_components. This test verifies hardware
# activation, joint_state_broadcaster, and arm joint presence.

import unittest

from franka_bringup.testing.fake_hardware_test_base import (
    collect_unexpected_error_lines,
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


CONTROLLER_NAME = 'mobile_fr3_duo_joint_impedance_example_controller'

EXPECTED_ARM_JOINTS = {
    f'{arm_prefix}_fr3v2_joint{joint_index}'
    for arm_prefix in ('left', 'right')
    for joint_index in range(1, 8)
}


class TestFakeHardwareMobileFr3Duo(FakeHardwareTestBase):
    """Verify Mobile FR3 Duo stack launches with fake hardware."""

    NODE_NAME = 'fake_hardware_mobile_fr3_duo_test'

    def test_mobile_duo_controller_activates(self):
        """Verify hardware activates and both arms publish joints."""
        # swerve_drive_controller cannot activate (needs real TMR interfaces).
        # We verify hardware activation, broadcaster, and joint presence.
        self.wait_for_stack_ready()

        joint_state = self.wait_for_joint_states(min_joints=14)
        self.assertTrue(
            EXPECTED_ARM_JOINTS.issubset(set(joint_state.name)),
            f'Expected arm joints not found. Got: {joint_state.name}',
        )


@launch_testing.post_shutdown_test()
class TestFakeHardwareMobileFr3DuoShutdown(unittest.TestCase):
    """Verify the Mobile FR3 Duo launch exits without unexpected errors."""

    # swerve_drive_controller cannot activate with mock_components (needs real
    # TMR cartesian_pose_state interfaces); its activation error is expected.
    IGNORED_PATTERNS = DEFAULT_SHUTDOWN_IGNORE_PATTERNS + [
        'swerve_drive_controller',
        'cartesian_pose_state',
    ]

    def test_has_no_error(self, proc_output):
        """Check no unexpected [ERROR] messages appear across the full run."""
        error_lines = collect_unexpected_error_lines(
            proc_output, self.IGNORED_PATTERNS
        )
        self.assertEqual(
            error_lines, [],
            f'Found unexpected [ERROR] log messages: {error_lines}',
        )


def generate_test_description():
    """Create the Mobile FR3 Duo fake-hardware launch for tests."""
    mobile_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare('franka_bringup'),
                        'launch',
                        'mobile_fr3_duo.launch.py',
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
                    'test_fake_hardware_mobile_fr3_duo.config.yaml',
                ]
            ),
            'controller_name': CONTROLLER_NAME,
        }.items(),
    )

    return (
        LaunchDescription(
            [
                mobile_launch,
                launch_testing.actions.ReadyToTest(),
            ]
        ),
        {},
    )
