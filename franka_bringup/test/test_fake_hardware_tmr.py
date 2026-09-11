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

"""Fake-hardware integration test for the TMR v0.2 mobile base."""

from pathlib import Path
import unittest

from franka_bringup.testing.fake_hardware_test_base import (
    collect_unexpected_error_lines,
    DEFAULT_SHUTDOWN_IGNORE_PATTERNS,
    FakeHardwareTestBase,
)

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare

import launch_testing
import launch_testing.actions

STARTUP_TIMEOUT = 10.0
JOINT_STATE_TIMEOUT = 15.0
FRANKA_JOINT_STATES_TOPIC = '/franka/joint_states'

# TMR drive joints (4 joints: front/rear steering + front/rear driving)
EXPECTED_TMR_JOINTS = {
    'tmrv0_2_joint_0',
    'tmrv0_2_joint_1',
    'tmrv0_2_joint_2',
    'tmrv0_2_joint_3',
}

# Simulation-only passive base joints gated by include_passive_base.
PASSIVE_TMR_BASE_JOINTS = {
    'rocker_arm_joint',
    'caster_front_left_steering_joint',
    'caster_front_left_joint',
    'caster_rear_right_steering_joint',
    'caster_rear_right_joint',
}


def _joint_names_from_hardware_interfaces(interfaces):
    """Extract joint names from interface names like 'joint/position'."""
    names = set()
    for interface in interfaces:
        if '/' not in interface.name:
            continue
        names.add(interface.name.split('/', 1)[0])
    return names


def generate_test_description():
    """Launch TMR with fake hardware and joint_state_broadcaster."""
    tmr_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare('franka_bringup'),
                        'launch',
                        'tmrv0_2.launch.py',
                    ]
                )
            ]
        ),
        launch_arguments={
            'robot_config_file': str(
                Path(__file__).resolve().parent / 'config'
                / 'test_fake_hardware_tmr.config.yaml'
            ),
            'controller_name': 'swerve_drive_controller',
        }.items(),
    )

    return (
        LaunchDescription(
            [
                tmr_launch,
                TimerAction(
                    period=STARTUP_TIMEOUT,
                    actions=[launch_testing.actions.ReadyToTest()],
                ),
            ]
        ),
        {'tmr_launch': tmr_launch},
    )


class TestFakeHardwareTmr(FakeHardwareTestBase):
    """Verify TMR mobile base launches and publishes joint states."""

    NODE_NAME = 'fake_hardware_tmr_test'

    def test_tmr_publishes_drive_joints(self):
        """Verify TMR drive joints appear in aggregate /joint_states."""
        # Global /joint_states is joint_state_publisher output and still
        # includes passive URDF joints back-filled at 0.0; only require drives.
        joint_state = self.wait_for_joint_states(
            min_joints=4,
            timeout_sec=JOINT_STATE_TIMEOUT,
        )
        self.assertTrue(
            EXPECTED_TMR_JOINTS.issubset(set(joint_state.name)),
            f'Expected TMR drive joints not found. '
            f'Expected: {EXPECTED_TMR_JOINTS}, Got: {set(joint_state.name)}',
        )

    def test_ros2_control_surface_excludes_passive_base_joints(self):
        """Pin include_passive_base=false on the live ros2_control joint surface."""
        self.wait_for_stack_ready()

        franka_joint_state = self.wait_for_joint_states(
            min_joints=len(EXPECTED_TMR_JOINTS),
            timeout_sec=JOINT_STATE_TIMEOUT,
            topic=FRANKA_JOINT_STATES_TOPIC,
        )
        franka_names = set(franka_joint_state.name)
        self.assertEqual(
            franka_names,
            EXPECTED_TMR_JOINTS,
            f'{FRANKA_JOINT_STATES_TOPIC} must expose exactly the four drive '
            f'joints (no rocker/caster). Got: {sorted(franka_names)}',
        )
        self.assertFalse(
            franka_names & PASSIVE_TMR_BASE_JOINTS,
            f'{FRANKA_JOINT_STATES_TOPIC} must not contain passive base joints: '
            f'{sorted(franka_names & PASSIVE_TMR_BASE_JOINTS)}',
        )

        hardware_interfaces = self.controller_client.list_hardware_interfaces()
        self.assertIsNotNone(
            hardware_interfaces,
            'list_hardware_interfaces service did not respond',
        )
        state_joint_names = _joint_names_from_hardware_interfaces(
            hardware_interfaces.state_interfaces
        )
        registered_tmr_joints = state_joint_names & (
            EXPECTED_TMR_JOINTS | PASSIVE_TMR_BASE_JOINTS
        )
        self.assertEqual(
            registered_tmr_joints,
            EXPECTED_TMR_JOINTS,
            'ros2_control state interfaces must register exactly the four drive '
            f'joints. Got TMR joints: {sorted(registered_tmr_joints)}',
        )


@launch_testing.post_shutdown_test()
class TestFakeHardwareTmrShutdown(unittest.TestCase):
    """Verify TMR launch exits without unexpected errors."""

    IGNORED_PATTERNS = DEFAULT_SHUTDOWN_IGNORE_PATTERNS + [
        'swerve_drive_controller',
        'joint_state_publisher',
    ]

    def test_has_no_error(self, proc_output):
        """Check no unexpected ERROR messages in output."""
        error_lines = collect_unexpected_error_lines(
            proc_output, self.IGNORED_PATTERNS
        )
        self.assertEqual(
            error_lines, [],
            f'Found unexpected [ERROR] log messages: {error_lines}',
        )
