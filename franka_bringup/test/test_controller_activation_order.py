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

"""Fake-hardware controller activation order test for FR3 single arm."""

from pathlib import Path

from franka_bringup.testing.fake_hardware_test_base import FakeHardwareTestBase

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare

import launch_testing.actions

import yaml


JOINT_STATE_BROADCASTER = 'joint_state_broadcaster'


def _load_test_config(config_name):
    config_path = Path(__file__).resolve().parent / 'config' / config_name
    with config_path.open(encoding='utf-8') as config_file:
        return yaml.safe_load(config_file)


class TestControllerActivationOrder(FakeHardwareTestBase):
    """Verify fake-hardware startup activates controllers correctly."""

    NODE_NAME = 'controller_activation_order_test'

    def test_broadcaster_is_active(self):
        """Verify joint_state_broadcaster is the first active controller."""
        self.wait_for_stack_ready()

        controllers = self.controller_client.list_controllers(
            timeout_sec=5.0
        )
        active_controllers = [
            c.name for c in controllers if c.state == 'active'
        ]

        self.assertEqual(
            active_controllers,
            [JOINT_STATE_BROADCASTER],
            'Unexpected active controllers during startup: '
            f'{active_controllers}',
        )

    def test_no_controller_in_error_state(self):
        """Verify no controller reaches the finalized error state."""
        self.wait_for_stack_ready()

        controllers = self.controller_client.list_controllers(
            timeout_sec=5.0
        )
        finalized_controllers = [
            c.name for c in controllers if c.state == 'finalized'
        ]

        self.assertEqual(
            finalized_controllers,
            [],
            'Controllers reached finalized state: '
            f'{finalized_controllers}',
        )


def generate_test_description():
    """Create the FR3 fake-hardware launch for activation order tests."""
    config = _load_test_config(
        'test_fake_hardware_fr3.config.yaml'
    )['TEST_ROBOT_0']

    franka_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare('franka_bringup'),
                        'launch',
                        'franka.launch.py',
                    ]
                )
            ]
        ),
        launch_arguments={
            'robot_type': config['robot_type'],
            'arm_prefix': config['arm_prefix'],
            'namespace': config['namespace'],
            'robot_ip': config['robot_ip'],
            'load_gripper': config['load_gripper'],
            'use_fake_hardware': config['use_fake_hardware'],
            'fake_sensor_commands': config['fake_sensor_commands'],
        }.items(),
    )

    return (
        LaunchDescription(
            [
                franka_launch,
                launch_testing.actions.ReadyToTest(),
            ]
        ),
        {},
    )
