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

"""Fake-hardware integration test for FR3 single-arm configuration."""
# Tests controller lifecycle with use_fake_hardware:=true.
# CONTROLLERS_TO_ACTIVATE: effort-only, can fully activate with
# mock_components.
# CONTROLLERS_TO_LOAD_ONLY: need Franka GPIO, can only load without
# real hardware.

from pathlib import Path
import time
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

import rclpy

import yaml


# Controllers that can fully activate with mock_components/GenericSystem
CONTROLLERS_TO_ACTIVATE = [
    'gravity_compensation_example_controller',
    'joint_impedance_example_controller',
]

# Controllers that need Franka-specific GPIO/command interfaces
CONTROLLERS_TO_LOAD_ONLY = [
    'joint_position_example_controller',
    'joint_velocity_example_controller',
    'cartesian_pose_example_controller',
    'cartesian_velocity_example_controller',
    'cartesian_elbow_example_controller',
    'cartesian_orientation_example_controller',
    'elbow_example_controller',
]


def _load_test_config(config_name):
    config_path = Path(__file__).resolve().parent / 'config' / config_name
    with config_path.open(encoding='utf-8') as config_file:
        return yaml.safe_load(config_file)


class TestFakeHardwareControllers(FakeHardwareTestBase):
    """Test controller lifecycle with fake hardware (FR3 single arm)."""

    NODE_NAME = 'fake_hardware_controllers_test'

    def _is_controller_loaded(self, controller_name):
        return any(
            c.name == controller_name
            for c in self.controller_client.list_controllers(timeout_sec=5.0)
        )

    def _wait_for_controller_unload(self, controller_name, timeout_sec=10.0):
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if not self._is_controller_loaded(controller_name):
                return True
            rclpy.spin_once(self.node, timeout_sec=0.1)
        return False

    def _exercise_controller(self, controller_name):
        """Load, configure, activate, verify joint states, then clean up."""
        controller_loaded = False
        controller_activated = False

        try:
            self.assertTrue(
                self.controller_client.load_controller(
                    controller_name, timeout_sec=10.0
                ),
                f'Failed to load {controller_name}',
            )
            controller_loaded = True

            self.assertTrue(
                self.controller_client.configure_controller(
                    controller_name, timeout_sec=10.0
                ),
                f'Failed to configure {controller_name}',
            )
            self.assertTrue(
                self.controller_client.wait_for_controller_state(
                    controller_name, ['inactive'], timeout_sec=10.0
                ),
                f'{controller_name} did not reach inactive state',
            )

            self.assertTrue(
                self.controller_client.switch_controllers(
                    activate=[controller_name], timeout_sec=10.0,
                ),
                f'Failed to activate {controller_name}',
            )
            controller_activated = True

            self.assertTrue(
                self.controller_client.wait_for_controller_state(
                    controller_name, ['active'], timeout_sec=10.0
                ),
                f'{controller_name} did not reach active state',
            )

            joint_state = self.wait_for_joint_states(min_joints=7)
            self.assertGreaterEqual(len(joint_state.name), 7)
        finally:
            if controller_activated:
                self.controller_client.switch_controllers(
                    deactivate=[controller_name],
                    strict=False, timeout_sec=10.0,
                )
            if controller_loaded:
                self.controller_client.unload_controller(
                    controller_name, timeout_sec=10.0
                )
                self._wait_for_controller_unload(
                    controller_name, timeout_sec=10.0
                )

    def test_example_controllers_activate(self):
        """Test controllers that can fully activate with fake hardware."""
        self.wait_for_stack_ready()

        for controller_name in CONTROLLERS_TO_ACTIVATE:
            with self.subTest(controller=controller_name):
                self._exercise_controller(controller_name)
                self.assertFalse(
                    self._is_controller_loaded(controller_name),
                    f'{controller_name} is still loaded after unload',
                )

    def test_controllers_load_and_configure(self):
        """Test controllers needing Franka GPIO can at least load."""
        self.wait_for_stack_ready()

        for controller_name in CONTROLLERS_TO_LOAD_ONLY:
            with self.subTest(controller=controller_name):
                try:
                    self.assertTrue(
                        self.controller_client.load_controller(
                            controller_name, timeout_sec=10.0
                        ),
                        f'Failed to load {controller_name}',
                    )
                finally:
                    self.controller_client.unload_controller(
                        controller_name, timeout_sec=10.0
                    )
                    self._wait_for_controller_unload(
                        controller_name, timeout_sec=10.0
                    )


@launch_testing.post_shutdown_test()
class TestFakeHardwareControllersShutdown(unittest.TestCase):
    """Verify the FR3 launch exits without unexpected errors."""

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
    """Create the FR3 fake-hardware launch for controller lifecycle tests."""
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
