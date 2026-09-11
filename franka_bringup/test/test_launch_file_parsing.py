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

"""Validate all first-party launch files parse without errors."""

import importlib.util
import os

from ament_index_python.packages import (
    get_package_share_directory,
    PackageNotFoundError,
)

from launch import LaunchDescription

import pytest


LAUNCH_FILES = [
    ('franka_bringup', 'franka.launch.py'),
    ('franka_bringup', 'fr3_duo.launch.py'),
    ('franka_bringup', 'mobile_fr3_duo.launch.py'),
    ('franka_bringup', 'example.launch.py'),
    ('franka_bringup', 'tmrv0_2.launch.py'),
    ('franka_bringup', 'mobile_teleop.launch.py'),
    ('franka_bringup', 'joint_impedance_with_ik_example_controller.launch.py'),
    ('franka_gripper', 'gripper.launch.py'),
    ('franka_fr3_moveit_config', 'moveit.launch.py'),
    ('franka_fr3_moveit_config', 'move_group.launch.py'),
    ('franka_mobile_fr3_duo_moveit_config', 'moveit.launch.py'),
    ('franka_mobile_fr3_duo_moveit_config', 'moveit_nodes.launch.py'),
    ('franka_gazebo_bringup', 'visualize_franka_robot.launch.py'),
    ('franka_gazebo_bringup', 'gazebo_franka_arm_example_controller.launch.py'),
    ('franka_gazebo_bringup', 'gazebo_fr3_duo_example.launch.py'),
    ('franka_gazebo_bringup', 'gazebo_mobile_fr3_duo_example.launch.py'),
    ('franka_gazebo_bringup', 'gazebo_tmr_example_controller.launch.py'),
    ('franka_mobile_sensors', 'franka_mobile_sensors.launch.py'),
    ('franka_vision_and_manipulation_kit', 'franka_vision_and_manipulation_kit.launch.py'),
]


def load_launch_description(
    package_name: str, launch_file_name: str
) -> tuple[str, LaunchDescription]:
    """Load a launch file as a Python module and return its LaunchDescription."""
    # This test intentionally spans launch files owned by other packages (e.g.
    # the MoveIt configs). Those cannot be declared as franka_bringup test
    # dependencies without a circular dependency, so in isolated
    # (--packages-up-to) builds they may be absent. Skip rather than fail there;
    # full-workspace CI still exercises every launch file.
    try:
        package_directory = get_package_share_directory(package_name)
    except PackageNotFoundError:
        pytest.skip(f'Package {package_name} not available in this build')
    launch_file = os.path.join(package_directory, 'launch', launch_file_name)

    assert os.path.exists(launch_file), f'Launch file not found: {launch_file}'

    module_name = (
        f'launch_test_{package_name}_{launch_file_name.removesuffix(".py").replace(".", "_")}'
    )
    spec = importlib.util.spec_from_file_location(module_name, launch_file)
    assert spec is not None and spec.loader is not None, (
        f'Unable to create an import spec for {launch_file}'
    )

    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    assert hasattr(module, 'generate_launch_description'), (
        f'{launch_file} has no generate_launch_description()'
    )

    return launch_file, module.generate_launch_description()


@pytest.mark.parametrize(
    'package_name,launch_file_name',
    LAUNCH_FILES,
    ids=[f'{package_name}/{launch_file_name}' for package_name, launch_file_name in LAUNCH_FILES],
)
def test_launch_file_parses(package_name: str, launch_file_name: str) -> None:
    """Verify launch file loads and generates a valid LaunchDescription."""
    launch_file, launch_description = load_launch_description(package_name, launch_file_name)

    assert launch_description is not None
    assert launch_description.entities, (
        f'{package_name}/{launch_file_name} generated an empty LaunchDescription'
    )

    declared_arguments = launch_description.get_launch_arguments(conditional_inclusion=True)
    assert all(argument.name for argument in declared_arguments), (
        f'{launch_file} contains an invalid launch argument declaration'
    )
