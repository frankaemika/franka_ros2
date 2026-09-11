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

"""Test that MoveIt gripper config matches the actual gripper node name."""

import os

from ament_index_python.packages import get_package_share_directory
import pytest
import yaml


GRIPPER_NODE_NAME = 'franka_gripper'


@pytest.fixture
def controllers_yaml():
    pkg_dir = get_package_share_directory('franka_fr3_moveit_config')
    yaml_path = os.path.join(pkg_dir, 'config', 'fr3_controllers.yaml')
    with open(yaml_path, 'r') as f:
        return yaml.safe_load(f)


def test_gripper_controller_name_matches_node(controllers_yaml):
    """
    Verify MoveIt controller name matches the gripper node name.

    MoveIt resolves GripperCommand action as <controller_name>/<action_ns>.
    The gripper node exposes ~/gripper_action, so the full action path is
    <node_name>/gripper_action. For MoveIt to connect, the controller name
    in fr3_controllers.yaml must equal the gripper node name.
    """
    controller_names = controllers_yaml['controller_names']
    assert GRIPPER_NODE_NAME in controller_names, (
        f'Expected gripper controller "{GRIPPER_NODE_NAME}" in controller_names, '
        f'got: {controller_names}. MoveIt will not be able to send gripper commands.'
    )


def test_gripper_controller_action_ns(controllers_yaml):
    """Gripper controller must specify the correct action namespace."""
    gripper_config = controllers_yaml.get(GRIPPER_NODE_NAME)
    assert gripper_config is not None, (
        f'No controller config block for "{GRIPPER_NODE_NAME}" in fr3_controllers.yaml'
    )
    assert gripper_config['action_ns'] == 'gripper_action', (
        f'Expected action_ns "gripper_action", got "{gripper_config["action_ns"]}"'
    )
    assert gripper_config['type'] == 'GripperCommand'


def test_joint_state_source_list_includes_gripper():
    """
    Verify moveit.launch.py source_list references franka_gripper/joint_states.

    The joint_state_broadcaster aggregates joint states from multiple sources.
    If the source_list references the wrong node name, gripper joints won't
    appear in /joint_states and MoveIt will treat the gripper as always closed.
    """
    pkg_dir = get_package_share_directory('franka_fr3_moveit_config')
    launch_path = os.path.join(pkg_dir, 'launch', 'moveit.launch.py')
    with open(launch_path, 'r') as f:
        content = f.read()

    expected_topic = f'{GRIPPER_NODE_NAME}/joint_states'
    assert expected_topic in content, (
        f'Expected "{expected_topic}" in moveit.launch.py source_list. '
        f'Gripper joint states will not be aggregated into /joint_states.'
    )

    # Ensure the OLD incorrect name is not present
    assert 'fr3_gripper/joint_states' not in content, (
        'Found stale "fr3_gripper/joint_states" in moveit.launch.py. '
        'This node does not exist — should be "franka_gripper/joint_states".'
    )
