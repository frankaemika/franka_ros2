# Copyright (c) 2026 Franka Robotics GmbH
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

"""
Expansion guard for the mobile_fr3_duo_v0_2 MoveIt URDF.

Expands mobile_fr3_duo_v0_2.moveit.urdf.xacro across the four canonical arg
combinations and asserts that hardware emission survives: the real/mock branch
keeps three ros2_control blocks; the Gazebo branch collapses to one block, gains
a gz_ros2_control system element, and gates effort interfaces on gazebo_effort.
"""

from os import path
import xml.etree.ElementTree as ElementTree

from ament_index_python.packages import get_package_share_directory

import pytest

import xacro

MOVEIT_URDF = 'mobile_fr3_duo_v0_2.moveit.urdf.xacro'

GZ_SYSTEM_PLUGIN_FILENAME = 'gz_ros2_control-system'
GZ_SIM_PLUGIN = 'gz_ros2_control/GazeboSimSystem'
FRANKA_GRAVITY_COMP_PLUGIN = 'franka_gazebo_hardware/FrankaGazeboHardwareInterface'
REAL_PLUGIN = 'franka_hardware/FrankaHardwareInterface'
MOCK_PLUGIN = 'mock_components/GenericSystem'


def moveit_urdf_path() -> str:
    """Resolve the MoveIt URDF xacro from the package share directory."""
    return path.join(
        get_package_share_directory('franka_mobile_fr3_duo_moveit_config'),
        'urdf',
        MOVEIT_URDF,
    )


def expand(mappings: dict) -> str:
    """Expand the MoveIt URDF xacro with the given args and return the URDF string."""
    return xacro.process_file(moveit_urdf_path(), mappings=mappings).toxml()


def hardware_plugins(urdf: str) -> list[str]:
    """Return the hardware plugin name of every <ros2_control> block, in order."""
    root = ElementTree.fromstring(urdf)
    plugins = []
    for block in root.findall('.//ros2_control'):
        plugin = block.find('.//hardware/plugin')
        plugins.append((plugin.text or '').strip() if plugin is not None else '')
    return plugins


def has_gz_system(urdf: str) -> bool:
    """Whether the gz_ros2_control system <plugin> element is present."""
    return f'filename="{GZ_SYSTEM_PLUGIN_FILENAME}"' in urdf


def has_effort_command(urdf: str) -> bool:
    """Whether any joint exposes an effort command interface."""
    return 'command_interface name="effort"' in urdf


# Args expand cleanly with defaults; only the differentiating flags are mapped.
CASES = {
    'real': {'simulate_in_gazebo': 'false'},
    'fake': {'use_fake_hardware': 'true'},
    'gz_effort_false': {'simulate_in_gazebo': 'true', 'gazebo_effort': 'false'},
    'gz_effort_true': {'simulate_in_gazebo': 'true', 'gazebo_effort': 'true'},
}


@pytest.mark.parametrize('mappings', CASES.values(), ids=CASES.keys())
def test_expansion_well_formed(mappings: dict):
    """xacro expansion succeeds and emits parseable, non-empty XML."""
    urdf = expand(mappings)
    assert len(urdf) > 0
    ElementTree.fromstring(urdf)


def test_expansion_real():
    """Real hardware: three FrankaHardwareInterface blocks, no Gazebo system."""
    plugins = hardware_plugins(expand(CASES['real']))
    assert plugins == [REAL_PLUGIN, REAL_PLUGIN, REAL_PLUGIN]
    assert not has_gz_system(expand(CASES['real']))


def test_expansion_fake():
    """Fake hardware: three mock_components blocks, no Gazebo system."""
    plugins = hardware_plugins(expand(CASES['fake']))
    assert plugins == [MOCK_PLUGIN, MOCK_PLUGIN, MOCK_PLUGIN]
    assert not has_gz_system(expand(CASES['fake']))


def test_expansion_gz_effort_false():
    """Gazebo without effort: one GazeboSimSystem block, gz system, no effort."""
    urdf = expand(CASES['gz_effort_false'])
    assert hardware_plugins(urdf) == [GZ_SIM_PLUGIN]
    assert has_gz_system(urdf)
    assert not has_effort_command(urdf)


def test_expansion_gz_effort_true():
    """Gazebo with effort: one gravity-comp block, gz system, effort interfaces."""
    urdf = expand(CASES['gz_effort_true'])
    assert hardware_plugins(urdf) == [FRANKA_GRAVITY_COMP_PLUGIN]
    assert has_gz_system(urdf)
    assert has_effort_command(urdf)


@pytest.mark.parametrize(
    'case, expect_gz_system',
    [('real', False), ('fake', False), ('gz_effort_false', True), ('gz_effort_true', True)],
)
def test_gz_system_iff_simulate(case: str, expect_gz_system: bool):
    """The gz_ros2_control system element appears only when simulate_in_gazebo:=true."""
    assert has_gz_system(expand(CASES[case])) == expect_gz_system


@pytest.mark.parametrize(
    'case, expect_effort',
    [('gz_effort_false', False), ('gz_effort_true', True)],
)
def test_effort_iff_gazebo_effort(case: str, expect_effort: bool):
    """In Gazebo, effort command interfaces appear only when gazebo_effort:=true."""
    assert has_effort_command(expand(CASES[case])) == expect_effort
