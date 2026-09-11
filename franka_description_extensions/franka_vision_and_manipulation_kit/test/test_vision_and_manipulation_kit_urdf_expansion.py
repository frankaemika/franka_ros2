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

"""Regression tests for standalone vision and manipulation kit xacro entry points."""

import os
import xml.etree.ElementTree as ElementTree

from ament_index_python.packages import get_package_share_directory
import pytest
import xacro


VISION_CASES = (
    ('stationary', {}, ('left', 'right'), '', False),
    (
        'stationary_prefixed',
        {'mobile_platform': 'false', 'prefix': 'test_'},
        ('test_left', 'test_right'),
        'test_',
        False,
    ),
    ('mobile', {'mobile_platform': 'true'}, ('left', 'right'), '', False),
    (
        'mobile_prefixed',
        {'mobile_platform': 'true', 'prefix': 'test_'},
        ('left', 'right'),
        'test_',
        False,
    ),
    (
        'stationary_fake_control',
        {'ros2_control': 'true', 'use_fake_hardware': 'true'},
        ('left', 'right'),
        '',
        True,
    ),
)

INVALID_PLACEHOLDER_ELEMENTS = {
    'included_robot_macro',
    'includeded_robot_macro',
}


def vision_xacro_path() -> str:
    """Resolve the vision kit xacro through the installed package index."""
    return os.path.join(
        get_package_share_directory('franka_vision_and_manipulation_kit'),
        'robots',
        'vision_and_manipulation_kit.urdf.xacro',
    )


@pytest.fixture(
    scope='module',
    params=VISION_CASES,
    ids=[case[0] for case in VISION_CASES],
)
def expanded_vision_case(request):
    """Expand one supported vision kit argument combination."""
    case_name, mappings, arm_prefixes, accessory_prefix, expects_control = request.param
    urdf = xacro.process_file(
        vision_xacro_path(),
        mappings=mappings,
    ).toxml()
    return {
        'name': case_name,
        'root': ElementTree.fromstring(urdf),
        'urdf': urdf,
        'arm_prefixes': arm_prefixes,
        'accessory_prefix': accessory_prefix,
        'expects_control': expects_control,
    }


def joint_links(root: ElementTree.Element, joint_name: str) -> tuple[str, str]:
    """Return the parent and child links for one uniquely named joint."""
    joints = [
        joint
        for joint in root.findall('joint')
        if joint.get('name') == joint_name
    ]
    assert len(joints) == 1, f'Expected one joint named {joint_name!r}'
    parent = joints[0].find('parent')
    child = joints[0].find('child')
    assert parent is not None, f'Joint {joint_name!r} has no parent element'
    assert child is not None, f'Joint {joint_name!r} has no child element'
    parent_link = parent.get('link')
    child_link = child.get('link')
    assert parent_link is not None, f'Joint {joint_name!r} has no parent link'
    assert child_link is not None, f'Joint {joint_name!r} has no child link'
    return parent_link, child_link


def named_elements(root: ElementTree.Element, element_name: str) -> list[str]:
    """Return the names of all links or joints in an expanded description."""
    names = [
        element.get('name')
        for element in root.findall(element_name)
    ]
    assert all(name for name in names), f'Unnamed {element_name} element found'
    return names


def assert_joint_links_are_emitted(root: ElementTree.Element) -> None:
    """Assert every joint references links emitted by the description."""
    emitted_link_names = set(named_elements(root, 'link'))
    for joint in root.findall('joint'):
        joint_name = joint.get('name')
        for relation in ('parent', 'child'):
            link_element = joint.find(relation)
            assert link_element is not None, (
                f'Joint {joint_name!r} has no {relation} element'
            )
            link_name = link_element.get('link')
            assert link_name is not None, (
                f'Joint {joint_name!r} has no {relation} link'
            )
            assert link_name in emitted_link_names, (
                f'Joint {joint_name!r} {relation} references missing link '
                f'{link_name!r}'
            )


def minimal_joint_graph() -> ElementTree.Element:
    """Build a valid one-joint graph for mutation tests."""
    return ElementTree.fromstring(
        '<robot><link name="base_link"/><link name="sensor_link"/>'
        '<joint name="sensor_joint"><parent link="base_link"/>'
        '<child link="sensor_link"/></joint></robot>'
    )


def test_rejects_dangling_joint_parent():
    """A joint parent must reference an emitted link."""
    root = minimal_joint_graph()
    root.find('joint/parent').set('link', 'missing_link')

    with pytest.raises(AssertionError, match='parent references missing link'):
        assert_joint_links_are_emitted(root)


def test_rejects_dangling_joint_child():
    """A joint child must reference an emitted link."""
    root = minimal_joint_graph()
    root.find('joint/child').set('link', 'missing_link')

    with pytest.raises(AssertionError, match='child references missing link'):
        assert_joint_links_are_emitted(root)


def test_expands_to_well_formed_xml(expanded_vision_case):
    """Each supported vision kit combination expands to a robot description."""
    root = expanded_vision_case['root']
    assert root.tag == 'robot'
    assert root.findall('.//link')
    assert root.findall('.//joint')
    if expanded_vision_case['expects_control']:
        assert root.findall('.//ros2_control')


def test_emits_unique_link_and_joint_names(expanded_vision_case):
    """Expanded descriptions do not contain duplicate link or joint names."""
    root = expanded_vision_case['root']
    link_names = named_elements(root, 'link')
    joint_names = named_elements(root, 'joint')
    assert len(link_names) == len(set(link_names)), 'Duplicate link name found'
    assert len(joint_names) == len(set(joint_names)), 'Duplicate joint name found'


def test_joint_links_reference_emitted_links(expanded_vision_case):
    """Every expanded joint references an emitted link."""
    root = expanded_vision_case['root']
    assert_joint_links_are_emitted(root)


def test_attaches_accessories_to_expected_mounting_points(expanded_vision_case):
    """Wrist accessories use arm link8 and the head camera uses its mount."""
    root = expanded_vision_case['root']
    accessory_prefix = expanded_vision_case['accessory_prefix']
    for side, arm_prefix in zip(('left', 'right'), expanded_vision_case['arm_prefixes']):
        joint_name = f'{accessory_prefix}{side}_ur_to_robotiq_joint'
        expected_parent = f'{arm_prefix}_fr3v2_1_link8'
        parent_link, _ = joint_links(root, joint_name)
        assert parent_link == expected_parent
    head_parent, head_child = joint_links(root, 'head_camera_mounting_joint')
    assert head_parent == 'head_camera_mounting_point'
    assert head_child == f'{accessory_prefix}head_camera_link'


def test_has_no_gazebo_or_placeholder_elements(expanded_vision_case):
    """Standalone descriptions contain no Gazebo-only or placeholder output."""
    root = expanded_vision_case['root']
    element_tags = {element.tag for element in root.iter()}
    assert not element_tags & INVALID_PLACEHOLDER_ELEMENTS
    assert not root.findall('.//gazebo')
    assert not any(
        'gazebo' in attribute_name.lower()
        for element in root.iter()
        for attribute_name in element.attrib
    )
