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

"""Verify Gazebo resource paths for the FR3 Duo example launches."""

import importlib.util
import os
from pathlib import Path
import unittest

from ament_index_python.packages import get_package_share_directory


LAUNCH_FILES = (
    'gazebo_fr3_duo_example.launch.py',
    'gazebo_mobile_fr3_duo_example.launch.py',
)
RESOURCE_PATH_VARIABLE = 'GZ_SIM_RESOURCE_PATH'
REALSENSE_MESH = Path('realsense2_description/meshes/d405.stl')


class LaunchContextWithValue:
    """Return one launch argument value for opaque-function tests."""

    def __init__(self, value):
        self.value = value

    def perform_substitution(self, _):
        return self.value


def load_launch_module(launch_file_name):
    """Load one installed Gazebo launch module."""
    launch_file = Path(
        get_package_share_directory('franka_gazebo_bringup'),
        'launch',
        launch_file_name,
    )
    spec = importlib.util.spec_from_file_location(
        f'resource_path_{launch_file_name.replace(".", "_")}',
        launch_file,
    )
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class TestGzSimResourcePath(unittest.TestCase):
    """Verify the launch resource paths resolve sensor assets."""

    def setUp(self):
        self.previous_resource_path = os.environ.get(RESOURCE_PATH_VARIABLE)

    def tearDown(self):
        if self.previous_resource_path is None:
            os.environ.pop(RESOURCE_PATH_VARIABLE, None)
        else:
            os.environ[RESOURCE_PATH_VARIABLE] = self.previous_resource_path

    def test_sensor_paths_resolve_realsense_mesh_and_preserve_existing_path(self):
        """Sensor launches resolve the D405 mesh without losing caller paths."""
        preserved_path = '/tmp/gazebo-resource-path'
        realsense_parent = os.path.dirname(
            get_package_share_directory('realsense2_description'))
        os.environ[RESOURCE_PATH_VARIABLE] = os.pathsep.join((
            preserved_path,
            '',
            preserved_path,
        ))

        for launch_file_name in LAUNCH_FILES:
            with self.subTest(launch_file_name=launch_file_name):
                os.environ[RESOURCE_PATH_VARIABLE] = os.pathsep.join((
                    preserved_path,
                    '',
                    preserved_path,
                ))
                module = load_launch_module(launch_file_name)
                module.set_gz_sim_resource_path(
                    LaunchContextWithValue('true'),
                    object(),
                )
                resource_paths = os.environ[RESOURCE_PATH_VARIABLE].split(os.pathsep)
                mesh_path = next(
                    (
                        Path(resource_path, REALSENSE_MESH)
                        for resource_path in resource_paths
                        if Path(resource_path, REALSENSE_MESH).is_file()
                    ),
                    None,
                )

                self.assertEqual(len(resource_paths), len(set(resource_paths)))
                self.assertNotIn('', resource_paths)
                self.assertIn(preserved_path, resource_paths)
                self.assertIn(realsense_parent, resource_paths)
                self.assertIsNotNone(mesh_path)

    def test_no_sensor_paths_preserve_existing_path_without_sensor_assets(self):
        """Non-sensor launches keep their existing path without sensor roots."""
        preserved_path = '/tmp/gazebo-resource-path'
        description_parent = os.path.dirname(
            get_package_share_directory('franka_description'))
        realsense_parent = os.path.dirname(
            get_package_share_directory('realsense2_description'))
        os.environ[RESOURCE_PATH_VARIABLE] = os.pathsep.join((
            preserved_path,
            '',
            preserved_path,
        ))

        for launch_file_name in LAUNCH_FILES:
            with self.subTest(launch_file_name=launch_file_name):
                os.environ[RESOURCE_PATH_VARIABLE] = os.pathsep.join((
                    preserved_path,
                    '',
                    preserved_path,
                ))
                module = load_launch_module(launch_file_name)
                module.set_gz_sim_resource_path(
                    LaunchContextWithValue('false'),
                    object(),
                )
                resource_paths = os.environ[RESOURCE_PATH_VARIABLE].split(os.pathsep)

                self.assertEqual(
                    resource_paths,
                    [description_parent, preserved_path],
                )
                self.assertNotIn(realsense_parent, resource_paths)
