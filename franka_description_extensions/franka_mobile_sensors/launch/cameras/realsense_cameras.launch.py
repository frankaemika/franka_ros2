#  Copyright (c) 2025 Franka Robotics GmbH
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

from pathlib import Path
import sys
from typing import List

from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

current_dir = Path(__file__).parent
sys.path.append(str(current_dir))

from camera_configs import CameraSuite, load_camera_suite_from_yaml  # noqa


def create_camera_launch_arguments() -> List[DeclareLaunchArgument]:
    """Create camera launch arguments."""
    args = [
        DeclareLaunchArgument(
            'config_file',
            default_value='default_sensor_suite',
            description='Camera configuration file to use (without .yaml extension)',
        ),
    ]
    return args


def create_camera_nodes(
    context: LaunchContext, camera_suite: CameraSuite
) -> List[GroupAction]:
    """Create camera nodes for launch."""
    camera_groups = []
    for i, camera in enumerate(camera_suite.cameras, 1):
        camera_name = camera.name
        namespace = camera.namespace
        camera_specific_params = camera.load_camera_parameters()
        # Build base params - use serial_no if available, otherwise usb_port_id
        base_params = {'camera_name': camera_name}
        if camera.serial_number:
            base_params['serial_no'] = camera.serial_number
        elif camera.usb_port:
            base_params['usb_port_id'] = camera.usb_port
        else:
            raise ValueError(
                f'Camera {camera_name} must have either serial_number or usb_port defined.')
        all_params = {**base_params, **camera_specific_params}
        realsense_node = Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name=camera_name,
            namespace=namespace,
            parameters=[all_params],
            output='screen',
        )
        camera_group = GroupAction([realsense_node])
        camera_groups.append(camera_group)
    return camera_groups


def camera_launch_setup(context: LaunchContext, *args, **kwargs):
    """Set up camera launch."""
    config_file = context.perform_substitution(
        LaunchConfiguration('config_file'))
    camera_suite = load_camera_suite_from_yaml(config_file)
    camera_nodes = create_camera_nodes(context, camera_suite)
    return camera_nodes


def generate_launch_description() -> LaunchDescription:
    """Generate launch description."""
    return LaunchDescription(
        [
            *create_camera_launch_arguments(),
            OpaqueFunction(function=camera_launch_setup),
        ]
    )


if __name__ == '__main__':
    generate_launch_description()
