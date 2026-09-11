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

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_robot_nodes(context):
    robot_ip = LaunchConfiguration('robot_ip').perform(context)
    namespace = LaunchConfiguration('namespace').perform(context)

    battery_config = os.path.join(
        get_package_share_directory('franka_mobile'),
        'config',
        'franka_battery_node.yaml',
    )

    nodes = [
        Node(
            package='franka_mobile',
            executable='battery_server_node.py',
            name='franka_battery_node',
            namespace=namespace,
            parameters=[{'robot_ip': robot_ip}, battery_config],
            output='screen',
        ),
    ]
    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'robot_ip',
                description='IP address or hostname of the Franka TMR robot.',
            ),
            DeclareLaunchArgument(
                'namespace',
                default_value='',
                description='Namespace for the battery node.',
            ),
            OpaqueFunction(function=generate_robot_nodes),
        ]
    )
