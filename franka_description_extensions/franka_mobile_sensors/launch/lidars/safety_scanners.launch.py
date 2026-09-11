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
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

current_dir = Path(__file__).parent
sys.path.append(str(current_dir))

from lidar_configs import (LidarConfig, LidarSuite,  # noqa
                           load_lidar_suite_from_yaml, NetworkConfig)


def create_lidar_launch_arguments() -> List[DeclareLaunchArgument]:
    """Create lidar launch arguments."""
    args = [
        DeclareLaunchArgument(
            'config_file',
            default_value='default_sensor_suite',
            description='Lidar configuration file to use (without .yaml extension)',
        ),
    ]
    return args


def create_lidar_node(
        lidar_config: LidarConfig,
        network_config: NetworkConfig) -> Node:
    """Create a lidar node for launch."""
    lidar_specific_params = lidar_config.load_lidar_parameters()
    base_params = {
        'frame_id': lidar_config.frame_id,
        'sensor_ip': lidar_config.sensor_ip,
        'host_ip': network_config.host_ip,
        'interface_ip': network_config.interface_ip,
        'host_udp_port': network_config.host_udp_port,
    }
    all_params = {**base_params, **lidar_specific_params}
    return Node(
        package='sick_safetyscanners2',
        executable='sick_safetyscanners2_node',
        name=lidar_config.node_name,
        namespace=lidar_config.namespace,
        output='screen',
        emulate_tty=True,
        parameters=[all_params],
    )


def create_lidar_nodes(
        context: LaunchContext,
        lidar_suite: LidarSuite) -> List[Node]:
    """Create lidar nodes for launch."""
    lidar_nodes = []
    for lidar in lidar_suite.lidars:
        lidar_node = create_lidar_node(lidar, lidar_suite.network)
        lidar_nodes.append(lidar_node)
    return lidar_nodes


def lidar_launch_setup(context: LaunchContext, *args, **kwargs):
    """Set up lidar launch."""
    config_file = context.perform_substitution(
        LaunchConfiguration('config_file'))
    lidar_suite = load_lidar_suite_from_yaml(config_file)
    lidar_nodes = create_lidar_nodes(context, lidar_suite)
    return lidar_nodes


def generate_launch_description() -> LaunchDescription:
    """Generate launch description."""
    return LaunchDescription(
        [*create_lidar_launch_arguments(), OpaqueFunction(function=lidar_launch_setup)]
    )


if __name__ == '__main__':
    generate_launch_description()
