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

from dataclasses import dataclass, field
from pathlib import Path
import sys
from typing import Dict, List

# Add utils to path for shared utilities
current_dir = Path(__file__).parent
sys.path.append(str(current_dir.parent / 'utils'))

from config_loader import load_yaml_config  # noqa: E402


@dataclass(frozen=True)
class LidarConfig:
    """Configuration for a single lidar."""

    name: str
    namespace: str
    frame_id: str
    sensor_ip: str
    device_profile: str

    @property
    def node_name(self) -> str:
        return f'{self.name}_node'

    @classmethod
    def from_dict(cls, data: Dict) -> 'LidarConfig':
        return cls(
            name=data['name'],
            namespace=data['namespace'],
            frame_id=data['frame_id'],
            sensor_ip=data['sensor_ip'],
            device_profile=data['device_profile'],
        )

    def load_lidar_parameters(self) -> Dict:
        """
        Load lidar-specific parameters from the device profile file.

        Returns
        -------
        dict
            Dictionary containing lidar-specific parameters.

        """
        return load_yaml_config(self.device_profile, subdirectory='lidars')


@dataclass(frozen=True)
class NetworkConfig:
    """Network configuration for sensors."""

    host_ip: str = '172.16.1.9'
    interface_ip: str = '0.0.0.0'
    host_udp_port: int = 0

    @classmethod
    def from_dict(cls, data: Dict) -> 'NetworkConfig':
        return cls(
            host_ip=data['host_ip'],
            interface_ip=data['interface_ip'],
            host_udp_port=data['host_udp_port'],
        )


@dataclass(frozen=True)
class LidarSuite:
    """Configuration for all lidars."""

    name: str
    description: str
    lidars: List[LidarConfig] = field(default_factory=list)
    network: NetworkConfig = field(default_factory=NetworkConfig)

    def __post_init__(self):
        """Validate lidar configuration after initialization."""
        lidar_names = [lidar.name for lidar in self.lidars]
        if len(set(lidar_names)) != len(lidar_names):
            raise ValueError('Lidar names must be unique')

    @classmethod
    def from_dict(cls, data: Dict) -> 'LidarSuite':
        lidars = [LidarConfig.from_dict(lidar_data)
                  for lidar_data in data['lidars']]
        network = NetworkConfig.from_dict(data.get('network', {}))
        return cls(
            name=data['name'],
            description=data['description'],
            lidars=lidars,
            network=network,
        )


def load_lidar_suite_from_yaml(config_name: str) -> LidarSuite:
    """
    Load lidar suite configuration from YAML file.

    Parameters
    ----------
    config_name : str
        Name of the config file (without .yaml extension)

    Returns
    -------
    LidarSuite
        LidarSuite object loaded from the YAML file.

    """
    config_data = load_yaml_config(config_name)
    return LidarSuite.from_dict(config_data)
