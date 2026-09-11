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
class CameraConfig:
    """Configuration for a single camera."""

    name: str
    namespace: str
    device_profile: str
    serial_number: str = None
    usb_port: str = None

    @property
    def node_name(self) -> str:
        return f'{self.name}_node'

    @classmethod
    def from_dict(cls, data: Dict) -> 'CameraConfig':
        return cls(
            name=data['name'],
            namespace=data['namespace'],
            device_profile=data['device_profile'],
            serial_number=data.get('serial_number'),
            usb_port=data.get('usb_port'),
        )

    def load_camera_parameters(self) -> Dict:
        """
        Load camera-specific parameters from the device profile file.

        Returns
        -------
        dict
            Dictionary containing camera-specific parameters.

        """
        return load_yaml_config(self.device_profile, subdirectory='cameras')


@dataclass(frozen=True)
class CameraSuite:
    """Configuration for all cameras."""

    name: str
    description: str
    cameras: List[CameraConfig] = field(default_factory=list)

    def __post_init__(self):
        """Validate camera configuration after initialization."""
        camera_names = [cam.name for cam in self.cameras]
        serial_numbers = [
            cam.serial_number for cam in self.cameras if cam.serial_number is not None]
        usb_ports = [
            cam.usb_port for cam in self.cameras if cam.usb_port is not None]

        if len(set(camera_names)) != len(camera_names):
            raise ValueError('Camera names must be unique')
        if len(set(serial_numbers)) != len(serial_numbers):
            raise ValueError('Camera serial numbers must be unique')
        if len(set(usb_ports)) != len(usb_ports):
            raise ValueError('Camera USB ports must be unique')

    @classmethod
    def from_dict(cls, data: Dict) -> 'CameraSuite':
        cameras = [CameraConfig.from_dict(cam_data)
                   for cam_data in data['cameras']]
        return cls(
            name=data['name'],
            description=data['description'],
            cameras=cameras)


def load_camera_suite_from_yaml(config_name: str) -> CameraSuite:
    """
    Load camera suite configuration from YAML file.

    Parameters
    ----------
    config_name : str
        Name of the config file (without .yaml extension)

    Returns
    -------
    CameraSuite
        CameraSuite object loaded from the YAML file.

    """
    config_data = load_yaml_config(config_name)
    return CameraSuite.from_dict(config_data)
