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

"""Battery controller that encapsulates battery REST business logic."""

from dataclasses import dataclass, field
import logging
from typing import List, Optional

from franka_mobile.battery_api_client import BatteryApiClient


@dataclass
class MobileChargingSystemStatusResult:
    """Result of a mobile charging system status query."""

    success: bool
    status: str = ''
    message: str = ''


@dataclass
class TemperaturesData:
    """Charging system temperatures in degC."""

    heatsink: float = 0.0
    coil_mobile: float = 0.0
    hf1_terminal: float = 0.0
    batt_pos_terminal: float = 0.0


@dataclass
class SystemStatesResult:
    """Result of a system-states query."""

    success: bool
    battery_level_rsoc: int = 0
    mob_charging_unit_errors: List[str] = field(default_factory=list)
    temperatures: TemperaturesData = field(default_factory=TemperaturesData)
    wireless_charging_active: bool = False
    wired_charging_active: bool = False
    message: str = ''


@dataclass
class IsChargingResult:
    """Result of an is-charging query."""

    success: bool
    is_charging: bool = False
    message: str = ''


@dataclass
class SystemInfoResult:
    """Result of a system-info query."""

    success: bool
    serial_number_mobile: int = 0
    serial_number_station: int = 0
    sw_version_mobile_revision: int = 0
    sw_version_mobile_major: int = 0
    sw_version_mobile_minor: int = 0
    message: str = ''


@dataclass
class CommandResult:
    """Result of a simple battery command."""

    success: bool
    message: str = ''


class BatteryController:
    """Handles battery operations, independent of the ROS 2 layer."""

    def __init__(
        self,
        robot_ip: str,
        timeout: float,
        logger: Optional[logging.Logger] = None,
    ):
        self.api = BatteryApiClient(robot_ip, timeout=timeout)
        self.logger = logger or logging.getLogger(__name__)

    def get_mobile_charging_system_status(self) -> MobileChargingSystemStatusResult:
        """Query wireless mobile charging system status."""
        success, data = self.api.get_mobile_charging_system_status()
        if not success:
            return MobileChargingSystemStatusResult(success=False, message=str(data))
        # API returns a bare JSON string enum value.
        if isinstance(data, str):
            return MobileChargingSystemStatusResult(success=True, status=data)
        return MobileChargingSystemStatusResult(
            success=False,
            message=f'Unexpected response type: {type(data).__name__}',
        )

    def get_system_states(self) -> SystemStatesResult:
        """Query aggregated battery / charging system states."""
        success, data = self.api.get_system_states()
        if not success:
            return SystemStatesResult(success=False, message=str(data))
        if not isinstance(data, dict):
            return SystemStatesResult(
                success=False,
                message=f'Unexpected response type: {type(data).__name__}',
            )

        temperatures = data.get('temperatures', {}) or {}
        errors = data.get('mob_charging_unit_errors', []) or []
        return SystemStatesResult(
            success=True,
            battery_level_rsoc=int(data.get('battery_level_rsoc', 0)),
            mob_charging_unit_errors=[str(e) for e in errors],
            temperatures=TemperaturesData(
                heatsink=float(temperatures.get('heatsink', 0.0) or float('nan')),
                coil_mobile=float(temperatures.get('coil_mobile', 0.0) or float('nan')),
                hf1_terminal=float(temperatures.get('hf1_terminal', 0.0) or float('nan')),
                batt_pos_terminal=float(temperatures.get('batt_pos_terminal', 0.0) or float('nan')),
            ),
            wireless_charging_active=bool(data.get('wireless_charging_active', False)),
            wired_charging_active=bool(data.get('wired_charging_active', False)),
        )

    def get_is_charging(self) -> IsChargingResult:
        """Query whether the battery is currently charging."""
        success, data = self.api.get_is_charging()
        if not success:
            return IsChargingResult(success=False, message=str(data))
        # API returns a bare JSON boolean (newtype BatterychargingState).
        if isinstance(data, bool):
            return IsChargingResult(success=True, is_charging=data)
        return IsChargingResult(
            success=False,
            message=f'Unexpected response type: {type(data).__name__}',
        )

    def get_system_info(self) -> SystemInfoResult:
        """Query charging system serial numbers and software versions."""
        success, data = self.api.get_system_info()
        if not success:
            return SystemInfoResult(success=False, message=str(data))
        if not isinstance(data, dict):
            return SystemInfoResult(
                success=False,
                message=f'Unexpected response type: {type(data).__name__}',
            )
        return SystemInfoResult(
            success=True,
            serial_number_mobile=int(data.get('serial_number_mobile', 0)),
            serial_number_station=int(data.get('serial_number_station', 0)),
            sw_version_mobile_revision=int(data.get('sw_version_mobile_revision', 0)),
            sw_version_mobile_major=int(data.get('sw_version_mobile_major', 0)),
            sw_version_mobile_minor=int(data.get('sw_version_mobile_minor', 0)),
        )

    def stop_wireless_charging(self) -> CommandResult:
        """Stop wireless charging."""
        success, data = self.api.stop_wireless_charging()
        if not success:
            return CommandResult(success=False, message=str(data))
        return CommandResult(success=True, message='Wireless charging stopped')

    def try_start_wireless_charging(self) -> CommandResult:
        """Attempt to start wireless charging."""
        success, data = self.api.try_start_wireless_charging()
        if not success:
            return CommandResult(success=False, message=str(data))
        return CommandResult(success=True, message='Wireless charging start requested')
