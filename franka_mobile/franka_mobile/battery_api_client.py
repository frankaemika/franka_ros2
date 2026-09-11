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

"""HTTP client for the Franka TMR Battery REST API."""

from typing import Any, Tuple

from franka_desk_api._rest_client import _FrankaRestClient


class BatteryApiClient(_FrankaRestClient):
    """Client for the Franka TMR Battery REST API."""

    def __init__(self, robot_ip: str, timeout: float):
        """
        Initialize the Battery API client.

        :param robot_ip: IP address or hostname of the TMR robot.
        :param timeout: HTTP request timeout in seconds.
        """
        super().__init__(robot_ip, 'battery/api', timeout)

    def get_mobile_charging_system_status(self) -> Tuple[bool, Any]:
        """GET /battery/api/mobile-charging-system-status."""
        return self.get('mobile-charging-system-status')

    def get_system_states(self) -> Tuple[bool, Any]:
        """GET /battery/api/system-states."""
        return self.get('system-states')

    def get_is_charging(self) -> Tuple[bool, Any]:
        """GET /battery/api/is-charging."""
        return self.get('is-charging')

    def get_system_info(self) -> Tuple[bool, Any]:
        """GET /battery/api/system-info."""
        return self.get('system-info')

    def stop_wireless_charging(self) -> Tuple[bool, Any]:
        """POST /battery/api/wireless-charging:stop."""
        return self.post('wireless-charging:stop')

    def try_start_wireless_charging(self) -> Tuple[bool, Any]:
        """POST /battery/api/wireless-charging:try-start."""
        return self.post('wireless-charging:try-start')
