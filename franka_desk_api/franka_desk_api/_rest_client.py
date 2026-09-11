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

"""Internal HTTPS REST client for Franka device APIs.

This module is an implementation detail of franka_ros2 and is not part of the
public API. It may change or be removed without notice.
"""

import threading
from typing import Any, Dict, Optional, Tuple

import requests
import urllib3

# Robot Desk certificates are typically not in the system trust store; callers
# connect over a trusted robot LAN. Suppress only the matching urllib3 warning.
urllib3.disable_warnings(urllib3.exceptions.InsecureRequestWarning)


class _FrankaRestClient:
    """Minimal HTTPS JSON client used by Franka device REST APIs.

    Builds URLs as ``https://{host}/{api_path}/{endpoint}`` with certificate
    verification disabled (same behaviour as Desk when opened by IP).

    ``requests.Session`` is not thread-safe. Each calling thread gets its own
    session via ``threading.local``.
    """

    def __init__(self, host: str, api_path: str, timeout: float):
        """
        :param host: IP address or hostname of the device.
        :param api_path: API path prefix without leading/trailing slashes,
            e.g. ``spine/api`` or ``battery/api``.
        :param timeout: HTTP request timeout in seconds.
        """
        api_path = api_path.strip('/')
        self.base_url = f'https://{host}/{api_path}'
        self.timeout = timeout
        self._local = threading.local()

    @property
    def session(self) -> requests.Session:
        """Return the ``requests.Session`` for the calling thread."""
        session = getattr(self._local, 'session', None)
        if session is None:
            session = requests.Session()
            session.headers.update({'Content-Type': 'application/json'})
            session.verify = False
            self._local.session = session
        return session

    def get(self, endpoint: str) -> Tuple[bool, Any]:
        """
        Perform a GET request.

        :return: Tuple of (success, response_data_or_error_message).
            ``response_data`` is ``None`` when the body is empty.
        """
        try:
            response = self.session.get(
                f'{self.base_url}/{endpoint.lstrip("/")}',
                timeout=self.timeout,
            )
            response.raise_for_status()
            if response.content:
                return True, response.json()
            return True, None
        except requests.exceptions.RequestException as e:
            return False, str(e)

    def post(
        self,
        endpoint: str,
        data: Optional[Dict] = None,
        timeout: Optional[Any] = None,
    ) -> Tuple[bool, Any]:
        """
        Perform a POST request.

        :param timeout: Overrides the client default. A ``(connect, read)``
            tuple sets the two phases separately.
        :return: Tuple of (success, response_data_or_error_message).
            ``response_data`` is ``None`` when the body is empty.
        """
        try:
            response = self.session.post(
                f'{self.base_url}/{endpoint.lstrip("/")}',
                json=data,
                timeout=self.timeout if timeout is None else timeout,
            )
            response.raise_for_status()
            if response.content:
                return True, response.json()
            return True, None
        except requests.exceptions.RequestException as e:
            return False, str(e)
