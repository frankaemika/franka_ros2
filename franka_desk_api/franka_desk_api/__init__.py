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

"""Internal package — not part of the public franka_ros2 API.

Downstream packages (``franka_mobile``, ``franka_spine_server``) may import
private modules such as ``franka_desk_api._rest_client``. External users must
not depend on this package; it may be replaced or removed without notice.
"""

__all__: list[str] = []
