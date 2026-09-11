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

"""String literals from the Franka Spine REST API."""


class SpineStatus:
    """DS402 states from GET /spine/api/state."""

    NotReadyToSwitchOn = 'NotReadyToSwitchOn'
    SwitchedOff = 'SwitchedOff'
    ReadyToSwitchOn = 'ReadyToSwitchOn'
    MotorOff = 'MotorOff'
    SwitchedOn = 'SwitchedOn'
    QuickStopActive = 'QuickStopActive'
    FaultReactionActive = 'FaultReactionActive'
    Fault = 'Fault'
    Unknown = 'Unknown'

    FAULT_STATES = (Fault, FaultReactionActive)


class SpineMotion:
    """Bodies returned by POST /spine/api/motion-mm:start."""

    Finished = 'Finished'
