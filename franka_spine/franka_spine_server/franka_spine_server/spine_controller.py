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

"""Spine controller that encapsulates all spine business logic."""

from dataclasses import dataclass
import logging
import threading
import time
from typing import Any, Callable, Optional, Tuple

from franka_spine_server.spine_api import SpineMotion, SpineStatus
from franka_spine_server.spine_api_client import SpineApiClient


def _motion_payload_label(data: Any) -> str:
    """Normalize a start_motion body to a stop-reason string."""
    if isinstance(data, str):
        return data
    if isinstance(data, dict):
        return str(data.get('StopBy', ''))
    return str(data) if data is not None else ''


def _start_finished_at_target(start_result: list) -> bool:
    """True if start_motion returned HTTP OK with body Finished."""
    if not start_result:
        return False
    success, data = start_result[0]
    return bool(success) and _motion_payload_label(data) == SpineMotion.Finished


def _state_from_payload(data: Any) -> str:
    """Extract a DS402 state string from a REST payload."""
    if isinstance(data, str):
        return data
    if isinstance(data, dict):
        return str(data.get('state', ''))
    return str(data) if data is not None else ''


def _not_ready_to_move(state: str = '') -> str:
    """Error when motion-mm:start is refused because the drive is not on."""
    if state:
        return (
            f'Cannot start motion: spine is {state} '
            f'(expected {SpineStatus.SwitchedOn})'
        )
    return f'Cannot start motion: spine is not {SpineStatus.SwitchedOn}'


def _is_failed_dependency(data: Any) -> bool:
    """True if the device rejected the call with HTTP 424."""
    text = str(data)
    return '424' in text or 'Failed Dependency' in text


@dataclass
class MotionResult:
    """Result of a move operation."""

    success: bool
    stop_by: str = ''
    error: str = ''
    cancelled: bool = False


@dataclass
class _PollOutcome:
    """Internal result of polling while a start_motion call is in flight."""

    cancelled: bool = False
    error: str = ''


@dataclass
class StateResult:
    """Result of a state query."""

    success: bool
    state: str = ''
    error_code: str = ''
    error_description: str = ''


@dataclass
class PositionResult:
    """Result of a position query."""

    success: bool
    position: float = 0.0


@dataclass
class CommandResult:
    """Result of a simple command (switch on/off, fault reset, halt)."""

    success: bool
    state: str = ''
    message: str = ''


@dataclass
class UserLimitsData:
    """User-defined motion limits."""

    lower_limit: float = 0.0
    upper_limit: float = 0.0


@dataclass
class ParametersResult:
    """Result of a parameters query."""

    success: bool
    user_limits: Optional[UserLimitsData] = None
    message: str = ''


class SpineController:
    """Handles all spine operations, independent of the ROS 2 layer."""

    def __init__(
        self,
        spine_ip: str,
        timeout: float,
        feedback_rate: float = 10.0,
        logger: Optional[logging.Logger] = None,
    ):
        self.api = SpineApiClient(spine_ip, timeout=timeout)
        self.feedback_rate = feedback_rate
        self.logger = logger or logging.getLogger(__name__)
        self._motion_lock = threading.Lock()

    @property
    def motion_in_progress(self) -> bool:
        """Return True if a motion is currently running."""
        return self._motion_lock.locked()

    def get_state(self) -> StateResult:
        """Query the current spine state."""
        success, data = self.api.get_state()
        if not success:
            self.logger.error(f'get_state failed: {data}')
            return StateResult(success=False)

        result = StateResult(success=True)
        if isinstance(data, str):
            result.state = data
        elif isinstance(data, dict):
            result.state = data.get('state', '')
            error = data.get('error', {})
            if error:
                result.error_code = str(error.get('code', ''))
                result.error_description = error.get('description', '')
        return result

    def get_position(self) -> PositionResult:
        """Query the current position in metres."""
        success, data = self.api.get_position()
        if not success:
            self.logger.error(f'get_position failed: {data}')
            return PositionResult(success=False)
        return PositionResult(success=True, position=data.get('position', 0.0))

    def fault_reset(self) -> CommandResult:
        """Reset the spine from a fault state."""
        success, data = self.api.fault_reset()
        if not success:
            msg = f'Fault reset failed: {data}'
            self.logger.error(msg)
            return CommandResult(success=False, message=msg)
        state = data if isinstance(data, str) else str(data)
        return CommandResult(success=True, state=state, message='Fault reset successful')

    def switch_on(self) -> CommandResult:
        """Switch the spine on."""
        success, data = self.api.switch_on()
        if not success:
            msg = f'Switch on failed: {data}'
            self.logger.error(msg)
            return CommandResult(success=False, message=msg)
        state = data if isinstance(data, str) else str(data)
        return CommandResult(success=True, state=state, message='Switch on successful')

    def switch_off(self) -> CommandResult:
        """Switch the spine off."""
        success, data = self.api.switch_off()
        if not success:
            msg = f'Switch off failed: {data}'
            self.logger.error(msg)
            return CommandResult(success=False, message=msg)
        state = data if isinstance(data, str) else str(data)
        return CommandResult(success=True, state=state, message='Switch off successful')

    def halt(self) -> CommandResult:
        """Halt any ongoing motion and restore SwitchedOn (Halt.srv contract).

        The device only exposes DS402 quick-stop, which lands in SwitchedOff.
        After the drive leaves QuickStopActive, this re-arms with switch-on.
        """
        stop = self._quick_stop()
        if not stop.success:
            return stop
        rearm = self._rearm_after_quick_stop()
        if not rearm.success:
            return CommandResult(
                success=False,
                state=rearm.state,
                message=f'Halt stopped motion but failed to restore SwitchedOn: {rearm.message}',
            )
        return CommandResult(success=True, state=rearm.state, message='Halt successful')

    def get_parameters(self) -> ParametersResult:
        """Retrieve spine parameters (user limits)."""
        success, data = self.api.get_parameters()
        if not success:
            msg = f'Failed to get parameters: {data}'
            self.logger.error(msg)
            return ParametersResult(success=False, message=msg)

        user_limits_data = data.get('user_limits', {})
        return ParametersResult(
            success=True,
            user_limits=UserLimitsData(
                lower_limit=user_limits_data.get('lower_limit', 0.0),
                upper_limit=user_limits_data.get('upper_limit', 0.0),
            ),
            message='Parameters retrieved successfully',
        )

    def move_absolute(
        self,
        position: float,
        velocity: float,
        acceleration: float,
        deceleration: float,
        is_cancelled: Callable[[], bool] = lambda: False,
        on_feedback: Optional[Callable[[float], None]] = None,
        is_active: Callable[[], bool] = lambda: True,
    ) -> MotionResult:
        """
        Execute an absolute move and block until completion.
        Spine is expected to be in the ``SwitchedOn`` state before starting the motion.

        :param position: Target position in metres.
        :param velocity: Motion velocity in m/s.
        :param acceleration: Motion acceleration in m/s².
        :param deceleration: Motion deceleration in m/s².
        :param is_cancelled: Callable returning True when cancelled.
        :param on_feedback: Called with current position (m) each cycle.
        :param is_active: Callable returning True while system is running.
        """
        with self._motion_lock:
            ready = self.get_state()
            if not ready.success:
                return MotionResult(
                    success=False, error='Cannot start motion: failed to read spine state'
                )
            if ready.state != SpineStatus.SwitchedOn:
                msg = _not_ready_to_move(ready.state)
                self.logger.error(msg)
                return MotionResult(success=False, error=msg)

            motion_done = threading.Event()
            start_result = []

            def _start():
                try:
                    start_result.append(
                        self.api.start_motion(
                            position, velocity, acceleration, deceleration
                        )
                    )
                except Exception as exc:
                    start_result.append((False, str(exc)))
                finally:
                    motion_done.set()

            worker = threading.Thread(target=_start, name='spine-start-motion', daemon=True)
            worker.start()

            poll = self._poll_position_until_done(
                is_cancelled=is_cancelled,
                on_feedback=on_feedback,
                is_active=is_active,
                motion_done=motion_done,
            )

            worker.join()

            if poll.cancelled:
                if poll.error:
                    return MotionResult(success=False, error=poll.error, cancelled=False)
                # start may have returned Finished between the cancel check
                # and the quick-stop POST; do not re-arm or report cancel.
                if _start_finished_at_target(start_result):
                    return MotionResult(success=True, stop_by=SpineMotion.Finished)
                rearm = self._rearm_after_quick_stop()
                if not rearm.success:
                    return MotionResult(
                        success=False,
                        error=(
                            'Motion cancelled but failed to restore SwitchedOn: '
                            f'{rearm.message}'
                        ),
                        cancelled=False,
                    )
                return MotionResult(
                    success=False,
                    error='Motion cancelled',
                    cancelled=True,
                )

            if poll.error:
                return MotionResult(success=False, error=poll.error)

            if not start_result:
                return MotionResult(success=False, error='Motion start did not return')

            success, data = start_result[0]
            if not success:
                error = self._start_motion_error(data)
                self.logger.error(f'Failed to start motion: {error}')
                if not _is_failed_dependency(data):
                    self._quick_stop()
                return MotionResult(success=False, error=error)

            stop_by = _motion_payload_label(data)
            if stop_by != SpineMotion.Finished:
                return MotionResult(
                    success=False, stop_by=stop_by, error=f'Stopped by {stop_by}'
                )
            return MotionResult(success=True, stop_by=SpineMotion.Finished)

    def _start_motion_error(self, data: Any) -> str:
        """Turn a failed start_motion response into a user-facing error."""
        current = self.get_state()
        if current.success and current.state != SpineStatus.SwitchedOn:
            return _not_ready_to_move(current.state)
        if _is_failed_dependency(data):
            return _not_ready_to_move()
        return str(data)

    def _quick_stop(self) -> CommandResult:
        """POST motion:quick-stop. Does not re-arm."""
        success, data = self.api.halt_motion()
        if not success:
            msg = f'Halt failed: {data}'
            self.logger.error(msg)
            return CommandResult(success=False, message=msg)
        state = _state_from_payload(data)
        return CommandResult(success=True, state=state, message='Quick-stop successful')

    def _rearm_after_quick_stop(self) -> CommandResult:
        """Wait for SwitchedOff after quick-stop, then switch on."""
        reached, state = self._wait_for_state(SpineStatus.SwitchedOff)
        if not reached:
            msg = f'Halt did not reach SwitchedOff (state={state})'
            self.logger.error(msg)
            return CommandResult(success=False, state=state, message=msg)
        return self.switch_on()

    def _wait_for_state(self, expected: str) -> Tuple[bool, str]:
        """Poll get_state until ``expected`` or timeout.

        Returns (reached, last_state).
        """
        period = 1.0 / self.feedback_rate
        deadline = time.monotonic() + self.api.timeout
        last_state = ''
        while time.monotonic() < deadline:
            success, data = self.api.get_state()
            if success:
                last_state = _state_from_payload(data)
                if last_state == expected:
                    return True, last_state
                if last_state in SpineStatus.FAULT_STATES:
                    return False, last_state
            time.sleep(period)
        return False, last_state

    def _poll_position_until_done(
        self,
        is_cancelled: Callable[[], bool],
        on_feedback: Optional[Callable[[float], None]],
        is_active: Callable[[], bool],
        motion_done: threading.Event,
    ) -> _PollOutcome:
        """
        Publish feedback and watch for cancel until motion-mm:start returns.

        Returns a ``_PollOutcome``. ``cancelled`` is True only after a cancel
        request; ``error`` is set when halt failed or the spine faulted.
        """
        period = 1.0 / self.feedback_rate

        while is_active():
            if is_cancelled() and not motion_done.is_set():
                self.logger.info('Motion cancelled, sending halt')
                stop = self._quick_stop()
                if not stop.success:
                    return _PollOutcome(cancelled=True, error=stop.message)
                return _PollOutcome(cancelled=True)

            success, data = self.api.get_position()
            if success:
                current_position = data.get('position', 0.0)
                if on_feedback:
                    on_feedback(current_position)
            else:
                self.logger.warning(f'Failed to read position: {data}')

            state_success, state_data = self.api.get_state()
            if state_success:
                state = _state_from_payload(state_data)
                if state in SpineStatus.FAULT_STATES:
                    self.logger.error(f'Spine entered fault state: {state}')
                    return _PollOutcome(error=f'Spine entered fault state: {state}')

            if motion_done.is_set():
                return _PollOutcome()

            time.sleep(period)

        return _PollOutcome()
