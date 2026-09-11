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

"""Unit tests for spine halt, re-arm, and mid-move cancellation."""

import threading
import unittest
from unittest.mock import MagicMock, patch

from franka_spine_server.spine_api import SpineMotion, SpineStatus
from franka_spine_server.spine_api_client import SpineApiClient
from franka_spine_server.spine_controller import SpineController


def _make_controller():
    """Build a controller with a mocked SpineApiClient."""
    api = MagicMock()
    api.timeout = 1.0
    with patch(
        'franka_spine_server.spine_controller.SpineApiClient',
        return_value=api,
    ):
        controller = SpineController('172.16.16.10', timeout=1.0, feedback_rate=100.0)
    return controller, api


class TestHaltMotion(unittest.TestCase):
    """Halt path, Halt.srv re-arm, and move_absolute cancellation."""

    def test_halt_motion_posts_quick_stop(self):
        client = SpineApiClient('172.16.16.10', timeout=1.0)
        client.post = MagicMock(return_value=(True, SpineStatus.QuickStopActive))

        success, data = client.halt_motion()

        self.assertTrue(success)
        self.assertEqual(data, SpineStatus.QuickStopActive)
        client.post.assert_called_once_with('motion:quick-stop')

    def test_halt_rearms_to_switched_on(self):
        controller, api = _make_controller()
        api.halt_motion.return_value = (True, SpineStatus.QuickStopActive)
        api.get_state.return_value = (True, SpineStatus.SwitchedOff)
        api.switch_on.return_value = (True, SpineStatus.SwitchedOn)

        result = controller.halt()

        self.assertTrue(result.success)
        self.assertEqual(result.state, SpineStatus.SwitchedOn)
        self.assertEqual(result.message, 'Halt successful')
        api.halt_motion.assert_called_once()
        api.switch_on.assert_called_once()

    def test_halt_does_not_rearm_when_quick_stop_fails(self):
        controller, api = _make_controller()
        api.halt_motion.return_value = (
            False,
            '404 Client Error: Not Found for url: '
            'https://172.16.16.10/spine/api/motion:halt',
        )

        result = controller.halt()

        self.assertFalse(result.success)
        self.assertIn('404', result.message)
        api.switch_on.assert_not_called()

    def test_move_absolute_publishes_feedback_while_start_blocks(self):
        controller, api = _make_controller()
        started = threading.Event()
        saw_feedback = threading.Event()
        positions = []

        def blocking_start(*_args, **_kwargs):
            started.set()
            self.assertTrue(saw_feedback.wait(timeout=2.0))
            return True, SpineMotion.Finished

        api.start_motion.side_effect = blocking_start
        api.get_position.return_value = (True, {'position': 0.521})
        api.get_state.return_value = (True, SpineStatus.SwitchedOn)

        def on_feedback(position):
            positions.append(position)
            if started.is_set():
                saw_feedback.set()

        result = controller.move_absolute(0.45, 0.02, 0.2, 0.2, on_feedback=on_feedback)

        self.assertTrue(result.success)
        self.assertEqual(result.stop_by, SpineMotion.Finished)
        self.assertTrue(positions)
        self.assertTrue(started.is_set())

    def test_cancel_halts_while_start_motion_is_in_flight(self):
        controller, api = _make_controller()
        started = threading.Event()
        release = threading.Event()
        cancel = {'value': False}
        positions = []

        def blocking_start(*_args, **_kwargs):
            started.set()
            self.assertTrue(release.wait(timeout=2.0))
            return True, 'QuickStop'

        state = {'value': SpineStatus.SwitchedOn}

        def halt_motion():
            state['value'] = SpineStatus.SwitchedOff
            release.set()
            return True, SpineStatus.QuickStopActive

        api.start_motion.side_effect = blocking_start
        api.halt_motion.side_effect = halt_motion
        api.get_position.return_value = (True, {'position': 0.521})
        api.get_state.side_effect = lambda: (True, state['value'])
        api.switch_on.return_value = (True, SpineStatus.SwitchedOn)

        def on_feedback(position):
            positions.append(position)
            if started.is_set():
                cancel['value'] = True

        result = controller.move_absolute(
            0.372,
            0.03,
            0.2,
            0.2,
            is_cancelled=lambda: cancel['value'],
            on_feedback=on_feedback,
        )

        self.assertTrue(result.cancelled)
        self.assertFalse(result.success)
        self.assertEqual(result.error, 'Motion cancelled')
        self.assertTrue(positions)
        api.halt_motion.assert_called()
        api.switch_on.assert_called_once()

    def test_cancel_does_not_report_canceled_when_halt_fails(self):
        controller, api = _make_controller()
        started = threading.Event()
        release = threading.Event()
        cancel = {'value': False}

        def blocking_start(*_args, **_kwargs):
            started.set()
            self.assertTrue(release.wait(timeout=2.0))
            return True, SpineMotion.Finished

        def halt_motion():
            release.set()
            return False, '404 Client Error: Not Found'

        api.start_motion.side_effect = blocking_start
        api.halt_motion.side_effect = halt_motion
        api.get_position.return_value = (True, {'position': 0.5})
        api.get_state.return_value = (True, SpineStatus.SwitchedOn)

        def on_feedback(_position):
            if started.is_set():
                cancel['value'] = True

        result = controller.move_absolute(
            0.55,
            0.03,
            0.2,
            0.2,
            is_cancelled=lambda: cancel['value'],
            on_feedback=on_feedback,
        )

        self.assertFalse(result.cancelled)
        self.assertFalse(result.success)
        self.assertIn('404', result.error)
        api.switch_on.assert_not_called()

    def test_cancel_after_start_reaches_target_skips_rearm_and_succeeds(self):
        controller, api = _make_controller()
        started = threading.Event()
        entered_halt = threading.Event()
        cancel = {'value': False}

        def blocking_start(*_args, **_kwargs):
            started.set()
            self.assertTrue(entered_halt.wait(timeout=2.0))
            return True, SpineMotion.Finished

        def halt_motion():
            entered_halt.set()
            return True, SpineStatus.QuickStopActive

        api.start_motion.side_effect = blocking_start
        api.halt_motion.side_effect = halt_motion
        api.get_position.return_value = (True, {'position': 0.55})
        api.get_state.return_value = (True, SpineStatus.SwitchedOn)

        def on_feedback(_position):
            if started.is_set():
                cancel['value'] = True

        result = controller.move_absolute(
            0.55,
            0.03,
            0.2,
            0.2,
            is_cancelled=lambda: cancel['value'],
            on_feedback=on_feedback,
        )

        self.assertTrue(result.success)
        self.assertFalse(result.cancelled)
        self.assertEqual(result.stop_by, SpineMotion.Finished)
        api.halt_motion.assert_called()
        api.switch_on.assert_not_called()

    def test_cancel_rearm_failure_does_not_report_canceled(self):
        controller, api = _make_controller()
        started = threading.Event()
        release = threading.Event()
        cancel = {'value': False}

        def blocking_start(*_args, **_kwargs):
            started.set()
            self.assertTrue(release.wait(timeout=2.0))
            return True, 'QuickStop'

        state = {'value': SpineStatus.SwitchedOn}

        def halt_motion():
            state['value'] = SpineStatus.SwitchedOff
            release.set()
            return True, SpineStatus.QuickStopActive

        api.start_motion.side_effect = blocking_start
        api.halt_motion.side_effect = halt_motion
        api.get_position.return_value = (True, {'position': 0.521})
        api.get_state.side_effect = lambda: (True, state['value'])
        api.switch_on.return_value = (False, 'drive still off')

        def on_feedback(_position):
            if started.is_set():
                cancel['value'] = True

        result = controller.move_absolute(
            0.372,
            0.03,
            0.2,
            0.2,
            is_cancelled=lambda: cancel['value'],
            on_feedback=on_feedback,
        )

        self.assertFalse(result.cancelled)
        self.assertFalse(result.success)
        self.assertIn(SpineStatus.SwitchedOn, result.error)
        api.switch_on.assert_called_once()

    def test_move_absolute_rejects_when_switched_off(self):
        controller, api = _make_controller()
        api.get_state.return_value = (True, SpineStatus.SwitchedOff)

        result = controller.move_absolute(0.2, 0.01, 0.2, 0.2)

        self.assertFalse(result.success)
        self.assertIn(SpineStatus.SwitchedOff, result.error)
        self.assertIn(SpineStatus.SwitchedOn, result.error)
        self.assertNotIn('424', result.error)
        api.start_motion.assert_not_called()
        api.halt_motion.assert_not_called()

    def test_move_absolute_maps_424_without_http_details(self):
        controller, api = _make_controller()
        api.get_state.return_value = (True, SpineStatus.SwitchedOn)
        api.get_position.return_value = (True, {'position': 0.321})
        api.start_motion.return_value = (
            False,
            '424 Client Error: Failed Dependency for url: '
            'https://172.16.16.10/spine/api/motion-mm:start',
        )

        result = controller.move_absolute(0.2, 0.01, 0.2, 0.2)

        self.assertFalse(result.success)
        self.assertIn(SpineStatus.SwitchedOn, result.error)
        self.assertNotIn('424', result.error)
        api.halt_motion.assert_not_called()


if __name__ == '__main__':
    unittest.main()
