#  Copyright (c) 2026 Franka Robotics GmbH
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

"""Gazebo integration test for the FR3 Duo launch."""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# Import the module (not the classes) so the shared base TestCases are NOT pulled
# into this module's namespace. launch_testing collects every unittest.TestCase in
# the module as an active (pre-shutdown) test; if GazeboShutdownTestBase were a
# module attribute it would run as an active test and its setUp would call
# ensure_gz_sim_not_running(), killing the live simulation mid-test.
import gazebo_test_utils  # noqa: E402

import launch_testing  # noqa: E402


EXPECTED_ARM_JOINTS = {
    f'{arm_prefix}_fr3v2_joint{joint_index}'
    for arm_prefix in ('left', 'right')
    for joint_index in range(1, 8)
}


def generate_test_description():
    """Launch the FR3 Duo Gazebo example in headless mode."""
    return gazebo_test_utils.make_gazebo_test_description('gazebo_fr3_duo_example.launch.py')


class TestGazeboFr3Duo(gazebo_test_utils.GazeboTestBase):
    """Verify the FR3 Duo Gazebo launch publishes both arm joint states."""

    NODE_NAME = 'gazebo_fr3_duo_test'
    EXPECTED_JOINTS = EXPECTED_ARM_JOINTS

    def test_publishes_dual_arm_joint_states(self):
        """Check that both FR3 Duo arms appear in /joint_states."""
        joint_state = self.wait_for_joint_states(min_joints=14)
        self.assertTrue(
            EXPECTED_ARM_JOINTS.issubset(set(joint_state.name)),
            f'Expected FR3 Duo arm joints not found. Got: {joint_state.name}',
        )


@launch_testing.post_shutdown_test()
class TestGazeboFr3DuoShutdown(gazebo_test_utils.GazeboShutdownTestBase):
    """Verify the FR3 Duo Gazebo launch exits without unexpected errors."""

    pass
