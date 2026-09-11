#  Copyright (c) 2026 Franka Robotics GmbH
#
#  Licensed under the Apache License, Version 2.0 (the 'License');
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an 'AS IS' BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.

"""
Gazebo contact-response test for the estimated external wrench.

Brings up an fr3 with the gravity_compensation_example_controller (which logs the
stiffness-frame external wrench K_F_ext_hat_K every 100 cycles), settles it, then
injects a known world-frame +Z force on fr3_link7 via Gazebo's ApplyLinkWrench
system and verifies the estimate reports the contact with the reaction-sign
convention: the reported external force opposes the applied push (a push in +x reads a
measured external force of -x), expressed in the end-effector / stiffness frame K as
K_F_ext_hat_K.

The base-frame push is rotated into K by O_R_EE^T, so the per-axis split depends on the
current orientation while the vector magnitude is invariant (a pure rotation preserves
norm). The primary oracle is therefore the force-triple magnitude: a +30 N base-Z push
must show up as ‖F‖ ~ 30 N in frame K regardless of pose. At the fr3 ready pose the
stiffness-frame Z-axis in base is ~(0.71, -0.71, 0.0), so the base +Z push lands almost
entirely in K-frame X (~-22 N) and Y (~+22 N) with near-zero Z (~-3 N) -- checked as
optional corroboration. The at-rest baseline (~0) is covered by the existing launch
suite; this exercises the loaded path the baseline cannot.
"""

import math
import os
import re
import statistics
import subprocess
import time
import unittest

from launch import (
    actions,
    launch_description_sources,
    LaunchDescription,
    substitutions,
)
import launch_ros.substitutions
import launch_testing
import launch_testing.actions
import rclpy

# Gazebo command-line tools live under the ROS vendor prefix and are not on PATH by
# default inside the test process.
GZ_TOOLS_BIN = '/opt/ros/jazzy/opt/gz_tools_vendor/bin'

# World name from empty.sdf and the link the spike characterized the estimate on.
WORLD_NAME = 'empty'
LINK_NAME = 'fr3_link7'

# Applied world-frame +Z force (N) and the acceptance band for the measured force.
# The controller logs the stiffness-frame wrench K_F_ext_hat_K, obtained by rotating the
# base-frame wrench into K via O_R_EE^T. A rotation preserves magnitude, so the primary
# oracle is the force-triple norm: a +30 N base-Z push must yield ‖F‖ ~ 30 N in frame K
# regardless of pose. The band is wide enough to absorb sim-dynamics oscillation around
# the applied magnitude while still catching a broken (non-norm-preserving) transform.
APPLIED_FORCE_Z = 30.0
CONTACT_FORCE_NORM_MIN = 25.0
CONTACT_FORCE_NORM_MAX = 35.0
# Per-axis corroboration (optional). At the fr3 ready pose the stiffness-frame Z-axis in
# base is ~(0.71, -0.71, 0.0), so under the reaction-sign convention (push +x -> read -x)
# the base +Z push splits into K-frame X ~ -22 N, Y ~ +22 N and Z ~ -3 N. These are the
# flip of the medians @tester measured on the live gz bench; the tolerance absorbs
# sim-dynamics spread while still pinning the rotation to the correct pose.
CONTACT_FORCE_X_EXPECTED = -22.0
CONTACT_FORCE_Y_EXPECTED = 22.0
CONTACT_FORCE_Z_EXPECTED = -3.0
CONTACT_PER_AXIS_TOLERANCE = 8.0
# At-rest tolerance (N): all axes must sit inside +/- this band with no load applied.
NEAR_ZERO_TOLERANCE = 3.0

# Number of trailing wrench samples to take the per-axis median over. A single tail
# sample can land on a sub-second overshoot; the median of the last few is robust to it.
WRENCH_MEDIAN_WINDOW = 10

# Seconds to let Gazebo, the controller_manager and the controller spin up and the
# arm settle at its home pose before the test body runs.
SPINUP_DURATION = 12.0
# Seconds to wait for the first logged wrench line before giving up.
FIRST_LOG_TIMEOUT = 30.0
# Seconds to let the injected force propagate and the estimate settle.
SETTLE_AFTER_INJECTION = 4.0

# Matches: External wrench (stiffness frame): F=[x, y, z] T=[...]
WRENCH_PATTERN = re.compile(
    r'External wrench.*?F=\[\s*(-?\d+\.?\d*),\s*(-?\d+\.?\d*),\s*(-?\d+\.?\d*)\]'
)


def ensure_gz_sim_not_running():
    """
    Kill any remaining Gazebo and related ROS processes between tests.

    On CI, gz sim and the controller_manager may not shut down in time between
    tests, causing the next test to fail because controllers are still active.
    See https://github.com/ros2/launch/issues/545 for details.
    """
    subprocess.run(['pkill', '-2', '-f', '^gz sim'], check=False)
    time.sleep(2)
    subprocess.run(['pkill', '-9', '-f', '^gz sim'], check=False)
    subprocess.run(['pkill', '-9', '-f', 'ruby.*gz'], check=False)
    subprocess.run(['pkill', '-9', '-f', 'controller_manager'], check=False)
    subprocess.run(['pkill', '-9', '-f', 'robot_state_publisher'], check=False)
    time.sleep(2)


def gz_environment():
    """Return an environment with the Gazebo tools prepended to PATH."""
    environment = dict(os.environ)
    environment['PATH'] = GZ_TOOLS_BIN + os.pathsep + environment.get('PATH', '')
    return environment


def load_apply_link_wrench_system():
    """Load the ApplyLinkWrench system into the running world at runtime."""
    request = (
        'entity: {{ name: "{world}", type: WORLD }}, '
        'plugins: {{ filename: "gz-sim-apply-link-wrench-system", '
        'name: "gz::sim::systems::ApplyLinkWrench" }}'
    ).format(world=WORLD_NAME)
    subprocess.run(
        [
            'gz', 'service', '-s', '/world/{}/entity/system/add'.format(WORLD_NAME),
            '--reqtype', 'gz.msgs.EntityPlugin_V',
            '--reptype', 'gz.msgs.Boolean',
            '--req', request,
            '--timeout', '5000',
        ],
        env=gz_environment(),
        check=True,
    )


def apply_persistent_force_z(force_z):
    """Publish a persistent world-frame +Z force on the target link."""
    message = (
        'entity: {{ name: "{link}", type: LINK }}, '
        'wrench: {{ force: {{ x: 0, y: 0, z: {force} }} }}'
    ).format(link=LINK_NAME, force=force_z)
    subprocess.run(
        [
            'gz', 'topic', '-t', '/world/{}/wrench/persistent'.format(WORLD_NAME),
            '-m', 'gz.msgs.EntityWrench',
            '-p', message,
        ],
        env=gz_environment(),
        check=True,
    )


def event_text(output_event):
    """Decode a captured process IO event to text."""
    text = getattr(output_event, 'text', None)
    if text is None:
        return str(output_event)
    if isinstance(text, bytes):
        return text.decode('utf-8', errors='replace')
    return str(text)


def median_wrench_force(proc_output):
    """
    Return the per-axis median of the most recent logged wrench forces, or None.

    Reading the single last sample can catch a sub-second overshoot in the sim
    dynamics; the median of the trailing window rejects those transients and reflects
    the steady-state estimate.
    """
    samples = []
    for output_event in proc_output:
        for match in WRENCH_PATTERN.finditer(event_text(output_event)):
            samples.append(
                (
                    float(match.group(1)),
                    float(match.group(2)),
                    float(match.group(3)),
                )
            )
    if not samples:
        return None
    window = samples[-WRENCH_MEDIAN_WINDOW:]
    return (
        statistics.median(axis_x for axis_x, _, _ in window),
        statistics.median(axis_y for _, axis_y, _ in window),
        statistics.median(axis_z for _, _, axis_z in window),
    )


def generate_test_description():
    """Bring up fr3 with the gravity_compensation controller in an empty world."""
    ensure_gz_sim_not_running()

    launch_description = actions.IncludeLaunchDescription(
        launch_description_sources.PythonLaunchDescriptionSource(
            substitutions.PathJoinSubstitution(
                [
                    launch_ros.substitutions.FindPackageShare(
                        'franka_gazebo_bringup'
                    ),
                    'launch',
                    'gazebo_franka_arm_example_controller.launch.py',
                ]
            )
        ),
        launch_arguments={
            'robot_type': 'fr3',
            'controller': 'gravity_compensation_example_controller',
            'gz_args': 'empty.sdf -r -s --headless-rendering',
            'rviz': 'false',
        }.items(),
    )

    test_description = (
        LaunchDescription(
            [
                launch_description,
                actions.TimerAction(
                    period=SPINUP_DURATION,
                    actions=[launch_testing.actions.ReadyToTest()],
                ),
            ],
        ),
        {'launch_description': launch_description},
    )
    return test_description


class TestGazeboContactWrench(unittest.TestCase):
    """Verify the estimated external wrench reports an injected contact force."""

    @classmethod
    def setUpClass(cls):
        """Initialize the ROS context."""
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """Shutdown the ROS context."""
        rclpy.shutdown()
        ensure_gz_sim_not_running()

    def test_contact_response(self, proc_output):
        """At rest the estimate is ~0; a base +Z push conserves ‖F‖ ~ 30 N in frame K."""
        # The controller must be active and logging before we trust any reading.
        # rclcpp loggers write to stderr; the baseline and contact reads below
        # consume the whole proc_output, so this gate must watch stderr too.
        assert proc_output.waitFor(
            'External wrench', timeout=FIRST_LOG_TIMEOUT, stream='stderr'
        ), 'Controller never logged an external wrench'

        at_rest = median_wrench_force(proc_output)
        assert at_rest is not None, 'No external-wrench reading parsed at rest'
        for axis_value in at_rest:
            assert abs(axis_value) <= NEAR_ZERO_TOLERANCE, (
                'At-rest external wrench not near zero: {}'.format(at_rest)
            )

        load_apply_link_wrench_system()
        time.sleep(2.0)
        apply_persistent_force_z(APPLIED_FORCE_Z)
        time.sleep(SETTLE_AFTER_INJECTION)

        under_contact = median_wrench_force(proc_output)
        assert under_contact is not None, 'No external-wrench reading parsed under load'

        force_x, force_y, force_z = under_contact

        # Primary oracle: the base-frame push is rotated into K by O_R_EE^T, which
        # preserves magnitude. A +Z base push of APPLIED_FORCE_Z must therefore show up
        # as ‖F‖ ~ APPLIED_FORCE_Z in frame K, independent of the arm's pose.
        force_norm = math.sqrt(force_x**2 + force_y**2 + force_z**2)
        assert CONTACT_FORCE_NORM_MIN <= force_norm <= CONTACT_FORCE_NORM_MAX, (
            'K-frame force magnitude {} N outside [{}, {}] N under a +{} N base-Z push'
            .format(
                force_norm, CONTACT_FORCE_NORM_MIN, CONTACT_FORCE_NORM_MAX,
                APPLIED_FORCE_Z,
            )
        )

        # Optional corroboration: pin the rotation to the fr3 ready pose, where the
        # reaction to the push splits into K-frame (-22, +22, -3) N (flip of @tester's
        # live measurement under the reaction-sign convention).
        for axis_label, measured, expected in (
            ('X', force_x, CONTACT_FORCE_X_EXPECTED),
            ('Y', force_y, CONTACT_FORCE_Y_EXPECTED),
            ('Z', force_z, CONTACT_FORCE_Z_EXPECTED),
        ):
            assert abs(measured - expected) <= CONTACT_PER_AXIS_TOLERANCE, (
                'K-frame {} force {} N differs from expected {} N by more than +/- {} N'
                .format(axis_label, measured, expected, CONTACT_PER_AXIS_TOLERANCE)
            )
