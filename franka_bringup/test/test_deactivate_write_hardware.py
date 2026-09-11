# Copyright 2026 Franka Robotics GmbH
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

"""
Pin the upstream ros2_control DEACTIVATE-from-write lifecycle contract.

This launch test loads ``hardware_interface_testing``'s ``TestSystem`` plugin
(``test_system``), not ``FrankaHardwareInterface``. It regression-tests
controller-manager behavior when a hardware ``write()`` returns ``DEACTIVATE``:
state interfaces remain available and ``joint_state_broadcaster`` stays active as
the representative state-only controller.

``franka_robot_state_broadcaster`` is intentionally omitted — ``TestSystem`` does
not export Franka's ``robot_state`` interface, so that broadcaster cannot be
activated against this fixture.
"""

import time
import unittest

from builtin_interfaces.msg import Duration
from controller_manager_msgs.srv import (
    ConfigureController,
    ListControllers,
    ListHardwareComponents,
    LoadController,
    SwitchController,
)

from launch import LaunchDescription
from launch_ros.actions import Node

import launch_testing
import launch_testing.actions

from lifecycle_msgs.msg import State
import rclpy
from std_msgs.msg import Float64MultiArray


COMMAND_CONTROLLER = 'deactivate_write_controller'
HARDWARE_COMPONENT = 'deactivating_hardware'
JOINT_STATE_BROADCASTER = 'joint_state_broadcaster'
STATE_INTERFACE_NAMES = {
    'test_joint/position',
    'test_joint/velocity',
    'test_joint/acceleration',
}
# Mirrors ros2_control_test_assets::test_constants::WRITE_DEACTIVATE_VALUE.
# TestSystem returns DEACTIVATE from write() when it receives this command value.
WRITE_DEACTIVATE_COMMAND = 24242424.0
CONTROLLER_MANAGER_NAME = 'controller_manager'
CONTROLLER_STATE_TIMEOUT = 10.0

# Minimal URDF for hardware_interface_testing/TestSystem — not a Franka hardware plugin.
ROBOT_DESCRIPTION = """
<robot name="write_deactivate_test_robot">
  <link name="base_link"/>
  <link name="test_link"/>
  <joint name="test_joint" type="continuous">
    <parent link="base_link"/>
    <child link="test_link"/>
    <axis xyz="0 0 1"/>
  </joint>
  <ros2_control name="deactivating_hardware" type="system">
    <hardware>
      <plugin>test_system</plugin>
    </hardware>
    <joint name="test_joint">
      <command_interface name="velocity"/>
      <command_interface name="acceleration"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <state_interface name="acceleration"/>
    </joint>
  </ros2_control>
</robot>
"""


def _controller_manager_parameters(deactivate_controllers):
    return {
        'robot_description': ROBOT_DESCRIPTION,
        'update_rate': 100,
        'defaults': {
            'deactivate_controllers_on_hardware_self_deactivate': deactivate_controllers,
        },
        JOINT_STATE_BROADCASTER: {
            'type': 'joint_state_broadcaster/JointStateBroadcaster',
        },
        COMMAND_CONTROLLER: {
            'type': 'forward_command_controller/ForwardCommandController',
        },
    }


def _command_controller_parameters():
    return {
        'joints': ['test_joint'],
        'interface_name': 'velocity',
    }


class ControllerManagerClient:
    """Minimal controller-manager client scoped to one test namespace."""

    def __init__(self, node, namespace):
        self._node = node
        manager_path = f'{namespace}/{CONTROLLER_MANAGER_NAME}'
        self._configure = node.create_client(
            ConfigureController, f'{manager_path}/configure_controller'
        )
        self._list_controllers = node.create_client(
            ListControllers, f'{manager_path}/list_controllers'
        )
        self._list_hardware = node.create_client(
            ListHardwareComponents, f'{manager_path}/list_hardware_components'
        )
        self._load = node.create_client(LoadController, f'{manager_path}/load_controller')
        self._switch = node.create_client(
            SwitchController, f'{manager_path}/switch_controller'
        )

    def wait_for_services(self):
        return all(
            client.wait_for_service(timeout_sec=CONTROLLER_STATE_TIMEOUT)
            for client in (
                self._configure,
                self._list_controllers,
                self._list_hardware,
                self._load,
                self._switch,
            )
        )

    def load_and_activate(self, controller_name):
        load_request = LoadController.Request()
        load_request.name = controller_name
        self._wait_for_success(self._load.call_async(load_request))

        configure_request = ConfigureController.Request()
        configure_request.name = controller_name
        self._wait_for_success(self._configure.call_async(configure_request))

        switch_request = SwitchController.Request()
        switch_request.activate_controllers = [controller_name]
        switch_request.strictness = SwitchController.Request.STRICT
        switch_request.activate_asap = True
        switch_request.timeout = Duration(sec=int(CONTROLLER_STATE_TIMEOUT))
        self._wait_for_success(self._switch.call_async(switch_request))

    def controller_state(self, controller_name):
        request = ListControllers.Request()
        response = self._wait_for_response(self._list_controllers.call_async(request))
        return next(
            controller.state
            for controller in response.controller
            if controller.name == controller_name
        )

    def hardware_component(self):
        request = ListHardwareComponents.Request()
        response = self._wait_for_response(self._list_hardware.call_async(request))
        return next(
            component
            for component in response.component
            if component.name == HARDWARE_COMPONENT
        )

    def _wait_for_success(self, future):
        response = self._wait_for_response(future)
        if response is None or not response.ok:
            raise RuntimeError('controller-manager service request failed')

    def _wait_for_response(self, future):
        rclpy.spin_until_future_complete(
            self._node, future, timeout_sec=CONTROLLER_STATE_TIMEOUT
        )
        if not future.done():
            raise RuntimeError('controller-manager service request timed out')
        return future.result()


class TestDeactivateWriteHardware(unittest.TestCase):
    """
    Verify TestSystem write-DEACTIVATE keeps state interfaces and JSB active.

    Exercises the upstream controller-manager self-deactivation contract with
    ``joint_state_broadcaster`` as the state-only controller under test.
    """

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('deactivate_write_hardware_test')

    def tearDown(self):
        self.node.destroy_node()

    def test_write_deactivate_preserves_state_interfaces_and_state_broadcaster(self):
        test_cases = {
            '/self_deactivate_enabled': True,
            '/self_deactivate_disabled': False,
        }

        for namespace, deactivate_controllers in test_cases.items():
            with self.subTest(
                deactivate_controllers_on_hardware_self_deactivate=deactivate_controllers
            ):
                self._verify_self_deactivation(namespace, deactivate_controllers)

    def _verify_self_deactivation(self, namespace, deactivate_controllers):
        client = ControllerManagerClient(self.node, namespace)
        self.assertTrue(client.wait_for_services())

        client.load_and_activate(JOINT_STATE_BROADCASTER)
        client.load_and_activate(COMMAND_CONTROLLER)

        publisher = self.node.create_publisher(
            Float64MultiArray, f'{namespace}/{COMMAND_CONTROLLER}/commands', 10
        )
        command = Float64MultiArray(data=[WRITE_DEACTIVATE_COMMAND])
        try:
            self._publish_until_hardware_inactive(client, publisher, command)
        finally:
            self.node.destroy_publisher(publisher)

        component = client.hardware_component()
        self.assertEqual(component.state.id, State.PRIMARY_STATE_INACTIVE)
        available_state_interfaces = {
            interface.name
            for interface in component.state_interfaces
            if interface.is_available
        }
        self.assertTrue(STATE_INTERFACE_NAMES.issubset(available_state_interfaces))
        self.assertEqual(client.controller_state(JOINT_STATE_BROADCASTER), 'active')

        expected_command_controller_state = (
            'inactive' if deactivate_controllers else 'active'
        )
        self.assertEqual(
            client.controller_state(COMMAND_CONTROLLER),
            expected_command_controller_state,
        )

    def _publish_until_hardware_inactive(self, client, publisher, command):
        deadline = time.monotonic() + CONTROLLER_STATE_TIMEOUT
        while time.monotonic() < deadline:
            publisher.publish(command)
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if client.hardware_component().state.id == State.PRIMARY_STATE_INACTIVE:
                return
        self.fail('hardware did not become inactive after write returned DEACTIVATE')


def generate_test_description():
    test_nodes = []
    for namespace, deactivate_controllers in (
        ('self_deactivate_enabled', True),
        ('self_deactivate_disabled', False),
    ):
        test_nodes.extend(
            [
                Node(
                    package='robot_state_publisher',
                    executable='robot_state_publisher',
                    namespace=namespace,
                    parameters=[{'robot_description': ROBOT_DESCRIPTION}],
                    output='screen',
                ),
                Node(
                    package='controller_manager',
                    executable='ros2_control_node',
                    namespace=namespace,
                    parameters=[
                        _controller_manager_parameters(deactivate_controllers),
                        _command_controller_parameters(),
                    ],
                    output='screen',
                ),
            ]
        )

    return LaunchDescription(
        [
            *test_nodes,
            launch_testing.actions.ReadyToTest(),
        ]
    )
