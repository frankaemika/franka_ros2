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

"""ROS 2 battery node for the Franka TMR Battery REST API."""

from franka_mobile.battery_controller import BatteryController
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_srvs.srv import Trigger

#: Default HTTP request timeout in seconds.
DEFAULT_HTTP_TIMEOUT = 20.0

#: Default battery topic publish rate in Hz.
DEFAULT_PUBLISH_RATE = 1.0


class BatteryServer(Node):
    """Publishes BatteryState and exposes Trigger services for wireless charging control."""

    def __init__(self):
        super().__init__('franka_battery_node')

        self.declare_parameter('robot_ip', '')
        self.declare_parameter('http_timeout', DEFAULT_HTTP_TIMEOUT)
        self.declare_parameter('publish_rate', DEFAULT_PUBLISH_RATE)
        self.declare_parameter('enable_battery_topic', True)

        robot_ip = self.get_parameter('robot_ip').get_parameter_value().string_value
        if not robot_ip:
            self.get_logger().fatal("Parameter 'robot_ip' not set")
            raise ValueError("Parameter 'robot_ip' not set")

        http_timeout = self.get_parameter('http_timeout').get_parameter_value().double_value
        publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        enable_battery_topic = (
            self.get_parameter('enable_battery_topic').get_parameter_value().bool_value
        )

        self.controller = BatteryController(
            robot_ip,
            timeout=http_timeout,
            logger=self.get_logger(),
        )
        self.get_logger().info(f'Battery controller initialized for {robot_ip}')

        reentrant_callback_group = ReentrantCallbackGroup()
        self.stop_wireless_charging_service = self.create_service(
            Trigger,
            '~/stop_wireless_charging',
            self._stop_wireless_charging_callback,
            callback_group=reentrant_callback_group,
        )
        self.try_start_wireless_charging_service = self.create_service(
            Trigger,
            '~/try_start_wireless_charging',
            self._try_start_wireless_charging_callback,
            callback_group=reentrant_callback_group,
        )

        self._battery_publisher = None
        self._publish_timer = None
        if enable_battery_topic:
            if publish_rate <= 0.0:
                self.get_logger().warn(
                    'publish_rate must be > 0 to enable the battery topic; topic disabled'
                )
            else:
                self._battery_publisher = self.create_publisher(BatteryState, '~/battery_state', 10)
                period = 1.0 / publish_rate
                self._publish_timer = self.create_timer(
                    period,
                    self._publish_battery_state,
                    callback_group=reentrant_callback_group,
                )
                self.get_logger().info(
                    f'Publishing sensor_msgs/BatteryState on ~/battery_state at {publish_rate} Hz'
                )

        self.get_logger().info('Battery server ready')

    def _stop_wireless_charging_callback(self, _request, response):
        result = self.controller.stop_wireless_charging()
        response.success = result.success
        response.message = result.message
        return response

    def _try_start_wireless_charging_callback(self, _request, response):
        result = self.controller.try_start_wireless_charging()
        response.success = result.success
        response.message = result.message
        return response

    def _to_battery_state(self, states) -> BatteryState:
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.percentage = float(states.battery_level_rsoc) / 100.0
        msg.present = True

        if states.wireless_charging_active or states.wired_charging_active:
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_CHARGING
        elif states.battery_level_rsoc >= 100:
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_FULL
        else:
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING

        if states.wireless_charging_active:
            msg.location = 'wireless'
        elif states.wired_charging_active:
            msg.location = 'wired'
        else:
            msg.location = ''

        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_UNKNOWN
        msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
        if states.mob_charging_unit_errors:
            msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_UNSPEC_FAILURE

        msg.voltage = float('nan')
        msg.temperature = states.temperatures.batt_pos_terminal
        msg.current = float('nan')
        msg.charge = float('nan')
        msg.capacity = float('nan')
        msg.design_capacity = float('nan')
        return msg

    def _publish_battery_state(self):
        """Poll system-states and publish a standard BatteryState at a low rate."""
        result = self.controller.get_system_states()
        if not result.success:
            self.get_logger().warn(
                f'Failed to publish battery state: {result.message}',
                throttle_duration_sec=5.0,
            )
            return

        self._battery_publisher.publish(self._to_battery_state(result))


def main(args=None):
    rclpy.init(args=args)
    node = BatteryServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

    print('Battery server shut down gracefully')


if __name__ == '__main__':
    main()
