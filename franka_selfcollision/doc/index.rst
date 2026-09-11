franka_selfcollision
====================

This package contains the library and the node for self-collision checking of the FR3 Duo and
Mobile FR3 Duo robots.

.. important::

    Minimum necessary `franka_description` version is 2.3.2.
    You can clone franka_description package from https://github.com/frankarobotics/franka_description.

Functionality
-------------

This monitoring node is launched by ``fr3_duo.launch.py`` and
``mobile_fr3_duo.launch.py`` in ``franka_bringup`` for the FR3 Duo and Mobile FR3 Duo
configurations. In the current launch setup, the node is always started for these
configurations.

The node continuously monitors the robot joint states for self-collisions between robot links.
It performs two main actions when it detects a collision, or when the security margin is
violated:

1. **Publishes Status:** Sends a boolean to the topic ``~/<node_name>/collision_detected``
   (where ``<node_name>`` is set via the ``name`` parameter in the launch file, default: ``self_collision_node``).
   The topic uses **best effort** QoS (``rclcpp::SensorDataQoS``). Subscribers must use a
   compatible QoS profile (e.g. ``rclcpp::SensorDataQoS()`` in C++ or
   ``qos_profile_sensor_data`` in Python) to receive messages.
2. **Logs Warning:** Prints the specific colliding link pairs to the console if enabled (throttled to 1Hz to prevent spam).

Configuration
-------------

Parameters are defined in ``config/self_collision_node.yaml``:

* ``security_margin``: Safety buffer around the robot links in meters (default: ``0.045``).
* ``print_collisions``: If ``true``, logs the names of the colliding links to the console.
* ``robot_description_semantic``: SRDF XML used to exclude allowable collision pairs.

Usage
-----

The self-collision node starts automatically when you launch either dual-arm configuration. No
extra launch argument is required.

.. code-block:: shell

    # FR3 Duo
    ros2 launch franka_bringup fr3_duo.launch.py \
        controller_name:=fr3_duo_joint_impedance_example_controller

    # Mobile FR3 Duo
    ros2 launch franka_bringup mobile_fr3_duo.launch.py \
        controller_name:=mobile_fr3_duo_joint_impedance_example_controller