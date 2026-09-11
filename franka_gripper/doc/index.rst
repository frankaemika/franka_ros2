franka_gripper
==============

This package provides the ``franka_gripper_node`` for the ``Franka Hand``.

Actions and Services
--------------------

The ``franka_gripper_node`` provides the following actions:

* ``homing`` - homes the gripper and updates the maximum width given the mounted fingers.
* ``move`` - moves to a target width with the defined speed.
* ``grasp`` - tries to grasp at the desired width with the desired force while closing
  with the given speed. The operation succeeds if the distance ``d`` between the gripper
  fingers is ``width - epsilon.inner < d < width + epsilon.outer``.
* ``gripper_action`` - a special grasping action for MoveIt.

The node also provides a ``stop`` service that aborts gripper actions and stops grasping.

Usage
-----

Start the gripper with:

.. code-block:: shell

    ros2 launch franka_gripper gripper.launch.py robot_ip:=<fci-ip>

Launch arguments
----------------

.. list-table::
    :header-rows: 1

    * - Name
      - Type
      - Default
      - Description
    * - ``robot_ip``
      - string
      - required
      - Hostname or IP address of the robot.
    * - ``use_fake_hardware``
      - bool
      - ``false``
      - Publish fake gripper joint states without connecting to a real gripper.
    * - ``robot_type``
      - string
      - ``fr3``
      - Name of the arm in the URDF file. This is used to generate the gripper joint names.
    * - ``namespace``
      - string
      - ``''``
      - Namespace for the gripper nodes. If empty, the nodes are not namespaced.

The example commands below assume the default ``robot_type:=fr3`` and no namespace.

Open a second terminal and send the actions:

.. code-block:: shell

    ros2 action send_goal /fr3_gripper/homing franka_msgs/action/Homing {}
    ros2 action send_goal -f /fr3_gripper/grasp franka_msgs/action/Grasp "{width: 0.00, speed: 0.03, force: 50}"

The default values for ``epsilon.inner`` and ``epsilon.outer`` are 0.005 m. To set them
explicitly:

.. code-block:: shell

    ros2 action send_goal -f /fr3_gripper/grasp franka_msgs/action/Grasp "{width: 0.00, speed: 0.03, force: 50, epsilon: {inner: 0.01, outer: 0.01}}"

To stop grasping, call the ``stop`` service:

.. code-block:: shell

    ros2 service call /fr3_gripper/stop std_srvs/srv/Trigger {}

Changes from franka_ros
-----------------------

* All topics and actions were previously prefixed with ``franka_gripper``. They are now
  prefixed with ``<robot_type>_gripper`` (for example ``fr3_gripper`` by default), which
  makes multi-arm setups easier to configure.

* The ``stop`` action is now a service because it is not preemptable.

* All actions except ``gripper_action`` now provide the current gripper width as feedback.