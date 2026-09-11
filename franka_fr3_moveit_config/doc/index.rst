franka_fr3_moveit_config
========================

This package contains the MoveIt 2 configuration for the FR3 robot.

Move Group
----------

There is one move group called ``(robot_type)_arm``, where ``(robot_type)`` is the robot
type selected by the user.

The TCP placement depends on the end-effector configuration. If the Franka Hand is
loaded, the TCP is located between the fingertips and its axes are parallel to the
gripper frame. Otherwise, the TCP is located at the robot flange.

.. figure:: ../../docs/assets/move_groups.png
    :align: center
    :figclass: align-center

    Visualization of the different TCP placements.

Usage
-----

To start MoveIt on a real robot:

.. code-block:: shell

    ros2 launch franka_fr3_moveit_config moveit.launch.py robot_ip:=<fci-ip>

Then activate the ``MotionPlanning`` display in RViz.

To test the setup without a robot, start MoveIt with fake hardware:

.. code-block:: shell

    ros2 launch franka_fr3_moveit_config moveit.launch.py robot_ip:=dont-care use_fake_hardware:=true

Wait until MoveIt prints the green ``You can start planning now!`` message. Then toggle
``PlanningScene`` off and on again, and enable ``MotionPlanning``.

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
    * - ``namespace``
      - string
      - ``''``
      - Namespace for the robot.
    * - ``load_gripper``
      - bool
      - ``true``
      - Whether to load the gripper or not (true or false).
    * - ``ee_id``
      - string
      - ``franka_hand``
      - End-effector ID to use. Available options: ``none``, ``franka_hand``, ``cobot_pump``.
    * - ``use_fake_hardware``
      - bool
      - ``false``
      - Use fake hardware.
    * - ``fake_sensor_commands``
      - bool
      - ``false``
      - Fake sensor commands. Only valid when ``use_fake_hardware`` is ``true``.
    * - ``db``
      - bool
      - ``False``
      - Database flag.

Configuration Files
-------------------

This package includes:

* Motion planning configuration for the FR3 robot
* Joint limits and safety settings
* Planning groups and link configurations
* Kinematics solver configuration (kinematics.yaml)

For the Joint Impedance With IK example controller, you can change the kinematic solver
in this package's ``kinematics.yaml`` file.
