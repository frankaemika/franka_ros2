franka_bringup
==============

Installation
------------

Please refer to the `README.md <https://github.com/frankarobotics/franka_ros2/blob/jazzy/README.md>`_

Package Overview
----------------

This package contains the main launch files for bringing up a Franka robot, loading
example controllers, and starting multi-robot setups.

To start a single robot without any example controller, run:

.. code-block:: shell

    ros2 launch franka_bringup franka.launch.py robot_type:=fr3 robot_ip:=172.16.0.3

Only the ``joint_state_broadcaster`` is started. The connection to the robot is
still established and the current robot pose is visualized in RViz. In this mode
the robot can be guided when the user-stop button is pressed. Once a controller
that uses the ``effort_command_interface`` is started, the robot switches to the
libfranka torque interface. For example, you can start the
``gravity_compensation_example_controller`` with:

.. code-block:: shell

    ros2 control load_controller --set-state active gravity_compensation_example_controller

This is the equivalent of running the ``gravity_compensation_example_controller`` example mentioned in
:doc:`Gravity Compensation <../../franka_example_controllers/doc/index>`.

To stop the controller again, run:

.. code-block:: shell

    ros2 control set_controller_state gravity_compensation_example_controller inactive

The robot then stops torque control and only publishes its current state over FCI.

You can restart the same controller with:

.. code-block:: shell

    ros2 control set_controller_state gravity_compensation_example_controller active

or load and start a different one:

.. code-block:: shell

    ros2 control load_controller --set-state active joint_impedance_example_controller

``franka.launch.py`` launch arguments
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. list-table::
    :header-rows: 1

    * - Name
      - Type
      - Default
      - Description
    * - ``robot_type``
      - string
      - ``''``
      - Robot model identifier passed to the URDF xacro, for example ``fr3``.
    * - ``arm_prefix``
      - string
      - ``''``
      - Prefix added to arm-specific topic and joint names.
    * - ``namespace``
      - string
      - ``''``
      - ROS namespace applied to the robot nodes and services.
    * - ``robot_ip``
      - string
      - ``172.16.0.3``
      - Hostname or IP address of the robot.
    * - ``load_gripper``
      - bool
      - ``false``
      - Load the Franka Gripper launch file.
    * - ``use_fake_hardware``
      - bool
      - ``false``
      - Start ``ros2_control`` with fake hardware instead of a real robot.
    * - ``fake_sensor_commands``
      - bool
      - ``false``
      - Enable fake sensor command interfaces when fake hardware is used.
    * - ``joint_state_rate``
      - integer
      - ``30``
      - Joint state publisher rate in Hz.
    * - ``load_franka_robot_state_broadcaster``
      - bool
      - ``true``
      - Load ``franka_robot_state_broadcaster``. Set this to ``false`` for robots
        that do not support it, such as TMR.
    * - ``controllers_yaml``
      - string
      - ``franka_bringup/config/controllers.yaml``
      - Override the controller configuration file passed to ``ros2_control_node``.


Namespace-enabled launch files
------------------------------

Use ``franka_bringup/launch/example.launch.py`` when you want to start one or more
single-robot setups from a YAML file.

By default, ``example.launch.py`` reads ``franka_bringup/config/franka.config.yaml``.
Each top-level YAML entry describes one robot. Namespacing is handled by the
``namespace`` key in that config file, which is passed through to
``franka.launch.py``. If you want to launch a single robot directly, you can also
set the ``namespace`` launch argument on ``franka.launch.py`` itself.

For example, this command starts a single robot in the ``franka1`` namespace:

.. code-block:: shell

    ros2 launch franka_bringup franka.launch.py \
        robot_type:=fr3 \
        robot_ip:=172.16.0.3 \
        namespace:=franka1

``example.launch.py`` launch arguments
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. list-table::
    :header-rows: 1

    * - Name
      - Type
      - Default
      - Description
    * - ``robot_config_file``
      - string
      - ``franka.config.yaml``
      - Config file name looked up in ``franka_bringup/config/`` or an absolute path.
    * - ``controller_names``
      - string
      - none
      - Comma-separated controller names to spawn. This argument is required.
    * - ``robot_ips``
      - string
      - ``''``
      - Optional comma-separated IP addresses that override ``robot_ip`` values in
        the config file.

Update ``franka_bringup/config/franka.config.yaml`` with your robot IP address and,
if needed, a unique ``namespace`` for each robot. Then start an example controller
with:

.. code-block:: shell

    ros2 launch franka_bringup example.launch.py controller_names:=move_to_start_example_controller

Further information about namespaces in ROS 2 is available in the
`ROS 2 documentation <https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Launch/Using-ROS2-Launch-For-Large-Projects.html#namespaces>`_.

FR3 Duo
------------------------

The ``franka_bringup`` package supports launching a FR3 Duo setup using the ``fr3_duo.launch.py``
launch file with the ``fr3_duo.config.yaml`` configuration file.

.. important::

    The FR3 Duo setup currently only supports the **torque (effort) command interface**. Position, velocity,
    and Cartesian pose/velocity interfaces are not supported for dual-arm configurations.

Configuration
^^^^^^^^^^^^^

``fr3_duo.launch.py`` accepts three CLI launch arguments. The long header comment at
the top of the file also lists keys from the YAML config file; those are not
additional ``ros2 launch`` command-line arguments.

``fr3_duo.launch.py`` launch arguments
""""""""""""""""""""""""""""""""""""""

.. list-table::
    :header-rows: 1

    * - Name
      - Type
      - Default
      - Description
    * - ``robot_config_file``
      - string
      - ``franka_bringup/config/fr3_duo.config.yaml``
      - Config file name looked up in ``franka_bringup/config/`` or an absolute path.
    * - ``controllers_yaml``
      - string
      - ``franka_bringup/config/controllers.yaml``
      - Override the controller definition file passed to ``ros2_control_node``.
    * - ``controller_name``
      - string
      - none
      - Controller to spawn. This argument is required.

The dual-arm configuration is defined in ``franka_bringup/config/fr3_duo.config.yaml``.
The main config file keys are:

.. list-table::
    :header-rows: 1

    * - Key
      - Type
      - Example
      - Description
    * - ``robot_types``
      - stringified list
      - ``"['fr3v2','fr3v2']"``
      - Robot model identifiers for the two arms.
    * - ``arm_prefixes``
      - stringified list
      - ``"['left','right']"``
      - Unique prefixes used for the two arms.
    * - ``robot_ips``
      - stringified list
      - ``"['172.16.16.11','172.16.16.12']"``
      - FCI IP addresses of the two robots.
    * - ``load_gripper``
      - bool
      - ``"false"``
      - Enable the gripper launch integration.
    * - ``use_fake_hardware``
      - bool
      - ``"false"``
      - Use fake hardware instead of real robots.
    * - ``fake_sensor_commands``
      - bool
      - ``"false"``
      - Enable fake sensor commands when fake hardware is used.
    * - ``namespace``
      - string
      - ``""``
      - ROS namespace applied to the whole dual-arm setup.
    * - ``joint_state_rate``
      - integer
      - ``1000``
      - Joint state publisher rate in Hz.
    * - ``use_rviz``
      - bool
      - ``"true"``
      - Start RViz together with the launch file.
    * - ``thread_priority``
      - integer
      - ``98``
      - Thread priority used by the hardware interface. Must stay above the controller
        manager's ``thread_priority``, since this thread owns the 1 kHz robot socket.

.. note::

    All three arrays (``robot_types``, ``robot_ips``, ``arm_prefixes``) must have the same length,
    and ``arm_prefixes`` must contain unique values.

    The self-collision node is started automatically by ``fr3_duo.launch.py``.
    There is no ``check_selfcollision`` config key in ``fr3_duo.config.yaml``.

Launching the FR3 Duo
^^^^^^^^^^^^^^^^^^^^^


To launch the dual-arm setup with the joint impedance controller using a config file:

.. code-block:: shell

    ros2 launch franka_bringup fr3_duo.launch.py \
        robot_config_file:=fr3_duo.config.yaml \
        controller_name:=fr3_duo_joint_impedance_example_controller

You can also specify just the config filename, and the launch file will automatically look in the
``franka_bringup/config/`` directory.

.. note::

    The FR3 Duo setup supports only **one controller** at a time using the ``controller_name`` parameter (singular),
    unlike ``example.launch.py`` which supports multiple controllers with ``controller_names`` (plural).

Mobile FR3 Duo
------------------------

The ``franka_bringup`` package supports launching a Mobile FR3 Duo setup (TMRv0.2 mobile base with dual FR3 arms)
using the ``mobile_fr3_duo.launch.py`` launch file with the ``mobile_fr3_duo.config.yaml`` configuration file.

.. important::

    The Mobile FR3 Duo setup combines:

    * **Dual FR3 arms**: Controlled via joint impedance using the torque (effort) command interface
    * **Mobile base**: Controlled via Cartesian velocity using GPIO interfaces

    Like the FR3 Duo, this setup currently only supports the **torque (effort) command interface** for the arms.

Configuration
^^^^^^^^^^^^^

``mobile_fr3_duo.launch.py`` also exposes only three CLI launch arguments. As with
``fr3_duo.launch.py``, the header comment in the launch file mostly documents keys
from the YAML config file, not extra command-line arguments.

``mobile_fr3_duo.launch.py`` launch arguments
""""""""""""""""""""""""""""""""""""""""""""""

.. list-table::
    :header-rows: 1

    * - Name
      - Type
      - Default
      - Description
    * - ``robot_config_file``
      - string
      - ``mobile_fr3_duo.config.yaml``
      - Config file name looked up in ``franka_bringup/config/`` or an absolute path.
    * - ``controllers_yaml``
      - string
      - ``franka_bringup/config/controllers.yaml``
      - Override the controller definition file passed to ``ros2_control_node``.
    * - ``controller_name``
      - string
      - none
      - Controller to spawn. This argument is required.

The mobile dual-arm configuration is defined in
``franka_bringup/config/mobile_fr3_duo.config.yaml``. The main config file keys are:

.. list-table::
    :header-rows: 1

    * - Key
      - Type
      - Example
      - Description
    * - ``robot_types``
      - stringified list
      - ``"['tmrv0_2', 'fr3v2', 'fr3v2']"``
      - Robot model identifiers for the mobile base and both arms.
    * - ``robot_prefixes``
      - stringified list
      - ``"['', 'left', 'right']"``
      - Prefixes for the mobile base and both arms. The first entry is the mobile
        base prefix and is usually empty.
    * - ``robot_ips``
      - stringified list
      - ``"['172.16.16.10', '172.16.16.11', '172.16.16.12']"``
      - IP addresses for the mobile base and both arms.
    * - ``load_gripper``
      - bool
      - ``false``
      - Enable the gripper launch integration.
    * - ``use_fake_hardware``
      - bool
      - ``false``
      - Use fake hardware instead of real robots.
    * - ``fake_sensor_commands``
      - bool
      - ``false``
      - Enable fake sensor commands when fake hardware is used.
    * - ``namespace``
      - string
      - ``''``
      - ROS namespace applied to the full mobile dual-arm setup.
    * - ``joint_state_rate``
      - integer
      - ``30``
      - Joint state publisher rate in Hz.
    * - ``use_rviz``
      - bool
      - ``true``
      - Start RViz together with the launch file.
    * - ``thread_priority``
      - integer
      - ``98``
      - Thread priority used by the hardware interface. Must stay above the controller
        manager's ``thread_priority``, since this thread owns the 1 kHz robot socket.

.. note::

    All three arrays (``robot_types``, ``robot_ips``, ``robot_prefixes``) must have
    exactly **3 entries** (mobile base + 2 arms), and ``robot_prefixes`` must be unique.

    The self-collision node is started automatically by
    ``mobile_fr3_duo.launch.py``. There is no ``check_selfcollision`` config key in
    ``mobile_fr3_duo.config.yaml``.


Launching the Mobile FR3 Duo
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

To launch the mobile dual-arm setup with the joint impedance controller using a config file:

.. code-block:: shell

    ros2 launch franka_bringup mobile_fr3_duo.launch.py \
        controller_name:=mobile_fr3_duo_joint_impedance_example_controller

.. note::

    The Mobile FR3 Duo setup supports only **one controller** at a time using the ``controller_name`` parameter (singular).

System Architecture
^^^^^^^^^^^^^^^^^^^

The mobile dual-arm system has the following kinematic structure:

.. code-block:: text

    base → base_link (TMRv0.2) → franka_spine → franka_spine_support (prismatic)
         → mount_link → left/right FR3 arms

The Franka spine includes a **prismatic joint** for vertical adjustment
(0 to 0.85 m range).

Non-realtime robot parameter setting
------------------------------------

Non-realtime robot parameter setting can be done via ROS 2 services. The services
are advertised after the robot hardware is initialized.

Service names are:

 * /service_server/set_cartesian_stiffness
 * /service_server/set_force_torque_collision_behavior
 * /service_server/set_full_collision_behavior
 * /service_server/set_joint_stiffness
 * /service_server/set_load
 * /service_server/set_parameters
 * /service_server/set_parameters_atomically
 * /service_server/set_stiffness_frame
 * /service_server/set_tcp_frame

Service message descriptions are given below.

 * ``franka_msgs::srv::SetJointStiffness`` specifies joint stiffness for the internal controller
   (damping is automatically derived from the stiffness).
 * ``franka_msgs::srv::SetCartesianStiffness`` specifies Cartesian stiffness for the internal
   controller (damping is automatically derived from the stiffness).
 * ``franka_msgs::srv::SetTCPFrame`` specifies the transformation from <robot_type>_EE (end effector) to
   <robot_type>_NE (nominal end effector) frame. The transformation from flange to end effector frame
   is split into two transformations: <robot_type>_EE to <robot_type>_NE frame and <robot_type>_NE to
   <robot_type>_link8 frame. The transformation from <robot_type>_NE to <robot_type>_link8 frame can only be
   set through the administrator's interface.
 * ``franka_msgs::srv::SetStiffnessFrame`` specifies the transformation from <robot_type>_K to <robot_type>_EE frame.
 * ``franka_msgs::srv::SetForceTorqueCollisionBehavior`` sets thresholds for external Cartesian
   wrenches to configure the collision reflex.
 * ``franka_msgs::srv::SetFullCollisionBehavior`` sets thresholds for external forces on Cartesian
   and joint level to configure the collision reflex.
 * ``franka_msgs::srv::SetLoad`` sets an external load to compensate (e.g. of a grasped object).

Launch ``franka_bringup/franka.launch.py`` to initialize the robot hardware:

.. code-block:: shell

    ros2 launch franka_bringup franka.launch.py robot_type:=fr3 robot_ip:=172.16.0.3

Here is a minimal example:

.. code-block:: shell

    ros2 service call /service_server/set_joint_stiffness \
      franka_msgs/srv/SetJointStiffness \
      "{joint_stiffness: [1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0]}"

.. important::

    Non-realtime parameter setting can only be done when the robot hardware is in `idle` mode.
    If a controller is active and claims command interface this will put the robot in the `move` mode.
    In `move` mode non-realtime param setting is not possible.

.. important::

    The <robot_type>_EE frame denotes the part of the
    configurable end effector frame which can be adjusted during run time through `franka_ros`. The
    <robot_type>_K frame marks the center of the internal
    Cartesian impedance. It also serves as a reference frame for external wrenches. *Neither the
    <robot_type>_EE nor the <robot_type>_K are contained in the URDF as they can be changed at run time*.
    In the examples above, ``<robot_type>`` is ``fr3``.

    .. figure:: ../../docs/assets/frames.svg
        :align: center
        :figclass: align-center

        Overview of the end-effector frames.

Non-realtime ROS 2 actions
--------------------------

Non-realtime ROS 2 actions are exposed by the action server. The following action is
available:

* ``/action_server/error_recovery`` - Recovers automatically from a robot error.

The following message type is used:

* ``franka_msgs::action::ErrorRecovery`` - no parameters.

Example usage:

.. code-block:: shell

    ros2 action send_goal /action_server/error_recovery franka_msgs/action/ErrorRecovery {}

Known Issues
------------

* When using fake hardware with MoveIt, it takes some time until the default
  position is applied.
