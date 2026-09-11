franka_semantic_components
==========================

This package provides semantic helper classes for Franka-specific state, model, and
Cartesian command interfaces. The main classes are
``FrankaRobotModel``, ``FrankaRobotState``, ``FrankaSemanticComponentInterface``,
``FrankaCartesianPoseInterface``, and ``FrankaCartesianVelocityInterface``.

These classes provide typed access to the model and robot state objects stored in the
hardware state interfaces and simplify working with Cartesian command interfaces in
controllers.

For further reference on how to use these classes:
`Franka Robot State Broadcaster <https://github.com/frankarobotics/franka_ros2/tree/jazzy/franka_robot_state_broadcaster>`_
and
`Franka example controller (model_example_controller)
<https://github.com/frankarobotics/franka_ros2/blob/jazzy/franka_example_controllers/src/fr3/model_example_controller.cpp>`_

Franka Cartesian Pose Interface
-------------------------------

``FrankaCartesianPoseInterface`` sends Cartesian pose commands through the loaned command
interfaces. ``FrankaSemanticComponentInterface`` manages the loaned command and state
interfaces.

When constructing the interface, pass a boolean that enables or disables elbow
commands. If the robot uses an ``arm_prefix``, pass that prefix to the constructor too.

.. code-block:: cpp

    auto is_elbow_active = false;
    franka_semantic_components::FrankaCartesianPoseInterface cartesian_pose_interface(
        is_elbow_active);

    // Initialization with an arm prefix
    std::string arm_prefix = "arm_1";
    franka_semantic_components::FrankaCartesianPoseInterface prefixed_interface(
        arm_prefix, is_elbow_active);

You can use this interface to read the current Cartesian pose command values from the
Franka hardware interface.

.. code-block:: cpp

    std::array<double, 16> pose = cartesian_pose_interface.getCurrentPoseMatrix();

You can also read the current orientation and translation in Eigen types.

.. code-block:: cpp

    Eigen::Quaterniond quaternion;
    Eigen::Vector3d translation;
    std::tie(quaternion, translation) =
        cartesian_pose_interface.getCurrentOrientationAndTranslation();

After creating the interface, call ``assign_loaned_command_interfaces`` and
``assign_loaned_state_interfaces`` in your controller ``on_activate()`` method. An
example is available in
`the Cartesian pose example controller
<https://github.com/frankarobotics/franka_ros2/blob/jazzy/franka_example_controllers/src/fr3/cartesian_pose_example_controller.cpp>`_

.. code-block:: cpp

    cartesian_pose_interface.assign_loaned_command_interfaces(command_interfaces_);
    cartesian_pose_interface.assign_loaned_state_interfaces(state_interfaces_);

In the controller ``update()`` method, send Cartesian pose commands to the robot.

.. code-block:: cpp

    std::array<double, 16> pose = {1, 0, 0, 0, 0, 1, 0, 0,
                                   0, 0, 1, 0, 0.5, 0, 0.5, 1};
    cartesian_pose_interface.setCommand(pose);

You can also send a quaternion and translation in Eigen format.

.. code-block:: cpp

    Eigen::Quaterniond quaternion(1, 0, 0, 0);
    Eigen::Vector3d translation(0.5, 0, 0.5);
    cartesian_pose_interface.setCommand(quaternion, translation);

Franka Cartesian Velocity Interface
-----------------------------------

``FrankaCartesianVelocityInterface`` sends Cartesian velocity commands through the
loaned command interfaces. ``FrankaSemanticComponentInterface`` manages the loaned
command and state interfaces here as well.

.. code-block:: cpp

    auto is_elbow_active = false;
    franka_semantic_components::FrankaCartesianVelocityInterface
        cartesian_velocity_interface(is_elbow_active);

    // Initialization with an arm prefix
    std::string arm_prefix = "arm_1";
    franka_semantic_components::FrankaCartesianVelocityInterface prefixed_interface(
        arm_prefix, is_elbow_active);

To send velocity commands, call ``assign_loaned_command_interfaces`` in your controller.

.. code-block:: cpp

    cartesian_velocity_interface.assign_loaned_command_interfaces(command_interfaces_);

In the controller ``update()`` method, send Cartesian velocity commands to the robot.

.. code-block:: cpp

    Eigen::Vector3d linear_velocity(0.0, 0.0, 0.0);
    Eigen::Vector3d angular_velocity(0.0, 0.0, 0.1);
    cartesian_velocity_interface.setCommand(linear_velocity, angular_velocity);

Robot State and Model Access
-----------------------------

The semantic components provide safe access to the robot state and model objects stored
as pointers in the hardware interface. This ensures correct type casting and safer
controller code when working with these objects.