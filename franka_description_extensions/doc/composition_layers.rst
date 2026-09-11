.. _franka_description_extension_layers:

Composition layers
==================

The description extensions follow three composition layers. Keeping the
layers separate makes a URDF usable for visualization or hardware bringup
without requiring a simulator.

Layer 1: base robot description
-------------------------------

``franka_description`` owns the robot geometry, kinematic frames, and the
standard mounting points. The relevant base macros are:

* ``tmrv0_2`` in
  ``franka_description/robots/tmrv0_2/tmrv0_2.xacro``;
* ``mobile_fr3_duo_v0_2`` in
  ``franka_description/robots/mobile_fr3_duo_v0_2/mobile_fr3_duo_v0_2.xacro``;
* ``fr3_duo`` in
  ``franka_description/robots/fr3_duo/fr3_duo.xacro``; and
* ``franka_robot`` in
  ``franka_description/robots/common/franka_robot.xacro`` for an arm and its
  optional built-in end effector.

The arm macro emits ``link0`` through ``link8`` (with the normal expanded
prefix). ``fr3_duo`` adds the duo mount and head, while
``mobile_fr3_duo_v0_2`` adds the TMR base and spine before composing the same
duo structure.

Layer 2: description extensions
--------------------------------

Layer 2 adds physical links, joints, and sensor descriptions. It does not
choose a simulator or a hardware plugin.

``franka_mobile_sensors`` provides these standalone entry points:

* ``robots/tmrv0_2_with_sensors.urdf.xacro`` composes ``tmrv0_2`` with
  ``sensor_suite``.
* ``robots/mobile_fr3_duo_v0_2_with_sensors.urdf.xacro`` composes
  ``mobile_fr3_duo_v0_2`` with the same ``sensor_suite``.

``sensor_suite`` attaches four RealSense D455 descriptions, two SICK
nanoScan3 descriptions, and one OLV-IMU01 description to the TMR mounting
frames. The standalone entries contain no ``<gazebo>`` elements and do not
accept a Gazebo flag.

``franka_vision_and_manipulation_kit`` provides
``robots/vision_and_manipulation_kit.urdf.xacro``. It selects either
``fr3_duo`` or ``mobile_fr3_duo_v0_2`` and then attaches two Robotiq gripper
chains with RealSense D405 wrist cameras and one ZED Mini head camera. The
``ros2_control`` xacro argument controls the optional Robotiq control block;
it is not a general hardware overlay for the arms.

Layer 3: runtime and simulation overlays
-----------------------------------------

An application-level entry point adds the runtime-specific pieces after the
description layers:

* ``franka_bringup`` and ``franka_hardware`` provide the real or mock
  ``ros2_control`` composition used for hardware bringup.
* ``franka_gazebo_bringup`` owns the Gazebo hardware plugin, transmissions,
  SDF elements, and sensor plugins. Its
  ``gazebo_fr3_duo_example.launch.py`` and
  ``gazebo_mobile_fr3_duo_example.launch.py`` launch files select the
  corresponding Gazebo entry points.

For example, the Gazebo layer can compose the vision kit with:

.. code-block:: shell

   ros2 launch franka_gazebo_bringup gazebo_fr3_duo_example.launch.py \
       with_sensors:=true

The mobile variant composes both sensor packages:

.. code-block:: shell

   ros2 launch franka_gazebo_bringup gazebo_mobile_fr3_duo_example.launch.py \
       with_sensors:=true

``with_sensors`` and ``gazebo_effort`` belong to these Gazebo entry points.
They are not arguments of the standalone Layer-2 extension files. For the
complete commands and launch arguments, see
:doc:`Worked examples <examples>`.
