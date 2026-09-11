.. _franka_description_extension_mounting_points:

Mounting-point catalog
======================

Mounting points are ordinary URDF links. Attach an extension with a fixed
joint whose ``parent`` is one of these links; do not attach to a mesh name or
to a controller name.

Arm flange and ``link8``
------------------------

Each arm ends at a fixed ``link8`` frame. With prefixes enabled, its expanded
name is ``<arm_prefix><robot_type>_link8``. The upstream
``franka_robot`` macro uses this frame as the built-in end-effector parent
when ``special_connection`` is empty. For example:

* a single ``fr3v2_1`` arm without a prefix has
  ``fr3v2_1_link8``;
* a left ``fr3v2_1`` arm in a duo has
  ``left_fr3v2_1_link8``; and
* a stationary vision kit with ``prefix:=kit_`` has
  ``kit_left_fr3v2_1_link8`` and ``kit_right_fr3v2_1_link8``.

The exact expansion depends on the selected ``robot_type``, ``arm_prefix``,
``prefix``, and ``no_prefix`` settings. See
:doc:`Naming and prefixes <naming_and_prefixes>` before hard-coding a parent
frame.

``special_connection`` is an upstream built-in-end-effector option. When it
is non-empty, ``franka_robot`` uses it instead of
``<robot_type>_link8`` and then prepends the arm prefix. It does not discover
or re-parent an external xacro macro. An external gripper should use its
explicitly expanded ``link8`` parent.

``link8``, end effector, and TCP have different roles:

* ``link8`` is the last fixed arm link and the standard flange/physical
  attachment frame. It is not the tool center point.
* The end effector (EE) is the attached body or mechanism, such as the
  upstream ``hand`` link or an external gripper base link.
* The tool center point (TCP) is a separate fixed child frame of the EE. The
  upstream built-in paths place it with ``tcp_xyz`` and ``tcp_rpy``; an
  external macro must emit its own TCP frame and transform.

TMRv0.2 sensor points
---------------------

``franka_description/robots/tmrv0_2/tmrv0_2.xacro`` creates these links. All
of them are children of ``base_link``:

* ``imu_mounting_point`` at ``(0.260, 0.0, 0.1478)`` with
  ``rpy=(pi, 0, 0)``. The mobile sensor suite attaches its OLV-IMU01 here.
* ``front_mounting_point`` at ``(0.380705, 0.0, 0.2345)`` with
  ``rpy=(pi, 0, 0)``.
* ``rear_mounting_point`` at ``(-0.380705, 0.0, 0.2345)`` with
  ``rpy=(pi, 0, pi)``.
* ``left_mounting_point`` at ``(0.0, 0.272712, 0.1145)`` with
  ``rpy=(pi, 0, pi/2)``.
* ``right_mounting_point`` at ``(0.0, -0.272712, 0.1145)`` with
  ``rpy=(pi, 0, -pi/2)``.
* ``lidar_front_mounting_point`` at ``(0.3275, 0.2175, 0.19065)`` with
  ``rpy=(0, pi, 3*pi/4)``.
* ``lidar_rear_mounting_point`` at ``(-0.3275, -0.2175, 0.19065)`` with
  ``rpy=(0, pi, 7*pi/4)``.

The current ``sensor_suite`` uses the four directional points for the D455
cameras and the two ``lidar_*`` points for the nanoScan3 descriptions. It
uses the frame names exactly as emitted, without an arm prefix.

FR3 Duo mount
-------------

The ``fr3_duo`` macro creates:

* ``fr3_duo_mount_mounting_point`` for the physical duo mount;
* ``fr3_duo_mount_origin`` for the mount's kinematic origin; and
* ``fr3_duo_mount_<left_prefix>`` and
  ``fr3_duo_mount_<right_prefix>`` for the two arm attachment points.

The mount macro attaches ``fr3_duo_mount_mounting_point`` to its ``parent``
with an identity fixed joint. The stationary ``fr3_duo`` entry point passes
``base`` as that parent. The mobile
``mobile_fr3_duo_v0_2`` entry point passes
``franka_spine_mounting_point`` instead.

Spine
-----

The mobile macro calls ``franka_spine parent="base_link"``. The resulting
chain is:

.. code-block:: text

   base_link
     └── franka_spine
           └── franka_spine_mounting_point
                 └── fr3_duo_mount_mounting_point

``franka_spine_vertical_joint`` is the prismatic joint between
``franka_spine`` and ``franka_spine_mounting_point``. The duo arms are
therefore mounted through ``franka_spine_mounting_point`` rather than
directly to ``base_link``.

Head camera point
-----------------

The ``fr3_duo`` macro creates the head once, attached to
``fr3_duo_mount_mounting_point``. The head macro then creates
``head_camera_mounting_point`` as a child of ``head_link``. The vision kit
attaches the ZED camera with a fixed joint whose parent is exactly
``head_camera_mounting_point`` and whose child is
``<prefix>head_camera_link``.

Do not add another ``head`` macro when composing the vision kit. The base
duo description already supplies ``head_link`` and
``head_camera_mounting_point``.

