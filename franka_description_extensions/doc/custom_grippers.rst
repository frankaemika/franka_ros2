.. _franka_description_extension_custom_grippers:

Attaching a third-party gripper
================================

An external gripper is a description extension, not an ``ee_id`` registration.
Keep the arm description and the gripper description in separate xacro
macros, then connect them with a fixed joint.

The supported composition pattern is:

#. Disable the built-in end effector in the Franka description with
   ``hand="false"`` and ``ee_id="none"``.
#. Expand the arm prefix using the same rule as the selected Franka entry
   point.
#. Attach the external macro to
   ``<arm_prefix><robot_type>_link8``.
#. Give the gripper its own base link, visual geometry, collision geometry,
   inertial data, and TCP frame.
#. Add ros2_control and controller wiring in the runtime overlay required by
   the gripper. The fixed-joint description macro does not provide that
   wiring.

Fixed-joint description macro
-----------------------------

The following is a complete shape for an external macro. The package name
``example_gripper_description`` is a placeholder for the third-party package
that owns the mesh and xacro files.

.. code-block:: xml

   <xacro:macro name="example_gripper"
       params="parent prefix:=''
               xyz:='0 0 0' rpy:='0 0 0'
               tcp_xyz:='0 0 0.15' tcp_rpy:='0 0 0'">
     <link name="${prefix}gripper_base_link">
       <visual>
         <geometry>
           <mesh filename="package://example_gripper_description/meshes/visual/gripper.dae"/>
         </geometry>
       </visual>
       <collision>
         <geometry>
           <mesh filename="package://example_gripper_description/meshes/collision/gripper.stl"/>
         </geometry>
       </collision>
       <inertial>
         <origin xyz="0 0 0" rpy="0 0 0"/>
         <mass value="1.0"/>
         <inertia ixx="0.01" ixy="0.0" ixz="0.0"
                  iyy="0.01" iyz="0.0" izz="0.01"/>
       </inertial>
     </link>

     <joint name="${prefix}gripper_mount_joint" type="fixed">
       <parent link="${parent}"/>
       <child link="${prefix}gripper_base_link"/>
       <origin xyz="${xyz}" rpy="${rpy}"/>
     </joint>

     <link name="${prefix}gripper_tcp"/>
     <joint name="${prefix}gripper_tcp_joint" type="fixed">
       <parent link="${prefix}gripper_base_link"/>
       <child link="${prefix}gripper_tcp"/>
       <origin xyz="${tcp_xyz}" rpy="${tcp_rpy}"/>
     </joint>
   </xacro:macro>

The mesh paths, mass, inertia, mount transform, and TCP transform must be
replaced with the gripper manufacturer's data. Additional gripper links and
moving joints can be emitted by the same macro.

Connecting the macro to a Franka arm
------------------------------------

For a single-arm wrapper that accepts the public, unexpanded ``arm_prefix``
token, derive the expanded prefix once and use it for both the arm and the
gripper parent:

.. code-block:: xml

   <xacro:include filename="$(find franka_description)/robots/common/franka_robot.xacro"/>
   <xacro:include
       filename="$(find example_gripper_description)/urdf/example_gripper.xacro"/>

   <xacro:macro name="franka_with_example_gripper"
       params="robot_type arm_prefix:=''">
     <xacro:property name="expanded_arm_prefix"
         value="${arm_prefix + '_' if arm_prefix else ''}"/>

     <xacro:franka_robot
         robot_type="${robot_type}"
         arm_prefix="${expanded_arm_prefix}"
         connected_to="base"
         hand="false"
         ee_id="none"/>

     <xacro:example_gripper
         parent="${expanded_arm_prefix}${robot_type}_link8"
         prefix="${expanded_arm_prefix}"/>
   </xacro:macro>

For ``robot_type="fr3v2_1"`` and ``arm_prefix="left"``, the fixed joint
parent is ``left_fr3v2_1_link8``. With an empty prefix it is
``fr3v2_1_link8``. A duo wrapper must perform the same calculation separately
for its left and right arm prefixes.

Built-in end effectors are a different path
--------------------------------------------

The upstream ``franka_robot`` macro has built-in handling for
``ee_id="franka_hand"`` and for upstream one-link end effectors. Those paths
load xacros and inertial data from the ``franka_description`` end-effector
tree. An arbitrary third-party package name is not a documented registry
interface: setting ``ee_id`` to an external name does not make that package
discoverable.

Use the external fixed-joint pattern above instead. ``special_connection`` is
also an upstream built-in-end-effector parameter. When it is non-empty, the
upstream macro uses it instead of ``<robot_type>_link8`` and prepends the
expanded arm prefix. It does not alter the ``parent`` supplied to an external
macro.

``description_pkg`` is not a complete package relocation switch
-----------------------------------------------------------------

The current base macros accept and forward ``description_pkg`` through the
outer and built-in end-effector calls, but the parameter does not relocate the
whole description. The current ``franka_robot`` call does not pass it to
``franka_arm``, so the arm still uses the default ``franka_description`` for
its visual resources. The arm also loads its joint limits, kinematics,
inertials, dynamics, and accelerometer YAML from ``franka_description``; its
collision mesh URI and several include paths are hard-coded to that package.
Built-in end-effector visual mesh URIs can use ``description_pkg``, while
their include and inertial-data paths remain tied to ``franka_description``.

Leave the base description resources on ``franka_description`` and keep
third-party gripper meshes and inertials in the external package's own macro.
Do not use ``description_pkg`` as a substitute for the fixed-joint extension
pattern.

Control and controller integration
----------------------------------

The fixed-joint macro describes geometry and TF only. If the gripper has
actuated joints, add its ``<ros2_control>`` hardware/component description,
controller configuration, driver, and lifecycle startup in the appropriate
overlay:

* use the real/mock composition owned by ``franka_bringup`` and
  ``franka_hardware`` for hardware bringup; or
* use the Gazebo composition owned by ``franka_gazebo_bringup`` for
  simulation.

The optional Robotiq control block in
``franka_vision_and_manipulation_kit`` is specific to that package's
Robotiq macro. It is not a general third-party gripper registration mechanism.

