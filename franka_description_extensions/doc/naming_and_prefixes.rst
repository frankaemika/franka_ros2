.. _franka_description_extension_naming:

Naming and prefixes
===================

The URDF name is built from the robot model and the prefix at the composition
boundary. Keep the separator rule in mind: the public single-arm entry
points accept a prefix token such as ``left`` and add the trailing
underscore internally.

Single-arm names
----------------

The ``franka_description`` entry point for a model such as
``fr3v2_1`` declares an ``arm_prefix`` argument. Its wrapper computes:

.. code-block:: text

   expanded_arm_prefix = arm_prefix + "_" if arm_prefix else ""
   link8 = expanded_arm_prefix + robot_type + "_link8"

With ``no_prefix:=false`` (the default), the resulting names include that
expanded prefix:

* ``arm_prefix:=`` (empty) gives ``fr3v2_1_link8``;
* ``arm_prefix:=left`` gives ``left_fr3v2_1_link8``; and
* ``arm_prefix:=right`` gives ``right_fr3v2_1_link8``.

Pass ``left``, not ``left_``, to a public single-arm entry point. The wrapper
adds the separator. ``no_prefix:=true`` suppresses the whole generated
prefix, producing ``link0`` through ``link8`` and unprefixed joints.

The low-level ``franka_robot`` macro receives the already expanded prefix.
This distinction matters when an extension calls that macro directly:
``arm_prefix="left_"`` produces ``left_fr3v2_1_link8`` for
``robot_type="fr3v2_1"``.

Stationary FR3 Duo
------------------

The stationary branch of
``vision_and_manipulation_kit.urdf.xacro`` passes
``[prefix + "left", prefix + "right"]`` to ``fr3_duo``. The duo macro adds
the separator before passing each arm prefix to ``franka_robot``.

For the default arguments:

.. code-block:: text

   left_arm_type=fr3v2_1
   right_arm_type=fr3v2_1
   prefix=""

the arm end frames are:

.. code-block:: text

   left_fr3v2_1_link8
   right_fr3v2_1_link8

To put the complete stationary kit under an outer name, pass a separator in
the vision-kit ``prefix`` value, for example ``prefix:=kit_``:

.. code-block:: text

   kit_left_fr3v2_1_link8
   kit_right_fr3v2_1_link8

The vision-kit entry point uses ``prefix`` literally; it does not add an
underscore before ``left`` or ``right``. Therefore ``kit_`` and ``kit`` have
different results.

Mobile FR3 Duo
--------------

The current ``mobile_fr3_duo_v0_2`` macro defines its arm prefixes internally
as ``left`` and ``right``. Consequently, the mobile branch of the vision kit
uses:

.. code-block:: text

   left_fr3v2_1_link8
   right_fr3v2_1_link8

for the default arm types, even when the outer vision-kit ``prefix`` is
non-empty. That outer prefix is used for accessory names, such as the ZED
camera and Robotiq chains; it does not rename the mobile arm frames.

The same distinction applies to the mobile sensor entry point. Its
``robot_types`` argument is a list whose current default is
``['tmrv0_2','fr3v2','fr3v2']``. The first item is the mobile base and the
two arm names are emitted with the upstream ``left`` and ``right`` prefixes.

Prefix rules for attachments
----------------------------

Use the expanded output name as the fixed-joint parent:

.. code-block:: text

   <arm_prefix><robot_type>_link8

Here ``<arm_prefix>`` includes its trailing underscore when it is non-empty.
For a public single-arm argument ``arm_prefix:=left``, the expanded value is
``left_``. For a stationary duo with ``prefix:=kit_``, the left expanded
value is ``kit_left_``. For the mobile duo, use the current fixed values
``left_`` and ``right_``.

``no_prefix`` is a description-wide switch, not an attachment remapping
feature. In particular, the vision-kit component macro still constructs
parents such as ``left_fr3v2_1_link8``. Leave ``no_prefix`` at its default
``false`` when using the supplied vision-kit components unless the whole
composition is adapted consistently.

