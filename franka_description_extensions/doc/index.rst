.. _franka_description_extensions:

Franka description extensions
=============================

This section documents the reusable robot-description extensions shipped with
``franka_ros2``. The source directory is a grouping folder; the two child
directories remain independent ROS packages with these unchanged package names:

* ``franka_mobile_sensors``
* ``franka_vision_and_manipulation_kit``

Start with the composition model when adding an extension or integrating one
of the supplied sensor kits. The focused guides describe the frame contract,
naming rules, external end-effector pattern, and the supported entry points.
The original package documentation remains available from this umbrella.

.. toctree::
   :maxdepth: 1
   :caption: Description extension guides

   composition_layers
   mounting_points
   naming_and_prefixes
   custom_grippers
   examples
   troubleshooting
   ../franka_mobile_sensors/doc/index
   ../franka_vision_and_manipulation_kit/doc/index
