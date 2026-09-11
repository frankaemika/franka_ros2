^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package franka_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------

* feat: **BREAKING CHANGE** add ``accelerometer_top`` and ``accelerometer_bottom``
  (``geometry_msgs/Vector3Stamped[6]``, translational acceleration of joints 1 through 6,
  stamped in the ``<prefix>link<i>_accelerometer_<side>`` frame) to ``FrankaRobotState``.
  The interface hash changes, so every subscriber, bridge and generated binding must be
  rebuilt against the new message definition.

1.0.0 (2025-01-22)
------------------
