franka_desk_api (internal)
==========================

**This package is an internal implementation detail of franka_ros2.**

It provides a small shared HTTPS client used by ``franka_mobile`` (battery) and
``franka_spine_server`` to access Franka Desk device REST APIs. It is **not**
part of the public ROS 2 API and may be replaced or removed without notice
(e.g. when a dedicated Python REST library is introduced).

Do not depend on this package from application code or third-party packages.
Use the public interfaces of ``franka_mobile`` / ``franka_spine_server`` instead.
