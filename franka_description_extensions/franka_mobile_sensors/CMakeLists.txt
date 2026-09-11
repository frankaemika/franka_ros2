cmake_minimum_required(VERSION 3.5)
project(franka_mobile_sensors)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config rviz robots doc
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_pytest_test(
    test_mobile_sensor_urdf_expansion
    test/test_mobile_sensor_urdf_expansion.py
  )
endif()

ament_package()
