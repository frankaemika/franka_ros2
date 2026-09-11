cmake_minimum_required(VERSION 3.5)
project(franka_vision_and_manipulation_kit)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config rviz robots accessories meshes
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_pytest_test(
    test_vision_and_manipulation_kit_urdf_expansion
    test/test_vision_and_manipulation_kit_urdf_expansion.py
  )
endif()

ament_package()
