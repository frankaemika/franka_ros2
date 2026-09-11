cmake_minimum_required(VERSION 3.5)

option(CHECK_TIDY "Enable clang-tidy checks" OFF)

project(franka_spine_server)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/spine_action_server_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_spine_controller test/test_spine_controller.py)
endif()

ament_package()
