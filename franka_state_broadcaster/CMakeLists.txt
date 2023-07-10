cmake_minimum_required(VERSION 3.5)
project(franka_state_broadcaster)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

option(CHECK_TIDY "Adds clang-tidy tests" OFF)

set(THIS_PACKAGE_INCLUDE_DEPENDS
    builtin_interfaces
    control_msgs
    controller_interface
    controller_manager
    franka_msgs
    franka_semantic_components
    generate_parameter_library
    pluginlib
    rclcpp_lifecycle
    rcutils
    realtime_tools
    sensor_msgs)


# find dependencies
find_package(ament_cmake REQUIRED)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(franka_state_broadcaster
        SHARED
        src/franka_state_broadcaster.cpp)
target_include_directories(
  franka_state_broadcaster
        PRIVATE
        include
)


generate_parameter_library(franka_state_broadcaster_parameters src/franka_state_broadcaster_parameters.yaml)

target_link_libraries(franka_state_broadcaster
                      PUBLIC franka_state_broadcaster_parameters)
ament_target_dependencies(franka_state_broadcaster PUBLIC
                          ${THIS_PACKAGE_INCLUDE_DEPENDS})


pluginlib_export_plugin_description_file(controller_interface franka_state_broadcaster.xml)


install(
        TARGETS franka_state_broadcaster
        DESTINATION lib
)
install(
        DIRECTORY include/
        DESTINATION include
)


if(BUILD_TESTING)
    find_package(ament_cmake_gmock REQUIRED)
    find_package(ament_cmake_clang_format REQUIRED)
    find_package(ament_cmake_copyright REQUIRED)
    find_package(ament_cmake_cppcheck REQUIRED)
    find_package(ament_cmake_flake8 REQUIRED)
    find_package(ament_cmake_lint_cmake REQUIRED)
    find_package(ament_cmake_pep257 REQUIRED)
    find_package(ament_cmake_xmllint REQUIRED)
    find_package(ros2_control_test_assets REQUIRED)
    find_package(controller_manager REQUIRED)

    set(CPP_DIRECTORIES src include)
    ament_clang_format(CONFIG_FILE ../.clang-format ${CPP_DIRECTORIES})
    ament_copyright(${CPP_DIRECTORIES} package.xml)
    ament_cppcheck(${CPP_DIRECTORIES})
    ament_lint_cmake(CMakeLists.txt)
    ament_flake8()
    ament_pep257()
    ament_xmllint()
    if(CHECK_TIDY)
        find_package(ament_cmake_clang_tidy REQUIRED)
        set(ament_cmake_clang_tidy_CONFIG_FILE ../.clang-tidy)
        ament_clang_tidy(${CMAKE_BINARY_DIR})
    endif()

    ament_add_gmock(${PROJECT_NAME}_test test/test_franka_state_broadcaster.cpp)
    target_include_directories(${PROJECT_NAME}_test PRIVATE include)
    target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})
    ament_target_dependencies(${PROJECT_NAME}_test controller_manager hardware_interface)

    ament_add_gmock(test_load_franka_state_broadcaster test/test_load_franka_state_broadcaster.cpp)
    target_include_directories(test_load_franka_state_broadcaster PRIVATE include)
    target_link_libraries(test_load_franka_state_broadcaster ${PROJECT_NAME})
    ament_target_dependencies(
      test_load_franka_state_broadcaster controller_manager hardware_interface
      ros2_control_test_assets)
endif()

ament_export_include_directories(
        include
)
ament_export_libraries(
        franka_state_broadcaster
)
ament_export_dependencies(
        hardware_interface
        pluginlib
        rclcpp
)
ament_package()
