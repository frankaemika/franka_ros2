cmake_minimum_required(VERSION 3.5)
project(franka_example_controllers)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(controller_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(franka_msgs REQUIRED)


find_package(Eigen3 REQUIRED)
add_library(
        ${PROJECT_NAME}
        SHARED
        src/gravity_compensation_example_controller.cpp
        src/joint_impedance_example_controller.cpp
)
target_include_directories(
        ${PROJECT_NAME}
        PUBLIC
        include
        ${EIGEN3_INCLUDE_DIRS}
)
ament_target_dependencies(
        ${PROJECT_NAME}
        geometry_msgs
        controller_interface
        hardware_interface
        pluginlib
        rclcpp
        rclcpp_lifecycle
)

pluginlib_export_plugin_description_file(
        controller_interface franka_example_controllers.xml)

install(
        TARGETS
        ${PROJECT_NAME}
        RUNTIME DESTINATION bin
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
)

install(
        DIRECTORY include/
        DESTINATION include
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(
        PROGRAMS
        scripts/interactive_marker.py
        DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
    find_package(ament_cmake_clang_format REQUIRED)
    find_package(ament_cmake_copyright REQUIRED)
    find_package(ament_cmake_cppcheck REQUIRED)
    find_package(ament_cmake_flake8 REQUIRED)
    find_package(ament_cmake_lint_cmake REQUIRED)
    find_package(ament_cmake_pep257 REQUIRED)
    find_package(ament_cmake_xmllint REQUIRED)

    set(CPP_DIRECTORIES src include)
    ament_clang_format(CONFIG_FILE ../.clang-format ${CPP_DIRECTORIES})
    ament_copyright(src ${CPP_DIRECTORIES} package.xml)
    ament_cppcheck(${CPP_DIRECTORIES})
    ament_lint_cmake(CMakeLists.txt)
    ament_flake8()
    ament_pep257()
    ament_xmllint()
endif()
ament_export_include_directories(
        include
)
ament_export_libraries(
        ${PROJECT_NAME}
)
ament_export_dependencies(
        controller_interface
        pluginlib
        rclcpp
        rclcpp_lifecycle
        hardware_interface
)
ament_package()
