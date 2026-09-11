cmake_minimum_required(VERSION 3.5)

project(franka_gazebo_hardware)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Franka 0.19.0 REQUIRED)
find_package(franka_hardware REQUIRED)
find_package(gz_ros2_control REQUIRED)
find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pinocchio REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(
        franka_gz_hardware_plugins
        SHARED
        src/franka_gazebo_hardware_interface.cpp
        src/gravity_compensation_model.cpp
)

target_include_directories(franka_gz_hardware_plugins
    PUBLIC
        include
        ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(franka_gz_hardware_plugins
        ament_index_cpp
        Franka
        franka_hardware
        hardware_interface
        pluginlib
        rclcpp
        rclcpp_lifecycle
        realtime_tools
)

# gz_ros2_control provides the GazeboSimSystemInterface base class as headers
# only. Deliberately depend on it for include directories WITHOUT linking its
# libraries: libgz_hardware_plugins.so / libgz_ros2_control-system.so are
# class_loader/pluginlib plugins and must be loaded exclusively through pluginlib
# at runtime. Linking them (as ament_target_dependencies would) adds a DT_NEEDED
# that makes the dynamic loader run GazeboSimSystem's plugin registration outside
# class_loader's tracking, so the createSharedInstance("gz_ros2_control/GazeboSimSystem")
# call in initSim() then fails with "no factory exists for it".
# ${gz_ros2_control_INCLUDE_DIRS} carries the transitive dependency headers
# (controller_manager, hardware_interface, ...), and the imported system target
# carries the package's own include/gz_ros2_control prefix. Taking include
# directories only (never the target itself) keeps the plugin libraries off the
# link interface.
target_include_directories(franka_gz_hardware_plugins SYSTEM PUBLIC
        ${gz_ros2_control_INCLUDE_DIRS}
)
# Also pull the package's own include/gz_ros2_control prefix from the imported
# system target. Guard on target existence so a renamed/removed target in a
# future gz_ros2_control release fails loudly at configure time rather than
# silently dropping the include path (which would only surface as a build error).
if(TARGET gz_ros2_control::gz_ros2_control-system)
        target_include_directories(franka_gz_hardware_plugins SYSTEM PUBLIC
                $<TARGET_PROPERTY:gz_ros2_control::gz_ros2_control-system,INTERFACE_INCLUDE_DIRECTORIES>
        )
elseif(NOT gz_ros2_control_INCLUDE_DIRS)
        message(FATAL_ERROR
                "gz_ros2_control provides neither the "
                "gz_ros2_control::gz_ros2_control-system imported target nor "
                "gz_ros2_control_INCLUDE_DIRS; cannot locate the "
                "GazeboSimSystemInterface headers.")
endif()

target_link_libraries(franka_gz_hardware_plugins
        gz-sim::gz-sim
        pinocchio::pinocchio
)

pluginlib_export_plugin_description_file(
        gz_ros2_control franka_gz_hardware_plugins.xml)

install(
        TARGETS
        franka_gz_hardware_plugins
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
)

install(
        DIRECTORY include/
        DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gmock REQUIRED)
  ament_add_gmock(gravity_compensation_model_test test/gravity_compensation_model_test.cpp)
  target_include_directories(gravity_compensation_model_test PRIVATE
          include
          ${EIGEN3_INCLUDE_DIRS}
  )
  target_link_libraries(gravity_compensation_model_test
          franka_gz_hardware_plugins
          pinocchio::pinocchio
  )
endif()

ament_package()
