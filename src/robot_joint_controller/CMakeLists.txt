cmake_minimum_required(VERSION 3.5)
project(robot_joint_controller)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Detect ROS distribution
if(DEFINED ENV{ROS_DISTRO})
  set(ROS_DISTRO_ENV $ENV{ROS_DISTRO})
  if(ROS_DISTRO_ENV STREQUAL "foxy")
    add_definitions(-DROS_DISTRO_FOXY)
  elseif(ROS_DISTRO_ENV STREQUAL "humble")
    add_definitions(-DROS_DISTRO_HUMBLE)
  endif()
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(robot_msgs REQUIRED)
find_package(urdf REQUIRED)

# Single joint controller library
add_library(robot_joint_controller SHARED
  src/robot_joint_controller.cpp
)
target_include_directories(robot_joint_controller PRIVATE include)
ament_target_dependencies(robot_joint_controller
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  robot_msgs
  urdf
)

# Joint controller group library
add_library(robot_joint_controller_group SHARED
  src/robot_joint_controller_group.cpp
)
target_include_directories(robot_joint_controller_group PRIVATE include)
ament_target_dependencies(robot_joint_controller_group
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  robot_msgs
)

# Export plugin description
pluginlib_export_plugin_description_file(controller_interface robot_joint_controller_plugins.xml)

# Install libraries
install(TARGETS
  robot_joint_controller
  robot_joint_controller_group
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(
  robot_joint_controller
  robot_joint_controller_group
)
ament_export_dependencies(
  controller_interface
  hardware_interface
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  robot_msgs
)

ament_package()

