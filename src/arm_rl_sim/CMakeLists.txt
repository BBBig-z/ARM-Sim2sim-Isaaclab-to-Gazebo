cmake_minimum_required(VERSION 3.5)
project(arm_rl_sim)

add_definitions(-DCMAKE_CURRENT_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# Install Python scripts
install(PROGRAMS
  scripts/arm_rl_sim.py
  scripts/target_pose_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install resource directories
install(
  DIRECTORY launch models config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
