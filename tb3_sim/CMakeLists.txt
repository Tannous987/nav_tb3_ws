cmake_minimum_required(VERSION 3.8)
project(tb3_sim)

# Find necessary dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)

# Define your C++ executable (AMCL initial pose publisher)
add_executable(amcl_init_pose_publisher src/amcl_init_pose_publisher.cpp)
ament_target_dependencies(amcl_init_pose_publisher rclcpp geometry_msgs tf2)

# Install the executable to ROS2 workspace
install(TARGETS amcl_init_pose_publisher
  DESTINATION lib/${PROJECT_NAME})

# Install launch and maps and config directories
install(DIRECTORY
  launch
  maps
  config
  DESTINATION share/${PROJECT_NAME}
)

# Ament package export
ament_package()