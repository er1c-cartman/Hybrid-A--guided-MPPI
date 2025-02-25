cmake_minimum_required(VERSION 3.5)
project(pointcloud_filter)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(livox_ros_driver2 REQUIRED) # Add this line

# Include directories
include_directories(include)

# Add executables
add_executable(pointcloud_filter_node src/pointcloud_filter_node.cpp)
add_executable(custom_pointcloud_filter_node src/custom_pointcloud_filter_node.cpp)

ament_target_dependencies(
  pointcloud_filter_node
  rclcpp
  sensor_msgs
  pcl_conversions
  pcl_ros
)

ament_target_dependencies(
  custom_pointcloud_filter_node
  rclcpp
  sensor_msgs
  pcl_conversions
  pcl_ros
  livox_ros_driver2
)

install(TARGETS
  pointcloud_filter_node
  custom_pointcloud_filter_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

