cmake_minimum_required(VERSION 3.5)
project(scout_navigation)

# Set compile options
add_compile_options(-std=c++17 -Wall)

# Find necessary packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 3.3 REQUIRED)

# Include directories
include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/include/scout_navigation  # 헤더 파일 경로 추가
)

# Add library for gridmap_converter
add_library(gridmap_converter src/gridmap_converter.cpp)

# Link library with dependencies
ament_target_dependencies(gridmap_converter 
  rclcpp 
  std_msgs 
  nav_msgs 
  grid_map_ros 
  grid_map_msgs 
)

# Add executable for gridmap_converter_node
add_executable(gridmap_converter_node src/nodes/gridmap_converter_node.cpp)

# Link node executable to the gridmap_converter library
target_link_libraries(gridmap_converter_node gridmap_converter)

# Specify dependencies for the node executable
ament_target_dependencies(gridmap_converter_node 
  rclcpp 
  std_msgs 
  nav_msgs 
  grid_map_ros 
  grid_map_msgs 
)

# Add executable for occmap_to_costmap node
add_executable(occmap_to_costmap src/occmap_to_costmap.cpp)

# Specify dependencies for the occmap_to_costmap node executable
ament_target_dependencies(occmap_to_costmap 
  rclcpp 
  nav2_costmap_2d 
  nav_msgs 
  tf2 
  tf2_ros
)

# Astar library and executable
add_library(Astar src/test_Astar.cpp)

# A* main executable
add_executable(test_Astar src/main.cpp)

# Specify dependencies for the Astar library and main executable
ament_target_dependencies(Astar 
  rclcpp 
  nav2_costmap_2d 
  nav_msgs 
  tf2 
  tf2_ros
  Eigen3
)

ament_target_dependencies(test_Astar 
  rclcpp 
  nav2_costmap_2d 
  nav_msgs 
  tf2 
  tf2_ros
  Eigen3
)

# Link A* executable to Astar library
target_link_libraries(test_Astar Astar)

# Install targets
install(
  TARGETS gridmap_converter gridmap_converter_node occmap_to_costmap Astar test_Astar
  DESTINATION lib/${PROJECT_NAME}
)

# Install other project-specific directories
install(
  DIRECTORY launch config maps rviz slam_params
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
