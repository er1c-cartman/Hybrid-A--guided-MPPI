cmake_minimum_required(VERSION 3.5.1)
project(elevation_mapping)

# Use C++17 standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(kindr REQUIRED)
find_package(kindr_ros REQUIRED)
find_package(message_filters REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters)

set(DEPENDENCIES
  grid_map_core
  grid_map_ros
  grid_map_msgs
  kindr
  kindr_ros
  message_filters
  pcl_ros
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
  tf2
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  geometry_msgs
)

include_directories(
  include
  SYSTEM
    ${Boost_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

###########
# Library #
###########

add_library(${PROJECT_NAME}_pcl_types
  src/PointXYZRGBConfidenceRatio.cpp
)

target_link_libraries(${PROJECT_NAME}_pcl_types
  ${PCL_LIBRARIES}
)

ament_target_dependencies(${PROJECT_NAME}_pcl_types
  ${DEPENDENCIES}
)

add_library(${PROJECT_NAME}_library
  src/ElevationMapping.cpp
  src/ElevationMap.cpp
  src/input_sources/Input.cpp
  src/input_sources/InputSourceManager.cpp
  src/postprocessing/PostprocessorPool.cpp
  src/postprocessing/PostprocessingWorker.cpp
  src/postprocessing/PostprocessingPipelineFunctor.cpp
  src/RobotMotionMapUpdater.cpp
  src/sensor_processors/SensorProcessorBase.cpp
  src/sensor_processors/StructuredLightSensorProcessor.cpp
  src/sensor_processors/StereoSensorProcessor.cpp
  src/sensor_processors/LaserSensorProcessor.cpp
  src/sensor_processors/PerfectSensorProcessor.cpp
)

ament_target_dependencies(${PROJECT_NAME}_library
  ${DEPENDENCIES}
)

# Ensure all PCL libraries are linked here as well
target_link_libraries(${PROJECT_NAME}_library
  ${PROJECT_NAME}_pcl_types
  ${Boost_LIBRARIES}
  ${PCL_COMMON_LIBRARIES}
  ${PCL_IO_LIBRARIES}
  ${PCL_FILTERS_LIBRARIES}
  Threads::Threads
)

##############
# Executable #
##############

add_executable(elevation_mapping_node
  src/elevation_mapping_node.cpp
)

ament_target_dependencies(elevation_mapping_node
  ${DEPENDENCIES}
)

target_link_libraries(elevation_mapping_node
  ${PROJECT_NAME}_library
  ${Boost_LIBRARIES}
  ${PCL_COMMON_LIBRARIES}
  ${PCL_IO_LIBRARIES}
  ${PCL_FILTERS_LIBRARIES}
  Threads::Threads
)

#############
## Install ##
#############
install(PROGRAMS
  scripts/tf_to_pose_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  ${PROJECT_NAME}_pcl_types
  ${PROJECT_NAME}_library
  elevation_mapping_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

############
## Launch ##
############
install(FILES
  launch/elevation.launch.py
  DESTINATION share/${PROJECT_NAME}/launch
)

##########
## Test ##
##########
# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   find_package(ament_cmake_gtest REQUIRED)

#   ament_lint_auto_find_test_dependencies()

#   ament_add_gtest(test_${PROJECT_NAME}_input_sources
#     test/input_sources/input_sources.test
#     test/input_sources/test_input_sources.cpp
#     test/input_sources/InputSourcesTest.cpp
#   )

#   target_link_libraries(test_${PROJECT_NAME}_input_sources
#     ${PROJECT_NAME}_library
#   )

#   target_include_directories(test_${PROJECT_NAME}_input_sources
#     PRIVATE
#       include
#   )

#   target_include_directories(test_${PROJECT_NAME}_input_sources SYSTEM PUBLIC
#     ${Boost_INCLUDE_DIRS}
#     ${Eigen_INCLUDE_DIRS}
#   )
# endif()

###########
## Clang ##
###########

find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
  add_default_clang_tooling()
endif(cmake_clang_tools_FOUND)

ament_export_include_directories(include)
ament_export_libraries(
  ${PROJECT_NAME}_pcl_types
  ${PROJECT_NAME}_library
)
ament_export_dependencies(${DEPENDENCIES})

ament_package()

