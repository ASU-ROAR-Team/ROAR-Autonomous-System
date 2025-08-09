cmake_minimum_required(VERSION 3.0.2)
project(zed_obstacle_detector)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  pcl_ros
  pcl_conversions
  roar_msgs
  visualization_msgs
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
  dynamic_reconfigure
)

find_package(PCL REQUIRED)

# Generate dynamic_reconfigure files
generate_dynamic_reconfigure_options(
  cfg/ZedObstacleDetector.cfg
)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs roar_msgs pcl_ros pcl_conversions visualization_msgs tf2_ros tf2_geometry_msgs tf2_eigen dynamic_reconfigure
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# Main executable with modular components
add_executable(zed_obstacle_detector_node 
  zed_obstacle_detector_node.cpp
  src/ground_detector.cpp
  src/obstacle_tracker.cpp
  src/point_cloud_processor.cpp
  src/cluster_detector.cpp
  src/coordinate_transformer.cpp
  src/performance_monitor.cpp
  src/obstacle_detector.cpp
)

target_link_libraries(zed_obstacle_detector_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
