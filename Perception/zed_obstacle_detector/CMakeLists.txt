cmake_minimum_required(VERSION 3.0.2)
project(zed_obstacle_detector)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  pcl_ros
  pcl_conversions
  roar_msgs
)

find_package(PCL REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs roar_msgs pcl_ros pcl_conversions
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(zed_obstacle_detector_node zed_obstacle_detector_node.cpp)

target_link_libraries(zed_obstacle_detector_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
