cmake_minimum_required(VERSION 3.0.2)
project(liveplotter)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roar_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

include_directories(
  include  
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES liveplotter
#  CATKIN_DEPENDS geometry_msgs matplotlib nav_msgs numpy roar_msgs roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)
