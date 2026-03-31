cmake_minimum_required(VERSION 3.0.2)
project(roscan)

find_package(catkin REQUIRED COMPONENTS
  rospy
  socketcan_interface
  socketcan_bridge
  roar_msgs
  sensor_msgs
  std_msgs
  can_msgs
)

catkin_package(
  #  INCLUDE_DIRS include
  LIBRARIES roscan
  CATKIN_DEPENDS rospy socketcan_interface roar_msgs sensor_msgs std_msgs socketcan_bridge can_msgs
  #  DEPENDS system_lib
)
include_directories(${catkin_INCLUDE_DIRS})

install(
  DIRECTORY src
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

catkin_python_setup()

catkin_install_python(PROGRAMS
  nodes/node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
