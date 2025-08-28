cmake_minimum_required(VERSION 3.0.2)
project(supervisor)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  controller_manager
  roar_msgs
  rospy
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs controller_manager roar_msgs
)

include_directories(${catkin_INCLUDE_DIRS})

install(
  DIRECTORY src launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)

catkin_python_setup()

catkin_install_python(PROGRAMS
  nodes/test_talker.py
  nodes/node.py
  nodes/test_listener.py
  src/supervisor/logger.py
  src/supervisor/module_manager.py
  src/supervisor/drilling_module.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

