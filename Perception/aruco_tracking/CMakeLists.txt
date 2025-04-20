cmake_minimum_required(VERSION 3.0.2)
project(aruco_tracking)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  std_msgs
  message_generation
  geometry_msgs
  roar_msgs
)

#catkin_install_python(PROGRAMS
  #scripts/aruco_tracking_node.py
 # DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)







## Declare ROS dynamic reconfigure parameters (if any)
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
# )

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS rospy sensor_msgs std_msgs geometry_msgs roar_msgs
)


include_directories(${catkin_INCLUDE_DIRS})

install(
  DIRECTORY src
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)

catkin_python_setup()

# Mark executable scripts (Python) for installation
catkin_install_python(PROGRAMS
  nodes/node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
