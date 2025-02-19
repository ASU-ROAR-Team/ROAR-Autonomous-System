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




## Generate added messages with any dependencies listed here
generate_messages(
  DEPENDENCIES
  sensor_msgs
  std_msgs
  geometry_msgs
  roar_msgs
)


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


###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
)

###########
## Install ##
#############

# Mark executable scripts (Python) for installation
catkin_install_python(PROGRAMS
  scripts/aruco_tracking_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Testing ##
#############

# Add folders to be run by Python nosetests (if any)
# catkin_add_nosetests(test)
