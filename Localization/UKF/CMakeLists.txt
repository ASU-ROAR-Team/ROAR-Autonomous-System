cmake_minimum_required(VERSION 3.0)
project(localization
        LANGUAGES CXX)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  roar_msgs
  geometry_msgs
  message_generation
  tf2
  tf2_ros
)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
  roscpp
  rospy
  std_msgs
  roar_msgs
  geometry_msgs
  message_runtime
  tf2
  tf2_ros
  INCLUDE_DIRS include 
)

include_directories(
  include  
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(main src/main.cpp src/ROAR_UKF.cpp src/Quaternion.cpp)
add_dependencies(main localization_generate_messages_cpp)

target_link_libraries(main ${catkin_LIBRARIES} Eigen3::Eigen)
target_include_directories(main PRIVATE include)
