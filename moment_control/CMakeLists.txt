cmake_minimum_required(VERSION 3.0.2)
project(moment_control)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)


catkin_package(
 INCLUDE_DIRS include
 LIBRARIES moment_control
 CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)

add_executable(moment_control_node
  src/moment_control_node.cpp
  include/moment_control/moment_control.cpp
)

target_link_libraries(moment_control_node
  ${catkin_LIBRARIES}
)