cmake_minimum_required(VERSION 3.0.2)
project(inverted_pend_sim)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

find_package(yaml-cpp REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)


catkin_package(
 INCLUDE_DIRS include
 LIBRARIES inverted_pend_sim
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
  ${YAML_CPP_INCLUDE_DIRS}
  ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)

add_executable(
  inverted_pend_sim_node
  src/ros_simulator_node.cpp
  include/inverted_pend_sim/system_dynamics.cpp
  include/inverted_pend_sim/system_xx.cpp
  include/inverted_pend_sim/system_yy.cpp
  include/inverted_pend_sim/system_zz.cpp
  include/ros_wrapper/ros_wrapper_simulator.cpp
  include/yaml_read/yaml_read.cpp
)

target_link_libraries(
  inverted_pend_sim_node
  ${YAML_CPP_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_executable(
  trajectory_generator_node
  src/ros_trajectory_node.cpp
  include/trajectory_generation/trajectory_generator.cpp
)

target_link_libraries(
  trajectory_generator_node
  ${catkin_LIBRARIES}
)