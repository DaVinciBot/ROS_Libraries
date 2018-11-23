cmake_minimum_required(VERSION 2.8.3)
project(dvb_control)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  geometry_msgs
  genmsg
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

add_action_files(DIRECTORY action FILES SetPIDSetpoint.action)
add_action_files(DIRECTORY action FILES SetMoveToPoint.action)
generate_messages(DEPENDENCIES std_msgs geometry_msgs actionlib_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES dvb_control
  CATKIN_DEPENDS actionlib actionlib_msgs geometry_msgs genmsg roscpp rospy sensor_msgs std_msgs
  DEPENDS message_runtime
)

###########
## Build ##
###########

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)
add_library(dvb_control
  src/simple_pid.cpp
  src/pid_control_cdr.cpp
  src/move_to_point.cpp
)

target_link_libraries(dvb_control
   ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)