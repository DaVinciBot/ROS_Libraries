cmake_minimum_required(VERSION 2.8.3)
project(dvb_navigation)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
  tf
)

find_package(Boost REQUIRED COMPONENTS system)

find_package(dvb_spinner REQUIRED)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES dvb_navigation
  CATKIN_DEPENDS geometry_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
  tf
  #DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${dvb_spinner_INCLUDE_DIRS}
)


add_library(dvb_navigation
  src/diff_robot_odometry.cpp
)

target_link_libraries(dvb_navigation
  ${catkin_LIBRARIES} 
  dvb_spinner
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

