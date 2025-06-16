cmake_minimum_required(VERSION 3.0.2)
project(realsense_ros)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
)

## Catkin specific configuration
catkin_package(
  CATKIN_DEPENDS rospy sensor_msgs
)

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Mark executable scripts (Python etc.) for installation
catkin_install_python(PROGRAMS
  scripts/realsense_d435_pub.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
