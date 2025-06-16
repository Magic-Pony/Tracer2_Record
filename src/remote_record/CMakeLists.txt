cmake_minimum_required(VERSION 3.0.2)
project(remote_record)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  urdf
  robot_state_publisher
  rviz
)

## Catkin specific configuration
catkin_package(
  CATKIN_DEPENDS roscpp rospy urdf robot_state_publisher rviz
)

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
)
