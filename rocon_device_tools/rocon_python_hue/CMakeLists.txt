cmake_minimum_required(VERSION 2.8.3)
project(rocon_python_hue)

find_package(catkin REQUIRED COMPONENTS
  rospy
)
catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

