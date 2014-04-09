##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
project(roshue)

##############################################################################
# Catkin
##############################################################################
find_package(catkin REQUIRED COMPONENTS
    message_generation
    std_msgs
)

catkin_python_setup()
## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   Hue.msg
   HueState.msg
   HueArray.msg
 )
## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
)
catkin_package(
  CATKIN_DEPENDS message_runtime
)
##############################################################################
# Installs
##############################################################################