##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
project(rocon_hue)

##############################################################################
# Catkin
##############################################################################
find_package(catkin REQUIRED COMPONENTS
    roslint
    message_generation
    std_msgs
)

catkin_python_setup()

add_message_files(
   FILES
   Hue.msg
   HueState.msg
   HueArray.msg
 )

generate_messages(
   DEPENDENCIES
   std_msgs  
)
catkin_package(
  CATKIN_DEPENDS message_runtime
)

file(GLOB_RECURSE ${PROJECT_NAME}_MODULE_SOURCES RELATIVE ${PROJECT_SOURCE_DIR} src/*.py)
file(GLOB_RECURSE ${PROJECT_NAME}_SCRIPT_SOURCES RELATIVE ${PROJECT_SOURCE_DIR} scripts/*)
roslint_python(${${PROJECT_NAME}_MODULE_SOURCES} ${${PROJECT_NAME}_SCRIPT_SOURCES})

##############################################################################
# Installs
##############################################################################

install(PROGRAMS     
    scripts/test.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)