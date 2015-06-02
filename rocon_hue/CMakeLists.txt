##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
project(rocon_hue)

##############################################################################
# Catkin
##############################################################################
find_package(catkin REQUIRED COMPONENTS
    roslint)

catkin_package()
catkin_python_setup()

file(GLOB_RECURSE ${PROJECT_NAME}_MODULE_SOURCES RELATIVE ${PROJECT_SOURCE_DIR} src/*.py)
file(GLOB_RECURSE ${PROJECT_NAME}_SCRIPT_SOURCES RELATIVE ${PROJECT_SOURCE_DIR} scripts/*)
roslint_python(${${PROJECT_NAME}_MODULE_SOURCES} ${${PROJECT_NAME}_SCRIPT_SOURCES})

##############################################################################
# Installs
##############################################################################
install(
    PROGRAMS
        scripts/server.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

install(DIRECTORY rapps
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
