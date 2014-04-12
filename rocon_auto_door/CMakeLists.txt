cmake_minimum_required(VERSION 2.8.3)
project(rocon_auto_door)
find_package(catkin REQUIRED)
catkin_python_setup()
catkin_package()

install(PROGRAMS scripts/auto_door.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})