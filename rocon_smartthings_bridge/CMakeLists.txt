cmake_minimum_required(VERSION 2.8.3)
project(rocon_smartthings_bridge)

find_package(catkin REQUIRED COMPONENTS
  rocon_iot_bridge
  rospy
)

catkin_python_setup()
catkin_package()
catkin_add_env_hooks(25.rocon-smartthings-bridge SHELLS sh DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY resources
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(PROGRAMS
  scripts/ninjablock_bridge.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
