cmake_minimum_required(VERSION 2.8.3)
project(rocon_iot_bridge)
find_package(catkin REQUIRED)

catkin_python_setup()
catkin_package()

catkin_add_env_hooks(25.rocon-iot-bridge SHELLS sh DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)

