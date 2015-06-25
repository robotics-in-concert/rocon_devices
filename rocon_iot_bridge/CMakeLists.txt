cmake_minimum_required(VERSION 2.8.3)
project(rocon_iot_bridge)
find_package(catkin REQUIRED COMPONENTS
  python-flask
)

catkin_python_setup()
catkin_package()
