cmake_minimum_required(VERSION 2.8.3)
project(rocon_rtsp_camera_relay)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge
  image_transport
  std_msgs
  sensor_msgs
)

find_package(OpenCV)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES rocon_rtsp_camera_relay
  CATKIN_DEPENDS roscpp cv_bridge image_transport sensor_msgs std_msgs
  DEPENDS OpenCV
)

catkin_add_env_hooks(25.rocon-rtsp-camera-relay SHELLS sh DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(rocon_rtsp_camera_relay
  src/rocon_rtsp_camera_relay.cpp
)

target_link_libraries(rocon_rtsp_camera_relay
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
add_dependencies(rocon_rtsp_camera_relay rocon_rtsp_camera_relay_generate_messages ${catkin_EXPORTED_TARGETS})

add_executable(rocon_rtsp_camera_relay_node src/main.cpp)
target_link_libraries(rocon_rtsp_camera_relay_node rocon_rtsp_camera_relay)
add_dependencies(rocon_rtsp_camera_relay_node rocon_rtsp_camera_relay)

#############
## Install ##
#############

install(TARGETS rocon_rtsp_camera_relay rocon_rtsp_camera_relay_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY rapps
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
