/*
 License: BSD
   https://raw.github.com/robotics-in-concert/rocon_devices/license/LICENSE
*/

#ifndef ROCON_RTSP_CAMERA_RELAY
#define ROCON_RTSP_CAMERA_RELAY

#include<ros/ros.h>
#include<opencv2/opencv.hpp>
#include<cv_bridge/cv_bridge.h>

#include<std_msgs/String.h>
#include<image_transport/image_transport.h>
#include<sensor_msgs/image_encodings.h>
#include<sensor_msgs/Image.h>
#include<sensor_msgs/CameraInfo.h>

namespace rocon {

namespace FoscamCommands {
  const std::string MOVE_UP           = "ptzMoveUp";
  const std::string MOVE_DOWN         = "ptzMoveDown";
  const std::string MOVE_LEFT         = "ptzMoveLeft";
  const std::string MOVE_TOP_LEFT     = "ptzMoveTopLeft";
  const std::string MOVE_TOP_RIGHT    = "ptzMoveTopRight";
  const std::string MOVE_BOTTOM_LEFT  = "ptzMoveBottomLeft";
  const std::string MOVE_BOTTOM_Right = "ptzMoveBottomRight";
}

class RoconFoscamControl {
  public:
    RoconFoscamControl(ros::NodeHandle& n);
    ~RoconFoscamControl();

    bool init();
    void spin();

  protected:
  private:
    std::string user;
    std::string password;
}
}
