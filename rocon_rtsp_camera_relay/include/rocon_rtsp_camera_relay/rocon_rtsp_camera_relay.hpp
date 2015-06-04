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

namespace rocon {

class RoconRtspCameraRelay {
  public:
    RoconRtspCameraRelay(ros::NodeHandle& n);
    ~RoconRtspCameraRelay();

    bool init(const std::string video_stream_url);
    bool reset(const std::string video_stream_url);

    void spin();
  
  protected:
    void convertCvToRosImg(const cv::Mat& mat, sensor_msgs::Image& ros_img, sensor_msgs::CameraInfo& ci);

  private:
    cv::VideoCapture                vcap_;
    std::string                     video_stream_address_;
    std::string                     status_;

    image_transport::Publisher pub_video_;
    ros::Publisher pub_camera_info_;
    ros::Publisher pub_status_;
    ros::NodeHandle nh_;
};
}

#endif
