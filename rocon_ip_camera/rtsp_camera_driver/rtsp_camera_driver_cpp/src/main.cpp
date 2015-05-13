#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <iostream>
 
using namespace cv;
using namespace std;
 
int main (int argc, char *argv[]) 
{ 
     ros::init(argc, argv, "rtsp_camera");
     
     VideoCapture vcap; 
     Mat image;

     ros::NodeHandle nh;
     image_transport::ImageTransport img_trs(nh);
     image_transport::Publisher img_pub;
     img_pub = img_trs.advertise("~image", 1);
     ros::Publisher chatter_pub = nh.advertise<sensor_msgs::CameraInfo>("~camera_info", 1);

     cv_bridge::CvImage cv_img;
     sensor_msgs::CameraInfo camera_info
     
     string user = "yujin";
     string password = "akqjqtk";
     string camera_address = "192.168.10.101:88";
     string video_stream_api = "/videoSub";

     string video_stream_address = 
          "rtsp://" + user + ":" + password + "@" + camera_address + video_stream_api;


     ROS_INFO("%s",video_stream_address.c_str());
     if (!vcap.open(video_stream_address)) { 
          ROS_ERROR("%s","Error opening video stream or file");
          return -1; 
     }
 
     while(ros::ok()) { 
          if(!vcap.read(image)) { 
               ROS_WARN("%s","No frame");
               waitKey();
          }
          cv_img.encoding = sensor_msgs::image_encodings::BGR8;
          cv_img.image = image;
          img_pub.publish(cv_img.toImageMsg());

          ROS_INFO("image.rows [%d]",image.rows);
          ROS_INFO("image.cols [%d]",image.rows);
          ROS_INFO("image.step [%d]",image.step);
          ROS_INFO("image.steps [%d]",image.steps);


          imshow("Output Window", image); 
          waitKey(1); 
     } 
     vcap.release();
     return 0;
}

