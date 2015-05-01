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
     ros::init(argc, argv, "rtsp_camera_driver");
     
     VideoCapture vcap; 
     Mat image;

     ros::NodeHandle nh;
     image_transport::ImageTransport img_trs(nh);
     image_transport::Publisher img_pub;
     img_pub = img_trs.advertise("rtsp_camera", 1);

     cv_bridge::CvImage cv_img;
     string videoStreamAddress = 
          "rtsp://yujin:akqjqtk@192.168.10.141:88/videoSub";  
     ROS_INFO("%s",videoStreamAddress.c_str());
     if (!vcap.open(videoStreamAddress)) { 
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

          imshow("Output Window", image); 
          waitKey(1); 
     } 
     vcap.release();
     return 0;
}

