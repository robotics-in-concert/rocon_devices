#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_devices/license/LICENSE
#
#################################################################################

# ros
import rospy
import cv_bridge

# open cv
import cv2
import sensor_msgs.msg as sensor_msgs

if __name__ == '__main__':
    user = rospy.get_param('~user','user')
    password = rospy.get_param('~password','password')
    camera_address = rospy.get_param('~camera_ip','localhost:8080')
    camera_api = rospy.get_param('~camera_api','video_stream')
    
    rospy.init_node('rtsp_camera_driver', anonymous=True)
    bridge = cv_bridge.CvBridge()
    image_pub = rospy.Publisher("~image", sensor_msgs.Image)
    camera_info_pub = rospy.Publisher("~camera_info", sensor_msgs.CameraInfo)

    rtsp_adress = "rtsp://%s:%s@%s/%s" % (user, password, camera_address, camera_api)
    vcap = cv2.VideoCapture()
    camera_info = sensor_msgs.CameraInfo()

    if not vcap.open(rtsp_adress):
        pass
    else:
        while not rospy.is_shutdown():
            ret, cv_image = vcap.read()
            if ret:
                cv2.imshow("Output Window", cv_image)
                image = bridge.cv2_to_imgmsg(cv_image, "bgr8")
                camera_info.header = image.header
                camera_info.width = image.width
                camera_info.height = image.height
                image_pub.publish(image)
                camera_info_pub.publish(camera_info)
            cv2.waitKey(1)
    vcap.release()
    print 'ByeBye'
