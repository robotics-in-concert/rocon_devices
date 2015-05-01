#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_devices/license/LICENSE
#
#################################################################################

# ros
import rospy
# open cv
import cv2

if __name__ == '__main__':
    rtsp_adress = "rtsp://yujin:akqjqtk@192.168.10.141:88/videoSub"
    vcap = cv2.VideoCapture()
    if not vcap.open(rtsp_adress):
        pass
    else:
        while not rospy.is_shutdown():
            ret, image = vcap.read()
            if ret:
                cv2.imshow("Output Window", image)
            cv2.waitKey(10)
    vcap.release()
    print 'ByeBye'
