#!/usr/bin/env python

#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_devices/license/LICENSE
#

import requests
import rospy
import std_msgs.msg as std_msgs

class RoconAutoDoor(object):
    '''
        Communicates with an Arduino Yun through its RESTful API to open/close automated doors
        
        :raises Exception: One or more necessary parameters are not available.
    '''
    def __init__(self):
        if rospy.has_param('~arduino_ip'):
            self._arduino_ip = rospy.get_param('~arduino_ip')
        else:
            raise Exception("Couldn't find the 'arduino_ip' parameter! Aborting.")
        if rospy.has_param('~door_ctrl_io_port'):
            self._door_ctrl_io_port = rospy.get_param('~door_ctrl_io_port')
        else:
            raise Exception("Couldn't find the 'door_ctrl_io_port' parameter! Aborting.")
        self._door_status_pub = rospy.Publisher("door_status", std_msgs.String, latch=True)
        self._door_control_sub = rospy.Subscriber("door_control", std_msgs.Bool, self._door_ctrl_cb)

    def _door_ctrl_cb(self, msg):
        if msg.data:
            ctrl_mode = 1 #open the door
        else:
            ctrl_mode = 0 # close the door
        resp = requests.post("http://" + str(self._arduino_ip)
                             + "/arduino/digital/" + str(self._door_ctrl_io_port) + "/" + str(ctrl_mode))

        status_msg = std_msgs.String()
        if resp.status_code == requests.codes.ok:
            if msg.data:
                status_msg.data = "Door is opening."
            else:
                status_msg.data = "Door is closing."
            self._door_status_pub.publish(status_msg)
            rospy.loginfo(status_msg.data)
        else:
            status_msg = "Failed to contact the door control! Error code: " + str(resp.status_code)
            self._door_status_pub.publish(status_msg)
            rospy.logerr(status_msg.data)

    def spin(self):
      rospy.spin()

if __name__ == '__main__':
    rospy.init_node("rocon_auto_door")

    try:
        auto_door = RoconAutoDoor()
        auto_door.spin()
    except Exception as e:
        rospy.logerr(str(e))
        pass

