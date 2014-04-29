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
        
        :raises AttributeError: One or more necessary parameters are not available.
    '''
    def __init__(self):
        if rospy.has_param('~arduino_ip'):
            self._arduino_ip = rospy.get_param('~arduino_ip')
        else:
            raise AttributeError("Couldn't find the 'arduino_ip' parameter.")
        if rospy.has_param('~door_ctrl_io_port'):
            self._door_ctrl_io_port = rospy.get_param('~door_ctrl_io_port')
        else:
            raise AttributeError("Couldn't find the 'door_ctrl_io_port' parameter.")

        self._door_status_pub = rospy.Publisher("door_status", std_msgs.String, latch=True)
        self._door_control_sub = rospy.Subscriber("door_ctrl", std_msgs.Bool, self._door_ctrl_cb)

        self._timer_mode = rospy.get_param('~timer_ctrl', False)
        if self._timer_mode:
            rospy.loginfo("Door control started in timer configuration.")
        else:
            rospy.loginfo("Door control started in open/close configuration.")

        # operation modes
        self._IDLING = 0
        self._OPENING = 1
        self._CLOSING = 2
        self._current_ctrl_mode = self._IDLING
        # required signal duration for opening the door in seconds
        self._open_door_trigger_duration = rospy.Duration(rospy.get_param('~open_door_trigger_duration', 5.0))
        # required signal duration for closing the door in seconds
        self._close_door_trigger_duration = rospy.Duration(rospy.get_param('~close_door_trigger_duration', 1.0))
        # required signal duration in timer mode in seconds
        self._open_door_trigger_duration_timer = \
            rospy.Duration(rospy.get_param('~open_door_trigger_duration_timer', 1.0))
        self._door_ctrl_triggered_time = rospy.Time.now()

    def _trigger_door_ctrl(self, ctrl_mode):
        try:
            resp = requests.post("http://" + str(self._arduino_ip)
                                 + "/arduino/digital/" + str(self._door_ctrl_io_port) + "/" + str(ctrl_mode))
        except requests.exceptions.RequestException as e:
            return False, str(e)

        if resp.status_code == requests.codes.ok:
            return True, str()
        else:
            return False, str(resp.content)

    def _door_ctrl_cb(self, msg):
        # TODO: use a separate thread for this
        status_msg = std_msgs.String()

        if self._timer_mode and not msg.data:
            status_msg.data = "Closing door operation is not available in timer mode."
            self._door_status_pub.publish(status_msg)
            rospy.logwarn(status_msg.data)
            return

        if msg.data and (self._current_ctrl_mode == self._OPENING):
            status_msg.data = "Cannot open door, since it is already opening."
            self._door_status_pub.publish(status_msg)
            rospy.logwarn(status_msg.data)
            return

        if not msg.data and (self._current_ctrl_mode == self._CLOSING):
            status_msg.data = "Cannot close door, since it is already closing."
            self._door_status_pub.publish(status_msg)
            rospy.logwarn(status_msg.data)
            return

        ctrl_mode = 1 # set output to high
        success, resp_msg = self._trigger_door_ctrl(ctrl_mode)
        self._door_ctrl_triggered_time = rospy.Time.now()

        if success:
            if msg.data:
                self._current_ctrl_mode = self._OPENING
                status_msg.data = "Door is opening."
            else:
                self._current_ctrl_mode = self._CLOSING
                status_msg.data = "Door is closing."
            self._door_status_pub.publish(status_msg)
            rospy.loginfo(status_msg.data)
        else:
            self._current_ctrl_mode = self._IDLING
            status_msg.data = "Failed to contact the door control! Error: " + str(resp_msg)
            self._door_status_pub.publish(status_msg)
            rospy.logwarn(status_msg.data)

    def spin(self):
        spin_rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            passed_time = rospy.Time.now() - self._door_ctrl_triggered_time
            if self._timer_mode:
                if (self._current_ctrl_mode == self._OPENING):
                    if passed_time > self._open_door_trigger_duration_timer:
                        success, status = self._trigger_door_ctrl(0) # set the output to low
                        status_msg = std_msgs.String()
                        self._current_ctrl_mode = self._IDLING
                        if success:
                            status_msg.data = "Idling."
                            self._door_status_pub.publish(status_msg)
                            rospy.loginfo(status_msg.data)
                        else:
                            status_msg.data = "Failed to contact the door control! Error: " + str(status)
                            self._door_status_pub.publish(status_msg)
                            rospy.logwarn(status_msg.data)
            else:
                if self._current_ctrl_mode == self._OPENING:
                    if passed_time > self._open_door_trigger_duration:
                        success, status = self._trigger_door_ctrl(0) # set the output to low
                        status_msg = std_msgs.String()
                        self._current_ctrl_mode = self._IDLING
                        if success:
                            status_msg.data = "Idling."
                            self._door_status_pub.publish(status_msg)
                            rospy.loginfo(status_msg.data)
                        else:
                            status_msg.data = "Failed to contact the door control! Error: " + str(status)
                            self._door_status_pub.publish(status_msg)
                            rospy.logwarn(status_msg.data)
                elif self._current_ctrl_mode == self._CLOSING:
                    if passed_time > self._close_door_trigger_duration:
                        success, status = self._trigger_door_ctrl(0)  # set the output to low
                        status_msg = std_msgs.String()
                        self._current_ctrl_mode = self._IDLING
                        if success:
                            status_msg.data = "Idling."
                            self._door_status_pub.publish(status_msg)
                            rospy.loginfo(status_msg.data)
                        else:
                            status_msg.data = "Failed to contact the door control! Error: " + str(status)
                            self._door_status_pub.publish(status_msg)
                            rospy.logwarn(status_msg.data)
            spin_rate.sleep()


if __name__ == '__main__':
    rospy.init_node("rocon_auto_door")
    try:
        auto_door = RoconAutoDoor()
        auto_door.spin()
    except Exception as e:
        rospy.logerr(str(e))
        pass

