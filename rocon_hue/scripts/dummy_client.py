#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_devices/license/LICENSE
#
#################################################################################
# sys
import random

# ros
import rospy
from rocon_device_msgs.msg import HueState, Hue, HueArray


class Test():

    def __init__(self):
        rospy.init_node("dummy_hue_client")
        rospy.loginfo("dummy_hue_client start")
        self.set_hue_publisher = rospy.Publisher("/set_hue", Hue, queue_size=10)
        rospy.Subscriber("/list_hue", HueArray, self.set_valid_hues)
        self.hue_list = []
        self.pre_define_color = ['WHITE', 'RED', 'GREEN', 'BLUE', 'ORANGE']

    def set_valid_hues(self, data):
        self.hue_list = data.hue_list

    def spin(self):
        while not rospy.is_shutdown():
            color = random.randint(0, len(self.pre_define_color) - 1)
            for k in [1, 2, 3]:
                hue = Hue()
                hue.state.color = str(self.pre_define_color[color])
                hue.light_id = k
                self.set_hue_publisher.publish(hue)
                rospy.sleep(0.1)
                rospy.loginfo(str(k) + "  change: " + str(self.pre_define_color[color]))
            rospy.sleep(8.0)
            """
            for hue in self.hue_list:
                if hue.state.reachable:
                    hue.state.hue = random.randint(0, 65535)
                    hue.state.sat = random.randint(0, 255)
                    hue.state.bri = random.randint(0, 255)
                    rospy.loginfo('change %s light: random color hsv [%d, %d, %d]' % (str(hue.light_id), hue.state.hue, hue.state.sat, hue.state.bri))
                    self.set_hue_publisher.publish(hue)
            rospy.sleep(5.0)

            for hue in self.hue_list:
                if hue.state.reachable:
                    color = random.randint(0, len(self.pre_define_color)-1)
                    hue.state.color = str(self.pre_define_color[color])
                    rospy.loginfo('change %s light: pre defined color [%s]' % (str(hue.light_id), str(self.pre_define_color[color])))
                    self.set_hue_publisher.publish(hue)
            rospy.sleep(5.0)
            """
if __name__ == '__main__':
    test = Test()
    test.spin()
