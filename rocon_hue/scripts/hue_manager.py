#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_demos/license/LICENSE
#

import rospy
import rocon_device_msgs.msg as rocon_device_msgs


class HueManager():
    # color
    COLOR_H_RED = 65535
    COLOR_H_GREEN = 25500
    COLOR_H_BLUE = 46920
    MAX_HUE = 65535

    MAX_SAT = 255
    MAX_BRI = 255

    def __init__(self):
        self.subscriber = {}
        self.publisher = {}
        self.hues = {}
        self.hues_init = False

        self.publisher['set_hue_hsv'] = rospy.Publisher('set_hue_color_hsv', rocon_device_msgs.Hue, queue_size=10)
        self.publisher['set_hue_on'] = rospy.Publisher('set_hue_color_on', rocon_device_msgs.Hue, queue_size=10)

        self.subscriber['set_color'] = rospy.Subscriber('set_color', rocon_device_msgs.SetColor, self.update_color)
        self.subscriber['hue_list'] = rospy.Subscriber('hue_list', rocon_device_msgs.HueArray, self.hue_update)
        pass

    def hue_update(self, data):
        if not self.hues_init and len(data.hue_list):
            self.loginfo("Yes Hue...")
            self.hues_init = True
            for hue in data.hue_list:
                hue.state.sat = 0
                hue.state.bri = 0
                self.publisher['set_hue_hsv'].publish(hue)
            return
        elif not len(data.hue_list):
            self.hues_init = False
            self.hues = {}
            return

        for hue in data.hue_list:
            self.hues[hue.light_id] = hue

    def update_color(self, data):
        id = str(data.id)
        color = str(data.color)
        hue = sat = bri = 0
        on = True
        if color == "OFF":
            on = False
        elif color == "RED":
            hue = self.COLOR_H_RED
            sat = self.MAX_SAT
            bri = self.MAX_BRI
        elif color == "GREEN":
            hue = self.COLOR_H_GREEN
            sat = self.MAX_SAT
            bri = self.MAX_BRI
        elif color == "BLUE":
            hue = self.COLOR_H_BLUE
            sat = self.MAX_SAT
            bri = self.MAX_BRI
        elif color == "WHITE":
            hue = 0
            sat = 0
            bri = self.MAX_BRI
        else:
            self.loginfo("You selected [%s]-You must choose [RED, BLUE, GREEN, WHITE, OFF]" % color)
            return

        for hue_id in self.hues.keys():
            if not self.hues_init:
                self.loginfo("hue does not init, yet")
                return
            elif id == str(hue_id):
                if not on:
                    self.loginfo("%s hue turn off" % str(hue_id))
                    self.hues[hue_id].state.on = on
                    self.publisher['set_hue_on'].publish(self.hues[hue_id])
                else:
                    self.hues[hue_id].state.hue = hue
                    self.hues[hue_id].state.sat = sat
                    self.hues[hue_id].state.bri = bri
                    self.publisher['set_hue_hsv'].publish(self.hues[hue_id])
                    self.loginfo("hue set the %s" % color)
            rospy.sleep(0.1)

    def loginfo(self, msg):
        rospy.loginfo('Hue Manager : ' + str(msg))

    def spin(self):
        while not rospy.is_shutdown():
            rospy.sleep(1)
if __name__ == '__main__':

    rospy.init_node('office_hue_manager')
    manager = HueManager()
    manager.spin()
