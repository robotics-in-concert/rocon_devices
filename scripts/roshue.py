#!/usr/bin/env python
#sys
import random
import thread
import time

#ros
import rospy
from std_msgs.msg import Empty
from hue_controller.msg import HueState
from hue_controller.msg import Hue
from hue_controller.msg import HueArray

#phue
import hue_controller


class RosHue():
    def __init__(self):
        self.name = 'ros_hue'
        self.bridge = hue_controller.Bridge()

        self.ip = self.bridge.get_ip_address(set_result=True)
        self.bridge.set_ip_address(self.ip)
        self.bridge.connect()

        rospy.init_node(self.name)
        self.hue_list_publisher = rospy.Publisher("/hue_list", HueArray, latch=False)
        rospy.Subscriber('/set_hue_color_on', Hue, self.set_hue_color_on)
        rospy.Subscriber('/set_hue_color_xy', Hue, self.set_hue_color_xy)
        rospy.Subscriber('/set_hue_color_hsv', Hue, self.set_hue_color_hsv)
        rospy.Subscriber('/set_hue_color_ct', Hue, self.set_hue_color_ct)
        rospy.Subscriber('/set_hue_color_mode', Hue, self.set_hue_color_mode)
        thread.start_new_thread(self.hue_checker())

    def hue_checker(self):
        while not rospy.is_shutdown():
            rospy.sleep(0.5)
            lights = self.bridge.get_light_objects()
            hues = HueArray()
            for light in lights:
                if light.reachable:
                    hue = Hue()
                    hue.light_id = light.light_id
                    hue.name = light.name
                    hue.state.on = light.on
                    hue.state.xy = light.xy
                    hue.state.hue = light._hue or 0
                    hue.state.sat = light._saturation or 0
                    hue.state.bri = light._brightness or 0
                    hue.state.mode = hue.state.NONE or light._effect or light._alert
                    hue.state.transitiontime = light.transitiontime or 0
                    hues.hue_list.append(hue)
                    self.hue_list_publisher.publish(hues)

    def set_hue_color_on(self, data):
        state = {}
        state["on"] = data.state.on
        self.bridge.set_light([data.light_id], state)

    def set_hue_color_xy(self, data):
        state = {}
        state["xy"] = data.state.xy
        self.bridge.set_light([data.light_id], state)

    def set_hue_color_hsv(self, data):
        state = {}
        state["hue"] = data.state.hue
        state["bri"] = data.state.bri
        state["sat"] = data.state.sat
        self.bridge.set_light([data.light_id], state)

    def set_hue_color_ct(self, data):
        state = {}
        state["ct"] = data.state.ct
        self.bridge.set_light([data.light_id], state)

    def set_hue_color_mode(self, data):
        state = {}
        if data.state.mode == HueState().NONE:
            state["alert"] = data.state.mode
            state["effect"] = data.state.mode
        elif data.state.mode == HueState().COLOR_LOOP:
            state["alert"] = data.state.NONE
            state["effect"] = data.state.mode
        elif data.state.mode == HueState().SELECT or data.state.mode == HueState().LSELECT:
            state["alert"] = data.state.mode
            state["effect"] = data.state.NONE
        else:
            state["alert"] = data.state.mode
            state["effect"] = data.state.mode
        print state
        self.bridge.set_light([data.light_id], state)

    def spin(self):
        while not rospy.is_shutdown():
            rospy.sleep(0.01)

if __name__ == '__main__':
    rh = RosHue()
    rh.spin()
