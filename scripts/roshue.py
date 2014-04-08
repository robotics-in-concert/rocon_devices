#!/usr/bin/env python
#sys
import random
import thread
import time
#ros
import rospy
from std_msgs.msg import Empty
from hue_controller.msg import RequestHue
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
        rospy.Subscriber('/set_hue_color_xy', Hue, self.set_hue_color_xy)
        rospy.Subscriber('/set_hue_color_hsv', Hue, self.set_hue_color_xy)
        rospy.Subscriber('/set_hue_color_ct', Hue, self.set_hue_color_xy)
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
                    hue.state.x = light.xy[0]
                    hue.state.y = light.xy[1]
                    hue.state.hue = light._hue or 0
                    hue.state.sat = light._saturation or 0
                    hue.state.bri = light._brightness or 0
                    hue.state.mode = hue.state.NONE or light._effect or light._alert 
                    hue.state.transitiontime = light.transitiontime or 0
                    hues.hue_list.append(hue)
                    print hues
                    self.hue_list_publisher.publish(hues)

    def set_hue_color_xy(self, data):
        lights = self.bridge.get_light_objects()
        for light in lights:
            if data.light_id == light.light_id:
                x = data.state.x
                y = data.state.y
                light.xy = [x, y]
                print "change the xy [%f, %f]" % (x, y)
    
    def set_hue_color_mode(self, data):
        lights = self.bridge.get_light_objects()
        for light in lights:
            x = data.x
            y = data.y
            light.xy = [x, y]
            print "change the xy [%f, %f]" % (x, y)
        
    def req_on_lights(self, data):
        lights = self.bridge.get_light_objects()
        for light in lights:
            light.on = True

    def req_off_lights(self, data):
        lights = self.bridge.get_light_objects()
        for light in lights:
            light.on = False

    def req_random_color(self, data):
        lights = self.bridge.get_light_objects()

        for light in lights:
            light.brightness = 254
            x = random.random()
            y = random.random()
            print "change the [%f, %f]" % (x, y)

    def req_crazy_random_color(self, data):
        for k in range(10):
            lights = self.bridge.get_light_objects()
            for light in lights:
                if light.light_id == 2:
                    light.brightness = 254
                    x = random.random()
                    y = random.random()
                    print "change the [%f, %f]" % (x, y)
                    light.xy = [x, y]

    def spin(self):
        while not rospy.is_shutdown():
            rospy.sleep(0.01)

if __name__ == '__main__':
    rh = RosHue()
    rh.spin()
