#!/usr/bin/env python
#sys
import random
import thread
import time
#ros
import rospy
from std_msgs.msg import Empty

#phue
import hue_controller


class RosHue():
    def __init__(self):
        self.name = 'ros_hue'
        self.bridge = hue_controller.Bridge()

        self.ip = self.bridge.get_ip_address(set_result=True)
        self.bridge.set_ip_address(self.ip)
        self.bridge.connect()

        thread.start_new_thread(self.checker_hue())

        rospy.init_node(self.name)
        rospy.Subscriber('/random_color', Empty, self.req_random_color)
        rospy.Subscriber('/crazy_random_color', Empty, self.req_crazy_random_color)
        rospy.Subscriber('/off_lights', Empty, self.req_off_lights)
        rospy.Subscriber('/on_lights', Empty, self.req_on_lights)
        pass

    """
    def hue_checker(self):
        while not rospy.is_shutdown():
            rospy.sleep(0.1)

            ip = self.bridge.check_validation()
            start = time.time()

            end = time.time()
            if ip:
                print "Connecting: %s %f[ms]" % (ip, end-start)
            else:
                print "Disconnecting %f[ms]" % (end-start)
     """
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
