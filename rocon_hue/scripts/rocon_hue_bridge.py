#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_devices/license/LICENSE
#
#################################################################################

#sys
import threading
import socket
from urllib2 import urlopen, URLError, HTTPError

#ros
import rospy
from rocon_device_msgs.msg import HueState, Hue, HueArray

#phue
from rocon_hue import Bridge
from rocon_hue import PhueRegistrationException, PhueException


class Rocon_Hue():
    def __init__(self):
        self.name = 'ros_hue'
        self.ip = None
        rospy.init_node(self.name)
        
        self.bridge = Bridge()
        
        if rospy.has_param('~hue_ip'):
            self.ip = rospy.get_param('~hue_ip')
            print self.ip
        else:
            print 'No argument hue ip'
            return
        
        self.bridge.set_ip_address(self.ip)
        self.hue_list_publisher = rospy.Publisher("hue_list", HueArray, latch=False)
        rospy.Subscriber('set_hue_color_on', Hue, self.set_hue_color_on)
        rospy.Subscriber('set_hue_color_xy', Hue, self.set_hue_color_xy)
        rospy.Subscriber('set_hue_color_hsv', Hue, self.set_hue_color_hsv)
        rospy.Subscriber('set_hue_color_ct', Hue, self.set_hue_color_ct)
        rospy.Subscriber('set_hue_color_mode', Hue, self.set_hue_color_mode)

        self.checker_th = threading.Thread(target=self.hue_checker)
        self.is_checking = True
        self.checker_th.start()

    def hue_checker(self):
        while self.is_checking and not rospy.is_shutdown():
            if self.ping_checker():
                if not self.bridge.is_connect:
                    try:
                        self.bridge.connect()
                    except PhueRegistrationException as e:
                        self.logwarn(e.message)
                    except PhueException as e:
                        self.logwarn(e.message)
                    else:
                        self.loginfo("bridge connect")
                        self.bridge.is_connect = True
                else:
                    self.bulb_checker()
            else:
                self.bridge.set_ip_address(self.ip)
                self.bridge.is_connect = False
                self.loginfo("bridge not connect %s"% self.ip)
                pass
            rospy.sleep(5)

    def ping_checker(self):
        time_out = 2  # 3secend
        socket.setdefaulttimeout(time_out)  # timeout in seconds
        try:
            url = "http://" + str(self.ip)
            urlopen(url, timeout=time_out)

        except HTTPError, e:
            self.logwarn('The server can not fulfill the request. Reason: %s' % str(e.code))
            return False
        except URLError, e:
            self.logwarn('failed to reach a server. Reason: %s' % str(e.reason))
            return False
        except socket.timeout, e:
            self.logwarn('failed socket timeout. Reason: %s' % str(e))
            return False
        except Exception, e:
            self.logwarn('failed. Reason:%s' % str(e))
        else:
            return True

    def bulb_checker(self):
        light_ids = self.bridge.get_light_objects(mode='id')
        hues = HueArray()
        for light_id in light_ids:
            state = self.bridge.get_light(light_id)
            if state is not "":
                hue = Hue()
                hue.light_id = light_id
                hue.name = state['name']
                hue.state.on = state['state']['on']
                hue.state.xy = state['state']['xy']
                hue.state.hue = state['state']['hue']
                hue.state.sat = state['state']['sat']
                hue.state.bri = state['state']['bri']
                hue.state.reachable = state['state']['reachable']
                hues.hue_list.append(hue)
        self.hue_list_publisher.publish(hues)

    def set_hue_color_on(self, data):
        if self.bridge.is_connect:
            state = {}
            state["on"] = data.state.on
            self.bridge.set_light([data.light_id], state)

    def set_hue_color_xy(self, data):
        if self.bridge.is_connect:
            state = {}
            state["xy"] = data.state.xy
            self.bridge.set_light([data.light_id], state)

    def set_hue_color_hsv(self, data):
        if self.bridge.is_connect:
            state = {}
            state["hue"] = data.state.hue
            state["bri"] = data.state.bri
            state["sat"] = data.state.sat
            self.bridge.set_light([data.light_id], state)

    def set_hue_color_ct(self, data):
        if self.bridge.is_connect:
            state = {}
            state["ct"] = data.state.ct
            self.bridge.set_light([data.light_id], state)

    def set_hue_color_mode(self, data):
        if self.bridge.is_connect:
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
            self.bridge.set_light([data.light_id], state)

    def loginfo(self, msg):
        rospy.loginfo('Rocon Hue : ' + str(msg))

    def logwarn(self, msg):
        rospy.logwarn('Rocon Hue : ' + str(msg))

    def spin(self):
        if self.ip is not None:
            while not rospy.is_shutdown():
                try:
                    rospy.sleep(0.01)
                except:
                    break
            self.is_checking = False
            self.checker_th.join(1)
        else:
            print ('Rocon Hue : Must set hue ip')

if __name__ == '__main__':
    rh = Rocon_Hue()
    rh.spin()
