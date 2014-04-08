#!/usr/bin/python
#sys
import random

#ros
import rospy
from hue_controller.msg import HueState
from hue_controller.msg import Hue
from hue_controller.msg import HueArray


class RandomColor():
    def __init__(self):
       print "Random Hue"
       rospy.init_node("random_color")
		self.hue_color_on_publisher = rospy.Publisher("/set_hue_color_on", Hue)
		self.hue_color_xy_publisher = rospy.Publisher("/set_hue_color_xy", Hue)
		self.hue_color_hsv_publisher = rospy.Publisher("/set_hue_color_hsv", Hue)
		self.hue_color_ct_publisher = rospy.Publisher("/set_hue_color_ct", Hue)
		self.hue_color_mode_publisher = rospy.Publisher("/set_hue_color_mode", Hue)
		rospy.Subscriber("/hue_list", HueArray, self.set_valid_hues)
		self.hue_list = []

	def set_valid_hues(self, data):
		self.hue_list = data.hue_list
		pass
	
	def spin(self):
		while not rospy.is_shutdown():
			for hue in self.hue_list:
				on = random.randint(0,1)
				if on:
					hue.state.on = True
				else:
					hue.state.on = False
				print "Set the color on: [%s]"% str(hue.state.on)
				self.hue_color_on_publisher.publish(hue)
				
				"""
				hue.state.xy = [random.random(), random.random()]
				print "Set the color xy: [%f, %f]"%(hue.state.xy[0], hue.state.xy[1])
				self.hue_color_xy_publisher.publish(hue)
				"""

				"""
				hue.state.hue = random.randint(0,65535)
				hue.state.sat = random.randint(0,255)
				hue.state.bri = random.randint(0,255)
				print "Set the color hsv: [%d, %d, %d]"%(hue.state.hue, hue.state.sat, hue.state.bri)
				self.hue_color_hsv_publisher.publish(hue)
				"""
				
				"""
				hue.state.ct = random.randint(153,500)
				print "Set the color ct: [%d]"%(hue.state.ct)
				self.hue_color_ct_publisher.publish(hue)
				"""
				"""
				mode = random.randint(0,2)
				if mode == 0:
					hue.state.mode = HueState().NONE
					print "Set the color mode: [NONE]"
				elif mode == 1:
					hue.state.mode = HueState().LSELECT
					print "Set the color mode: [LSELECT]"
				elif mode == 2:
					hue.state.mode = HueState().COLOR_LOOP
					print "Set the color mode: [COLOR_LOOP]"
				
				self.hue_color_mode_publisher.publish(hue)
				"""
			rospy.sleep(3.0)

if __name__ == '__main__':
	random_color = RandomColor()
	random_color.spin()
