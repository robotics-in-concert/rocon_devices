#!/usr/bin/python


"""
#from hue_controller import Bridge
from hue_controller import Bridge
import random

b = Bridge() # Enter bridge IP here.

#If running for the first time, press button on bridge and run with b.connect() uncommented
#b.connect()

lights = b.get_light_objects()

for light in lights:
light.brightness = 254
light.xy = [random.random(),random.random()]
"""
import random
import rospy
from hue_controller.msg import Hue
from hue_controller.msg import HueArray


class RandomColor():
	def __init__(self):
		print "Random Hue"
		rospy.init_node("random_color")
		self.hue_publisher = rospy.Publisher("/set_hue_color_xy", Hue)
		rospy.Subscriber("/hue_list", HueArray, self.set_valid_hues)
		self.hue_list = []

	def set_valid_hues(self, data):
		self.hue_list = data.hue_list
		print self.hue_list
		pass
	
	def spin(self):
		while not rospy.is_shutdown():
			for hue in self.hue_list:
				hue.state.x = random.random()
				hue.state.y = random.random()
				print "Set the color: [%f, %f]"%(hue.state.y, hue.state.y)
				self.hue_publisher.publish(hue)		
			rospy.sleep(0.5)

if __name__ == '__main__':
	random_color = RandomColor()
	random_color.spin()
	