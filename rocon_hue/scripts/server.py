#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_devices/license/LICENSE
#
#################################################################################

# sys
import sys

# ros
import rospy
from rocon_hue import RoconBridge

if __name__ == '__main__':
    rospy.init_node('hue_bridge')
    if rospy.has_param('~hue_ip'):
        hue_ip = rospy.get_param('~hue_ip')
        bridge = RoconBridge(hue_ip=hue_ip)
        bridge.spin()
    else:
        rospy.logwarn('No argument hue ip')
        sys.exit()
