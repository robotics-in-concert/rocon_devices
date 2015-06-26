#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_devices/license/LICENSE
#
#################################################################################

from abc import ABCMeta, abstractmethod

class Connector(object):
    """
      Abstract base class that defines the API of a iot platform connector.
    """
    __metaclass__ = ABCMeta

    @abstractmethod
    def init(self):
        """
          Initialises connector.  
        """
        pass

    @abstractmethod
    def close(self):
        """
        """
        pass

    @abstractmethod
    def call_get_device_list(self):
        """
          Request target engine to receive all available device list
        
          :returns: List of devices
          :rtypes: rocon_device_msgs.msg.Devices 
        """
        pass

    @abstractmethod
    def convert_post_to_msg(self, post):
        """
          It converts te target engine posts device events into ros msg
          
          :param post: device events

          :returns: ROS formatted device events
          :rtypes: rocon_device_msgs.msg.Devices
        """
        pass

    @abstractmethod
    def request_configuration_update(self, config):
        """
          requests target engine to update its configuration 
          :param config: Configuration(e.g server addr and port for device event post) 
          :type config: dict

          :returns: Success or fail
          :rtype: bool
        """
        pass
