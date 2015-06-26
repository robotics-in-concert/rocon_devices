#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_devices/license/LICENSE
#
#################################################################################

import json
import requests
import rospy
from rocon_iot_bridge import RoconIOTBridge, Connector

class SmartThingsConnector(Connector):

    _API = {
        'UPDATE_CONFIG' : 'configuration',
        'GET_DEVICE_LIST' : 'get_device_list',
        'RESET': 'reset'
    }

    def __init__(self):
        self._configuration_file = rospy.get_param('~target_configuration_file')
        self._load_configuration(self._configuration_file)

    def _load_configuration(self, filename):
        with open(filename) as f:
            config = json.load(f) 
        self._config = config

    def init(self, config=None):
        self._endpoint_url = self._get_endpoint_url()

        if config:
            return self.request_configuration_update(config)
        else:
            return None 

    def close(self):
        return self._request_reset()

    def call_get_device_list(self):
        request_url = "%s/%s"%(url, self._API['GET_DEVICE_LIST'])
        params = {}
        header =  {
          "Authorization": "Bearer %s" % self._config['access_token'],
        }
        resp = requests.get(url=request_url, params=params, headers=header)

    def _request_reset(self):
        request_url = "%s/%s"%(self._endpoint_url, self._API['RESET'])
        params = {}
        header =  {
          "Authorization": "Bearer %s" % self._config['access_token'],
        }
        resp = requests.put(url=request_url, params={}, headers=header)
        # Return true or false
        return resp

    def request_configuration_update(self, config):
        request_url = "%s/%s"%(self._endpoint_url, self._API['UPDATE_CONFIG'])
        header = { "Authorization": "Bearer %s" % self._config['access_token'],}
        resp = requests.put(url=request_url, params=config, headers=header)
        
        # Return true or false
        return resp

    def convert_post_to_msg(self, post):
        pass                                    

    def _get_endpoint_url(self):
        endpoints_url = self._config['api']
        endpoints_paramd = {
            "access_token": self._config['access_token']
        }

        endpoints_response = requests.get(url=endpoints_url, params=endpoints_paramd)
        end_url = endpoints_response.json()[0]['url']
        endpoint_url = 'http://%s%s'%(self._config['api_location'], end_url)

        return endpoint_url


if __name__ == '__main__':
    rospy.init_node('smartthings_bridge')
    
    local_address = rospy.get_param('~local_address')
    local_port    = rospy.get_param('~local_port')
    global_address = rospy.get_param('~global_address')
    global_port    = rospy.get_param('~global_port')

    connector = SmartThingsConnector()
    bridge = RoconIOTBridge(local_address, local_port, global_address, global_port, connector)

    bridge.loginfo("Initilialised")
    bridge.spin()
    bridge.loginfo("Bye Bye")
