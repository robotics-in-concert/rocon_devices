#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_devices/license/LICENSE
#
#################################################################################

import json
import requests
import rospy
import rocon_std_msgs.msg as rocon_std_msgs
import rocon_device_msgs.msg as rocon_device_msgs
from rocon_iot_bridge import RoconIOTBridge, Connector

class SmartThingsConnector(Connector):

    _API = {
        'UPDATE_CONFIG' : 'configuration',
        'GET_DEVICE_LIST' : 'get_device_list',
        'LOG':  'log',
        'RESET': 'reset'
    }

    def __init__(self):
        self._configuration_file = rospy.get_param('~target_configuration_file')

    def _load_configuration(self, filename):
        with open(filename) as f:
            config = json.load(f) 
        self._config = config

        if not 'access_token' in self._config:
            return False
        else:
            return True

    def init(self, config=None):

        if not self._load_configuration(self._configuration_file):
            return None, "Error while loading configuration : %s"%self._config

        self._endpoint_url = self._get_endpoint_url()

        if config:
            return self.request_configuration_update(config), "Success"
        else:
            return None, "No configuration is given"

    def close(self):
        return self._request_reset()

    def call_get_device_list(self):
        request_url = "%s/%s"%(self._endpoint_url, self._API['GET_DEVICE_LIST'])
        params = {}
        header =  {
          "Authorization": "Bearer %s" % self._config['access_token'],
        }
        resp = requests.get(url=request_url, params=params, headers=header)
        dev_resp = resp.json()

        devices = []
        for dev_type, devs in dev_resp.items():
            if devs:
                for d in devs:
                    dev_msg = self._convert_to_device_msg(dev_type, d)
                    devices.append(dev_msg)
        return devices

    def _convert_to_device_msg(self, d_type, d):
        m = rocon_device_msgs.Device()
        m.label = str(d['label'])
        m.uuid  = str(d['id'])
        m.type  = str(d_type)
        m.data  = []

        return m


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

    def convert_post_to_devices_msg(self, post):
        device = post['topic']
        device_data = post['data']
        dev_label, dev_type, dev_uuid = device.split("/")

        dev = rocon_device_msgs.Device()
        dev.label = str(dev_label)
        dev.type  = str(dev_type)
        dev.uuid  = str(dev_uuid)
        dev.data  = [rocon_std_msgs.KeyValue(str(key), str(value)) for key, value in device_data.items()]

        msg = rocon_device_msgs.Devices()
        msg.devices = []
        msg.devices.append(dev)

        return msg

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
