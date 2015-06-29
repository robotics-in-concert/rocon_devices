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


class NinjaBlockConnector(Connector):

    _API = {
        'UPDATE_CONFIG': 'configuration',
        'GET_DEVICE_LIST': 'get_device_list',
        'LOG':  'log',
        'RESET': 'reset'
    }

    def __init__(self):
        self._configuration_file = rospy.get_param('~target_configuration_file')
        self._devices_msgs = []

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
            return None, "Error while loading configuration : %s" % self._config

        self._endpoint_url = self._get_endpoint_url()

        if config:
            return self.request_configuration_update(config), "Success"
        else:
            return None, "No configuration is given"

    def close(self):
        return self._request_reset()

    def call_get_device_list(self):
        return self._get_device_list()

    def _convert_to_device_msgs(self, device_raw_data):
        msgs = []
        for dev_id in device_raw_data:
            device = device_raw_data[dev_id]
            if device['has_subdevice_count'] is 1:
                for sdev_id in device['subDevices']:
                    sub_device = device['subDevices'][sdev_id]
                    dev = rocon_device_msgs.Device()
                    dev.label = sub_device['shortName']
                    dev.type = sub_device['category']
                    dev.uuid = sub_device['data']
                    dev.data = []
                    dev.data.append(rocon_std_msgs.KeyValue('guid', str(dev_id)))
                    msgs.append(dev)
            else:
                dev = rocon_device_msgs.Device()
                dev.label = device['shortName'].replace(' ', '_').lower()
                dev.type = device['device_type']
                dev.data.append(rocon_std_msgs.KeyValue('guid', str(dev_id)))
                msgs.append(dev)
        return msgs

    def _request_reset(self):
        #delete callback function
        return True

    def request_configuration_update(self, config):
        #get device list
        self._get_device_list()
        #get device callback
        #set callback by put or post type
        return True

    def convert_post_to_devices_msg(self, post):
        device_data = post
        dev_msgs = rocon_device_msgs.Devices()
        for msg in self._devices_msgs:
            if msg.uuid == device_data['DA']:
                dev_msgs.devices.append(msg)
        return dev_msgs

    def _get_device_list(self):
        devices_msgs = []
        request_url = "%s/%s?user_access_token=%s" % (self._endpoint_url, 'devices', self._config['access_token'])
        resp = requests.get(url=request_url)
        if resp is not None and resp.status_code == 200:
            dev_resp = json.loads(str(resp.text))
            if 'data' in dev_resp.keys():
                devices_msgs = self._convert_to_device_msgs(dev_resp['data'])
        self._devices_msgs = devices_msgs
        return devices_msgs

    def _get_endpoint_url(self):
        endpoint_url = 'https://%s' % str(self._config['api'])
        return endpoint_url


if __name__ == '__main__':
    rospy.init_node('ninjablock_bridge')

    local_address = rospy.get_param('~local_address')
    local_port = rospy.get_param('~local_port')
    global_address = rospy.get_param('~global_address')
    global_port = rospy.get_param('~global_port')

    connector = NinjaBlockConnector()
    bridge = RoconIOTBridge(local_address, local_port, global_address, global_port, connector)

    bridge.loginfo("Initilialised")
    bridge.spin()
    bridge.loginfo("Bye Bye")
