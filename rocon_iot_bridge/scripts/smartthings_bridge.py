#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_devices/license/LICENSE
#
#################################################################################

import rospy
from rocon_iot_bridge import RoconIOTBridge, Connector

class SmartThingsConnector(Connector):

    _API = {
        'UPDATE_CONFIG' : 'update_config',
        'GET_DEVICE_LIST' : 'get_device_list',
    }

    def __init__(self):
        self._address = rospy.get_param('~target_address')
        self._port = rospy.get_param('~target_port')
        self._configuration_file = rospy.get_param('~target_configuration_file') 
        self._load_configuration()


    def _load_configuration(self):
        with open(filename) as f:
            config = json.load(f) 
        self._config = config

    def init(self, config=None):
        self._endpoint_url = self._get_endpoint_url()

        if config:
            self.request_configuration_update(config)

    def call_get_device_list(self):
        request_url = "%s/%s"%(url, self._API['GET_DEVICE_LIST'])
        params = {}
        header =  {
          "Authorization": "Bearer %s" % access_token,
        }
        resp = requests.get(url=request_url, params=params, headers=header)
        print(resp.json())

    def request_configuration_update(self, config):
        address = config['address']
        port = config['port']

        request_url = "%s/%s"%(self._endpoint_url, self._API['UPDATE_CONFIG'])
        header = { "Authorization": "Bearer %s" % access_token,}

        resp = requests.get(url=request_url, params=config, headers=header)
        print(resp.json())

        # Return true or false
        return

    def convert_post_to_msg(self, post):
        pass                                    

    def _get_endpoint_url(self):
        endpoints_url = self._config['api']
        endpoints_paramd = {
            "access_token": self._config['access_token']
        }

        endpoints_response = requests.get(url=endpoints_url, params=endpoints_paramd)
        end_url = endpoints_response.json()[0]['url']
        endpoint_url = 'http://%s%s'%(api_location, end_url)

        return endpoint_url


if __name__ == '__main__':
    rospy.init_node('smartthings_bridge')

    connector = SmartThingsConnector()
    bridge = RoconIOTBridge(connector)

    bridge.loginfo("Initilialised")
    bridge.spin()
    bridge.loginfo("Bye Bye")
