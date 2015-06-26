#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_devices/license/LICENSE
#

import rospy
import flask
import rocon_device_msgs.srv as rocon_device_srvs
import rocon_device_msgs.msg as rocon_device_msgs

from flask import Flask
from flask import request

class RoconIOTBridge(object):
    def __init__(self, address, port, connector):
        self._app = Flask(rospy.get_name())
        self._address = address
        self._port = port
        self._connector = connector

    def _init_flask(self):
        self._app.add_url_rule('/index', view_func=self._index, methods=['GET'])
        self._app.add_url_rule('/devices', view_func=self._received_devices_event, methods=['POST'])

    def _init_ros_api(self):
        self._srv_get_device_list = rospy.Service('get_device_list', rocon_device_msgs.GetDeviceList, self._process_get_device_list)
        self._pub_device_event = rospy.Publisher('devices', rocon_device_msgs.Devices, queue_size=2)

    def spin(self):
        self._init_flask()
        self._init_ros_api()

        subscription_uri = "%s:%s/devices"
        self.loginfo(subscription_uri)
        resp = self._connector().init({uri:subscription_uri})
        if resp:
            self.loginfo(str(resp))
        rospy.on_shutdown(self._shutdown_flask)
        self._app.run(host=self._address, port=self._port, debug=False)

    def _shutdown_flask(self):
        self.loginfo('request shutdown')
        func = request.environ.get('werkzeug.server.shutdown')
        if func is None:
            raise RuntimeError('Not running with the Werkzeug Server')
        func()

    def loginfo(self, msg):
        rospy.loginfo("%s : %s"%(rospy.get_name(), msg))
