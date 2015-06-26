#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_devices/license/LICENSE
#

import rospy
import requests
import threading
import rocon_device_msgs.srv as rocon_device_srvs
import rocon_device_msgs.msg as rocon_device_msgs

from flask import Flask
from flask import request

class RoconIOTBridge(object):
    def __init__(self, local_address, local_port, global_address, global_port, connector):
        self._app = Flask(rospy.get_name())
        self._local_address = local_address
        self._local_port = local_port
        self._global_address = global_address
        self._global_port = global_port
        self._connector = connector


    def _init_flask(self):
        self._app.add_url_rule('/index', view_func=self._index, methods=['GET'])
        self._app.add_url_rule('/devices', view_func=self._received_devices_event, methods=['POST'])
        self._app.add_url_rule(rule='/shutdown', view_func=self._shutdown_flask)

        self._server_thread = threading.Thread(target=self._app.run, kwargs={'host': self._local_address, 'port': self._local_port, 'debug': False}) 

    def _init_ros_api(self):
        self._srv_get_device_list = rospy.Service('get_device_list', rocon_device_srvs.GetDeviceList, self._process_get_device_list)
        self._pub_device_event = rospy.Publisher('devices', rocon_device_msgs.Devices, queue_size=2)

    def spin(self):
        self._init_flask()
        self._init_ros_api()

        subscription_uri = "%s:%s/devices"%(self._global_address, self._global_port)
        self.loginfo("initialising with %s"%subscription_uri)
        resp = self._connector.init({"address": self._global_address, "port": self._global_port, "api":"devices"})
        if resp:
            self.loginfo(str(resp))

        self.loginfo("Starting bridge server...")
        self._server_thread.start()
        self.loginfo("Started...")

        while not rospy.is_shutdown():
            rospy.sleep(1.0)

        resp = self._connector.close()
        if resp:
            self.loginfo(str(resp))

        result, e = self._shutdown_server()
        self.loginfo(e)
        self._server_thread.join(1)

    def _received_devices_event(self):
        j = request.json
        self.loginfo("Received : %s"%str(j))

        return "Hola"

    def _process_get_device_list(self, req):
        return rocon_device_srvs.GetDeviceListResponse()

    def _index(self):
        self.loginfo("index!")
        return "Index!"

    def _shutdown_flask(self, methods=['GET']):
        self.loginfo('shutdown requested')
        func = request.environ.get('werkzeug.server.shutdown')
        if func is None:
            raise RuntimeError('Not running with the Werkzeug Server')
        func()

    def _shutdown_server(self):
        try:
            address = 'http://' + self._local_address + ':' + str(self._local_port) + '/shutdown'
            resp = requests.get(address)
        except Exception, e:
            return (False, e)
        else:
            return (True, 'server shutdown success!!')


    def loginfo(self, msg):
        rospy.loginfo("%s : %s"%(rospy.get_name(), msg))
