#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_devices/license/LICENSE
#
#################################################################################

from flask import Flask



app = Flask(__name__)
class NinjaBlockBridge():
    def __init__(self):
        self._app = Flask(__name__)
        pass

    def connect(self):
        self._app.run()
        pass
"""
    @_app.route('/')
    def home(self):
        return 'hello world!!!'
"""