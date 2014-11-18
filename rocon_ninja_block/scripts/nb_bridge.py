#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_devices/license/LICENSE
#
#################################################################################

import rospy
from std_msgs.msg import String
#from rocon_ninja_block import NinjaBlockBridge

from flask import Flask
from flask import request

app = Flask(__name__)
publisher = {}
@app.route('/home')
def home():
    return "Hello, World!"

@app.route('/test',methods=['POST','GET','PUT'])
def test():
    if request.method == 'POST':
        print 'POST type'
    elif request.method == 'GET':
        print 'GET type'
    elif request.method == 'PUT':
        print 'PUT type'
    print request.json
    #publisher["ninja_block_data"].publish(String(str(request.json)))
    return 'hello world'
if __name__ == '__main__':
    # rospy.init_node("nb_bridge",anonymous = True)
    # publisher["ninja_block_data"] = rospy.Publisher("ninja_block_data", String)
    app.run(debug=True,host='192.168.10.24',port=5555)
    #r_nb = NinjaBlockBridge()
    #r_nb.connect()