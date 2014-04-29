# ROCON auto door

## Overview

This package contains a ROS node to open and close automatic doors. The node acts as the ROS interface to Arduino Yun's REST API. The Arduino Yun is connected to the remote control of the automatic door being controlled.

## Configuration

### ROS node

The roslauncher `launch/auto_door.launch` contains and explains all available parameters and topics.

### Arduino

The Arduino Yun needs to be conneted to an available WIFI network. To do so first reset the current WIFI configuration by pressing the WIFI reset button for 5 secs. The blue WLAN LED will start flashing. After a few minutes a "Ardunio-XXXX..." WIFI network will be available (XXXX will be the Yun's MAC address). Connect to that WIFI, open a browse (preferable Chrome) and open 'http://arduino.local'. Use the web interface to enter the details for the WIFI the Yun should connect to.

More details about how to configure and use the Arduino Yun can be found here: http://arduino.cc/en/Guide/ArduinoYun