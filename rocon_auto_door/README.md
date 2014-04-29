# ROCON auto door

## Overview

This package contains a ROS node to open and close automatic doors. The node acts as the ROS interface to Arduino Yun's REST API. The Arduino Yun is connected to the remote control of the automatic door being controlled.

## Usage

For running and testing this node use the launcher `launch/auto_door.launch`. It also contains all available parameters and topics.

The most important parameter is Arduino's IP address.

In order to use the Arduino is different WIFI networks, take a look at http://arduino.cc/en/Guide/ArduinoYun for instructions on how to configure it.