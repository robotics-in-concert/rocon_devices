rocon_devices
=============

A collection of device drivers involved in rocon

##roshue

For controlling PHILIPS hue by ros topic.

First, connect the bridge in your local network. The user have to get the hue bridge ip. There are two options. 
* Visit [meethue](https://www.meethue.com/api/nupnp)
* Ask your network manager

After above process, run the hue bridge by following command
```
> roslaunch rocon_hue hue_bridge_server.launch --screen hue_ip:=<bridge ip>
```

You can see the this message, when hue bridge connection is success.

```
[INFO] [WallTime: 1397022783.781245] bridge connect
```

Also, you can check the topic list

```
/list_hue
/set_hue
```
```/hue_list``` topic publish the valid hue bulb list. If you change the hue bulb color, you can use the one of ```/set_hue``` topics with hsv color space. We also refer to easy method in order to change color. If you want easliy to change color, you only set the light_id and predefined color. 
Example)
```
> rostopic pub /set_hue rocon_device_msgs/Hue "light_id: 0
  type: ''
  name: ''
  modelid: ''
  swvesion: ''
  state: {'on': false, hue: 0, sat: 0, bri: 0, reachable: false, color: 'RED'}"
```

The predefiend color list is described at [HueState](https://github.com/robotics-in-concert/rocon_msgs/blob/indigo/rocon_device_msgs/msg/HueState.msg)

If you want to handle the multiple hue light, you should use the wire network. Beacause hue is controlled by rest api. If they is used in wireless, they are not chaged color even though you publish the ```set_hue``` message.
