rocon_devices
=============

A collection of device drivers involved in rocon


##roshue

For controlling PHILIPS hue by ros protocol.

First, connect the bridge in your local network. And run bottom commend

```
> rosrun roshue roshue_bridge.py
```
If hue bridge connection is alright, you can see the this message.

```
[INFO] [WallTime: 1397022783.781245] bridge connect
```
Also, you can check the topic list

```

/hue_list

/set_hue_color_ct
/set_hue_color_hsv
/set_hue_color_mode
/set_hue_color_on
/set_hue_color_xy

```
```/hue_list``` topic publish the valid hue bulb list. If you change the hue bulb color, you can use the one of ```/set_hue_color_*``` topics.
