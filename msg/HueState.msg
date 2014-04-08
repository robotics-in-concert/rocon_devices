#specific effect
string NONE = none
#effect mode

#the light will cycle through all hues using the current brightness and saturation settings.
string COLOR_LOOP =  colorloop 
# alert mode

# The light is performing one breathe cycle.
string SELECT = select  
# he light is performing breathe cycles for 30 seconds or mode is none
string LSELECT = lselect 

#color mode
string HS = hs #hsv color space
string XY = xy #cie color space
string CT = ct #color temperature space

#state
bool on #light on/off flag true:on /false:off

#color coordination in CIE color space 
#http://developers.meethue.com/coreconcepts.html#color_gets_more_complicated
float32[] xy #xy 

#color coordination in HSV color space 
#http://en.wikipedia.org/wiki/HSL_and_HSV
uint16 hue #h
uint8 sat #s
uint8 bri #v

#color temperature  
#http://en.wikipedia.org/wiki/Mired
#capable of 153 (6500K) to 500 (2000K)
uint16 ct 

#specific effect
string mode

#transition time
int32 transitiontime


#config
string color_mode
bool reachable
