# alert mode
string none = "none"
string select = "select"
string lselect = "lselect"
#effect mode
string colorloop = "colorloop"

int32 light_id #light id
bool on #light on/off flag true:on /false:off

#color coordination in CIE color space http://developers.meethue.com/coreconcepts.html#color_gets_more_complicated
float32 x #x 
float32 y #y 

#color coordination in HSV color space http://en.wikipedia.org/wiki/HSL_and_HSV
uint16 hue
uint8 bri
uint8 sat

string alert
string effect


