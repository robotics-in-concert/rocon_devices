/**
 *  smartthings.groovy
 *  Original author : Davin Jane's IOTDB. Please refer IOTDB.org and https://github.com/dpjanes/iotdb-smartthings
 *
 *  Allow to receive device events via REST API 
 * 
 */
 
definition(
    name: "Rocon SmartThings Bridge",
    namespace: "",
    author: "Jihoon Lee",
    description: "Bridge to/from JSON",
    category: "My Apps",
    iconUrl: "https://s3.amazonaws.com/smartapp-icons/Convenience/Cat-Convenience.png",
    iconX2Url: "https://s3.amazonaws.com/smartapp-icons/Convenience/Cat-Convenience%402x.png",
    oauth: true)
    
/* --- setup section --- */
/*
 *  The user make sure that if you change anything related to this in the code
 *  that you update the preferences in your installed apps.
 *
 *  Note that there's a SmartThings magic that's _required_ here,
 *  in that you cannot access a device unless you have it listed
 *  in the preferences. Access to those devices is given through
 *  the name used here (i.e. d_*)
 */
preferences {
    section("Allow IOTDB to Control & Access These Things...") {
        input "d_motion", "capability.motionSensor", title: "Motion", required: false, multiple: true
        input "d_temperature", "capability.temperatureMeasurement", title: "Temperature", multiple: true
        input "d_contact", "capability.contactSensor", title: "Contact", required: false, multiple: true
        input "d_battery", "capability.battery", title: "Battery", multiple: true
    }
}

/*
input "d_switch", "capability.switch", title: "Switch", multiple: true
input "d_alarm", "capability.alarm", title: "alarm", multiple: true
input "d_configuration", "capability.configuration", title: "configuration", multiple: true
input "d_illuminanceMeasurement", "capability.illuminanceMeasurement", title: "illuminanceMeasurement", multiple: true
input "d_acceleration", "capability.accelerationSensor", title: "Acceleration", required: false, multiple: true
input "d_presence", "capability.presenceSensor", title: "Presence", required: false, multiple: true
input "d_polling", "capability.polling", title: "polling", multiple: true
input "d_relativeHumidityMeasurement", "capability.relativeHumidityMeasurement", title: "relativeHumidityMeasurement", multiple: true
input "d_thermostatCoolingSetpoint", "capability.thermostatCoolingSetpoint", title: "thermostatCoolingSetpoint", multiple: true
input "d_thermostatFanMode", "capability.thermostatFanMode", title: "thermostatFanMode", multiple: true
input "d_thermostatHeatingSetpoint", "capability.thermostatHeatingSetpoint", title: "thermostatHeatingSetpoint", multiple: true
input "d_thermostatMode", "capability.thermostatMode", title: "thermostatMode", multiple: true
input "d_thermostatSetpoint", "capability.thermostatSetpoint", title: "thermostatSetpoint", multiple: true
input "d_threeAxis", "capability.threeAxis", title: "3 Axis", multiple: true
input "d_threeAxisMeasurement", "capability.threeAxisMeasurement", title: "threeAxisMeasurement", multiple: true
input "d_waterSensor", "capability.waterSensor", title: "waterSensor", multiple: true

lqi: 100 %
acceleration: inactive
threeAxis: -38,55,1021
battery: 88 %
temperature: 65 F
*/

/*
 *  The API
 *  - ideally the command/update bit would actually
 *    be a PUT call on the ID to make this restful
 */
mappings {
    path("/get_all_types") {
      action: [
        GET: "_all_types"
      ]
    }

    path("/configuration") {
      action: [
        GET: "_get_config",
        PUT: "_set_config"
      ]
    }
}

/*
 *  This function is called once when the app is installed
 */
def installed() {
    _event_subscribe()
    _set_paired_uri("")
    
    log.debug "Started"
}

/*
 *  This function is called every time the user changes
 *  their preferences
 */
def updated()
{
    log.debug "updated"
    unsubscribe()
    _event_subscribe()
    _set_paired_uri("")
}

/* --- event section --- */

/*
 *  What events are we interested in. This needs
 *  to be in it's own function because both
 *  updated() and installed() are interested in it.
 */
def _event_subscribe()
{
    subscribe(d_motion, "motion", "_on_event")
    subscribe(d_temperature, "temperature", "_on_event")
    subscribe(d_contact, "contact", "_on_event")
    subscribe(d_battery, "battery", "_on_event")
}

/*
 *  This function is called whenever something changes.
 */
def _on_event(evt)
{
    log.debug "_on_event XXX event.id=${evt?.id} event.deviceId=${evt?.deviceId} event.isStateChange=${evt?.isStateChange} event.name=${evt?.name}"
    
    def dt = _device_and_type_for_event(evt)
    if (!dt) {
        log.debug "_on_event deviceId=${evt.deviceId} not found?"
        return;
    }
    
    def jd = _device_to_json(dt.device, dt.type)
    log.debug "_on_event deviceId=${jd}"

    _inform_partner(dt.device, dt.type, jd)

}

/* --- API section --- */
def _all_types()
{
  _dtd()
}

def _get_config()
{
  def resp = []
  def uri = _get_paired_uri()

  resp << ['paired_uri': uri]
  return resp 
}


def _set_config()
{
  def uri = params?.uri

  log.debug "_update_paired_uri : received uri=${uri}"
  _set_paired_uri(uri)
}


/*
 */
def deviceHandler(evt) {}


def _inform_partner(device, device_type, deviced) {
  
  def uri = _get_paired_uri() 
  if (!uri) {
    log.debug "_inform_partner : uri has not been set yet"
    return
  }

  def now = Calendar.instance
  def date = now.time
  def millis = date.time
  def sequence = millis
  def isodatetime = deviced?.value?.timestamp
  
  def topic = "st/${device_type}/${deviced.id}".toString()
  
  def headers = [:]
  def body = [
      "topic": topic,
      "payloadd": deviced?.value,
      "timestamp": isodatetime,
      "sequence": sequence,
  ]

  def params = [
      uri: uri,
      headers: headers,
      body: body
  ]

  log.debug "_inform_partner : Sending"

  httpPostJson(params) { log.debug "_inform_partner :response=${response}"}

}



/* --- internals --- */
/*
 *  Devices and Types Dictionary
 */
def _dtd()
{
    [ 
        switch: d_switch, 
        motion: d_motion, 
        temperature: d_temperature, 
        contact: d_contact,
        acceleration: d_acceleration,
        presence: d_presence,
        battery: d_battery,
        threeAxis: d_threeAxis
    ]
}

def _devices_for_type(type) 
{
    _dtd()[type]
}

def _device_and_type_for_event(evt)
{
    def dtd = _dtd()
    
    for (dt in _dtd()) {
        if (dt.key != evt.name) {
        	continue
        }
        
        def devices = dt.value
        for (device in devices) {
            if (device.id == evt.deviceId) {
                return [ device: device, type: dt.key ]
            }
        }
    }
}

private _set_paired_uri(uri) {
  atomicState.paired_uri = uri
}

private _get_paired_uri()
{
  return atomicState.paired_uri
}

/**
 *  Do a device command
 */
private _device_command(device, type, jsond) {
    if (!device) {
        return;
    }
    if (!jsond) {
        return;
    }
    
    if (type == "switch") {
        def n = jsond['switch']
        if (n == -1) {
            def o = device.currentState('switch')?.value
            n = ( o != 'on' )
        }
        if (n) {
            device.on()
        } else {
            device.off()
        }
    } else {
        log.debug "_device_command: device type=${type} doesn't take commands"
    }
}

/*
 *  Convert a single device into a JSONable object
 */
private _device_to_json(device, type) {
    if (!device) {
        return;
    }

    def vd = [:]
    def jd = [id: device.id, label: device.label, type: type, value: vd];
    
    if (type == "switch") {
        def s = device.currentState('switch')
        vd['timestamp'] = s?.isoDate
        vd['switch'] = s?.value == "on"
    } else if (type == "motion") {
        def s = device.currentState('motion')
        vd['timestamp'] = s?.isoDate
        vd['motion'] = s?.value == "active"
    } else if (type == "temperature") {
        def s = device.currentState('temperature')
        vd['timestamp'] = s?.isoDate
        vd['temperature'] = s?.value.toFloat()
    } else if (type == "contact") {
        def s = device.currentState('contact')
        vd['timestamp'] = s?.isoDate
        vd['contact'] = s?.value == "closed"
    } else if (type == "acceleration") {
        def s = device.currentState('acceleration')
        vd['timestamp'] = s?.isoDate
        vd['acceleration'] = s?.value == "active"
    } else if (type == "presence") {
        def s = device.currentState('presence')
        vd['timestamp'] = s?.isoDate
        vd['presence'] = s?.value == "present"
    } else if (type == "battery") {
        def s = device.currentState('battery')
        vd['timestamp'] = s?.isoDate
        vd['battery'] = s?.value.toFloat() / 100.0;
    } else if (type == "threeAxis") {
        def s = device.currentState('threeAxis')
        vd['timestamp'] = s?.isoDate
        vd['x'] = s?.xyzValue?.x
        vd['y'] = s?.xyzValue?.y
        vd['z'] = s?.xyzValue?.z
    }
    
    return jd
}
