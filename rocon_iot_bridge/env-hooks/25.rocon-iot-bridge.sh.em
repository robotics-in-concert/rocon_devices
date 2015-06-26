
: ${SMARTTHINGS_BRIDGE_ADDRESS:=localhost}
: ${SMARTTHINGS_BRIDGE_PORT:=port}
: ${SMARTTHINGS_TARGET_CONFIG:=`rospack find rocon_iot_bridge`/resources/smartthings/default.yaml}

export SMARTTHINGS_BRIDGE_ADDRESS
export SMARTTHINGS_BRIDGE_PORT
export SMARTTHINGS_TARGET_CONFIG
