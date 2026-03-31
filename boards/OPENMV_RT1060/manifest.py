# OpenMV library
add_library("openmv-lib", "$(OMV_LIB_DIR)")

# Drivers
require("onewire")
require("ds18x20")
require("dht")
require("neopixel")
freeze ("$(OMV_LIB_DIR)/", "modbus.py")
freeze ("$(OMV_LIB_DIR)/", "pid.py")
freeze ("$(OMV_LIB_DIR)/", "bno055.py")
freeze ("$(OMV_LIB_DIR)/", "ssd1306.py")
freeze ("$(OMV_LIB_DIR)/", "ssd1351.py")
freeze ("$(OMV_LIB_DIR)/", "pca9674a.py")
freeze ("$(OMV_LIB_DIR)/", "tb6612.py")
freeze ("$(OMV_LIB_DIR)/", "vl53l1x.py")
freeze ("$(OMV_LIB_DIR)/", "machine.py")
freeze ("$(OMV_LIB_DIR)/", "display.py")

# Bluetooth
require("aioble")

# Networking
require("ssl")
require("ntptime")
require("webrepl")
freeze ("$(OMV_LIB_DIR)/", "rpc.py")
# freeze ("$(OMV_LIB_DIR)/", "rtsp.py")  # Use device/rtsp.py instead
freeze ("$(OMV_LIB_DIR)/", "mqtt.py")
freeze ("$(OMV_LIB_DIR)/", "requests.py")

# Utils
require("time")
require("senml")
require("logging")
freeze ("$(OMV_LIB_DIR)/", "mutex.py")

# Libraries
require("ml", library="openmv-lib")
include("$(MPY_DIR)/extmod/asyncio")

# GroupGets device stack.
GROUPGETS_DIR = "../../../groupgets"
freeze(GROUPGETS_DIR + "/frameworks/microdot/src", "microdot")
freeze(GROUPGETS_DIR + "/device", "boson.py")
freeze(GROUPGETS_DIR + "/device", "boson_info.py")
freeze(GROUPGETS_DIR + "/device", "schema.py")
freeze(GROUPGETS_DIR + "/device", "openmv_server.py")
freeze(GROUPGETS_DIR + "/device", "main.py")
freeze(GROUPGETS_DIR + "/device", "rtsp.py")
freeze(GROUPGETS_DIR + "/device", "state.py")
freeze(GROUPGETS_DIR + "/device", "persistence.py")
freeze(GROUPGETS_DIR + "/device", "templates.py")
freeze(GROUPGETS_DIR + "/device", "routes_auth.py")
freeze(GROUPGETS_DIR + "/device", "routes_stream.py")
freeze(GROUPGETS_DIR + "/device", "routes_settings.py")
freeze(GROUPGETS_DIR + "/device", "routes_wifi.py")
freeze(GROUPGETS_DIR + "/device", "wifi_crypto.py")
freeze(GROUPGETS_DIR, "static")

# Boot script
freeze ("$(OMV_LIB_DIR)/", "_boot.py")
