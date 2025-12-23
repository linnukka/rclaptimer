#ifndef RCLAPTIMERMQTTDEFS
#define RCLAPTIMERMQTTDEFS

#define mqttenabled		"1"
#define mqttserver		"mosquitto"
#define mqttport		"1883"
#define mqtttimeout		"60"
#define mqttretain		"true"
#define mqttqos			"1"

#define mqttstatustopic     "fun/lrclaptimer/status"
#define mqttdebugtopic      "fun/lrclaptimer/debug"
#define mqttcommandtopic    "fun/lrclaptimer/cmd"

#define mqttclientname  "rclaptimer"

#define serialterminator    "\r"


#endif
