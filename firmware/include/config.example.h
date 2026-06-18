#pragma once

// Copy this file to config.h and fill in your values.
// config.h is gitignored.

#define WIFI_SSID     "your-ssid"
#define WIFI_PASSWORD "your-password"

// CC1101 wiring (ESP32 VSPI defaults)
#define PIN_SCK   18
#define PIN_MISO  19
#define PIN_MOSI  23
#define PIN_CS     5
#define PIN_GDO0   2

// MQTT broker
#define MQTT_HOST      "192.168.1.10"   // broker IP
#define MQTT_PORT      1883
#define MQTT_USER      ""               // "" = anonymous
#define MQTT_PASSWORD  ""
#define MQTT_CLIENT_ID "watts-bridge"
