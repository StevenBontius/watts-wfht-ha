#pragma once

// Copy this file to config.h and fill in your values.
// config.h is gitignored.
//
// NOTE: WiFi + MQTT values below are only SEED defaults, used while NVS holds no
// saved config. They are loaded into RAM each boot but NOT persisted -- so while
// NVS is empty (a fresh device or after a flash erase) you can edit these macros
// and reflash to change the running config. The moment the captive portal saves
// (POST /save), NVS becomes authoritative and these macros are ignored on boot;
// merely connecting with the seeds does NOT persist them. /reset-wifi then opens
// the portal (it does not fall back to these macros) -- only a full NVS/flash
// erase restores seeding from here. Leave WIFI_SSID blank to force the portal.

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

// Web UI login. A non-empty password requires a login (HTTP Digest auth -- the
// password is never sent in cleartext) on every served route; the captive setup
// portal is exempt. Unlike WiFi/MQTT, the password is authoritative here: a
// non-empty HTTP_PASS overrides NVS on every boot, so editing it + reflashing
// always takes effect. Leave it "" to manage the password from the captive
// portal's "Web UI login" field instead, or to serve the UI openly.
#define HTTP_USER "admin"
#define HTTP_PASS ""

// Debug/test HTTP endpoints. When 1, exposes the unauthenticated /status,
// /rx-on, /rx-off, /tx-test and /tx-watts endpoints -- handy for radio
// bring-up, but any host on the LAN can then key the PA / actuate heating.
// Leave at 0 for production. The captive portal and binding/pairing UI are
// always served regardless of this flag.
#define DEBUG_HTTP 0
