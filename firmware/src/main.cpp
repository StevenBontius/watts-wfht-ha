/**
 * Watts WFHT-RF thermostat emulator - ESP32 + CC1101 transmitter.
 *
 * Single device, no MQTT. Driven over HTTP: you POST/GET a temperature and
 * setpoint to /tx-watts and the ESP transmits a Watts frame (A-B-A burst) on
 * the air. This is the proof-of-concept TX path.
 *
 * RF layer: software Manchester end to end. The CC1101 hardware Manchester,
 * sync insertion, CRC and whitening are all OFF, because the rtl-433 decoder
 * is OOK_PULSE_MANCHESTER_ZEROBIT and Manchester-decodes the WHOLE frame,
 * sync word included. The frame (preamble, sync, length, header, data, both
 * CRCs) is built and Manchester-encoded in firmware, then shipped as raw OOK.
 *
 * THE ONE KNOB: MANCHESTER_ONE_IS_10 sets the half-bit polarity. If rtl-433
 * shows nothing or reports a CRC failure with bit-inverted bytes, flip it to 0
 * and reflash. That is the only physical-layer ambiguity left.
 *
 * Endpoints:
 *   GET /status                                  radio + config info
 *   GET /tx-test                                 raw OOK smoke test
 *   GET /tx-watts?amb=22.5&sp=24.0&cfh=0&mode=heat   send a Watts A-B-A burst
 *   GET /tx-pair?duration=30&mode=heat           send pairing frames at 2 Hz
 *                                                (blocks for `duration` s)
 *
 * Copyright (C) 2026. GPLv2 or later.
 */

// ---------------------------------------------------------------------------
// PLANNED ARCHITECTURE (design notes, not yet implemented)
// ---------------------------------------------------------------------------
// The bridge is a transcoder, not a controller-of-record: it reads ambient +
// setpoint from an HA thermostat over MQTT, runs the P-loop locally to derive
// call-for-heat, and transmits a Watts frame. HA needs zero configuration; the
// ESP owns the device registry (NVS) and all coupling.
//
// Pairing has two independent halves, joined on the ESP:
//
//   1. Watts RF side (needs an RX path, TX-only today). To pair a thermostat
//      the user switches it OFF; this transmits setpoint 0.0 packets, which are
//      rare in normal operation. The ESP listens for an off-frame, validates it
//      (CRC + FF FF FE shape), captures the device ID (frame bytes 13..15) and
//      registers it. The user then names that channel (e.g. "livingroom").
//      The deliberate switch-off is what ties an ID to a room.
//
//   2. HA side. Subscribe to Zigbee2MQTT's retained `zigbee2mqtt/bridge/devices`
//      inventory, filter for thermostat-capable devices, present a dropdown.
//      The `exposes` metadata gives the source topic + field names
//      (local_temperature / occupied_heating_setpoint) automatically -- nothing
//      to type.
//
// Binding = pair a Watts channel (device ID) with a discovered Z2M thermostat,
// stored on the ESP. Per channel: name, device ID, source topic, field map.
//
// CAVEAT: HA-thermostat discovery is Zigbee2MQTT-specific -- it reads the Z2M
// bridge topic, not a generic HA mechanism. Correct for this stack (W100 -> Z2M).
// Supporting non-Zigbee HA thermostats would need a second source
// (`homeassistant/climate/#` discovery configs), and even then only
// MQTT-published climate entities would appear -- HA-internal ones (e.g.
// generic_thermostat) would not. Not built; speculative.
// ---------------------------------------------------------------------------

#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <ELECHOUSE_CC1101_SRC_DRV.h>
#include <AsyncMqttClient.h>
#include <math.h>
#include <string.h>
#include "config.h"

// ---------------------------------------------------------------------------
// Tunables
// ---------------------------------------------------------------------------

// Software Manchester half-bit polarity.
//   1 -> logical 1 sends chips "10", logical 0 sends chips "01" (G.E. Thomas)
//   0 -> the inverse
// Start at 1. If the decoder is silent or CRCs fail, set to 0 and reflash.
#define MANCHESTER_ONE_IS_10 1

// Device ID cloned from captures (frame bytes 13..15).
static const uint8_t DEV_ID[3] = {0x34, 0x9E, 0x67};

// Mode byte (frame byte 12). bit1 = heat(1)/cool(0), bit0 = pairing.
// 0x02 = heat, normal operation, not pairing.
static const uint8_t MODE_HEAT_NORMAL = 0x02;

// ---------------------------------------------------------------------------
// CC1101 register / strobe constants (guarded)
// ---------------------------------------------------------------------------
#ifndef CC1101_MDMCFG4
#define CC1101_MDMCFG4 0x10
#endif
#ifndef CC1101_MDMCFG3
#define CC1101_MDMCFG3 0x11
#endif
#ifndef CC1101_MDMCFG2
#define CC1101_MDMCFG2 0x12
#endif
#ifndef CC1101_MDMCFG1
#define CC1101_MDMCFG1 0x13
#endif
#ifndef CC1101_PKTCTRL0
#define CC1101_PKTCTRL0 0x08
#endif
#ifndef CC1101_PKTLEN
#define CC1101_PKTLEN 0x06
#endif
#ifndef CC1101_TXFIFO
#define CC1101_TXFIFO 0x3F
#endif
#ifndef CC1101_MARCSTATE
#define CC1101_MARCSTATE 0x35
#endif
#ifndef CC1101_SIDLE
#define CC1101_SIDLE 0x36
#endif
#ifndef CC1101_SFTX
#define CC1101_SFTX 0x3B
#endif
#ifndef CC1101_STX
#define CC1101_STX 0x35
#endif

static AsyncWebServer server(80);
static AsyncMqttClient mqtt;

// Reconnect bookkeeping. AsyncMqttClient is event-driven and never blocks; on a
// drop we just re-arm a one-shot retry that loop() fires.
static uint32_t mqttRetryAt = 0;   // millis() of next connect attempt, 0 = idle

// Zigbee2MQTT auto-discovery. The broker pushes this retained inventory the
// instant we subscribe; we parse it for thermostat-capable devices. The payload
// is large and AsyncMqttClient delivers it in fragments, so we reassemble it in
// a heap buffer keyed by `index` before parsing.
static const char *Z2M_DEVICES_TOPIC = "zigbee2mqtt/bridge/devices";
static char  *z2mBuf   = nullptr;   // reassembly buffer, malloc'd per message
static size_t z2mTotal = 0;         // expected full payload length

// Registry of discovered thermostats. Lets an incoming state message on
// zigbee2mqtt/<name> be mapped back to the field names found during discovery.
// Rebuilt every time the inventory is (re)published.
struct Thermostat {
    char name[40];        // friendly_name == state topic suffix
    char tempField[32];   // e.g. local_temperature
    char spField[32];     // e.g. occupied_heating_setpoint
    char modeField[32];   // e.g. system_mode (off/heat/cool/auto), "" if none
};
static const int MAX_THERMOSTATS = 8;
static Thermostat thermostats[MAX_THERMOSTATS];
static int        thermostatCount = 0;

// ---------------------------------------------------------------------------
// Pairing state. The pairing stream lasts up to 120 s, far too long to run
// inside an AsyncWebServer callback (that blocks the AsyncTCP task and trips
// the task watchdog, rebooting the ESP mid-burst). Instead the handler arms
// this state and returns immediately; loop() emits one frame per `gap_ms`.
// ---------------------------------------------------------------------------
struct PairJob {
    bool     active   = false;
    int      sent     = 0;       // frames sent so far
    int      total    = 0;       // frames to send
    uint32_t gap_ms   = 500;
    uint32_t next_at  = 0;       // millis() of next frame
    float    amb      = 0.0f;
    float    sp        = 0.0f;
    uint8_t  mode     = 0x03;    // heat + pairing
};
static PairJob pairJob;

// ---------------------------------------------------------------------------
// CRC algorithms, matched bit-for-bit to the rtl-433 decoder
// ---------------------------------------------------------------------------

// CRC-8, poly 0xE6, init 0x00. Caller applies the trailing ^0xBE ^byte20.
static uint8_t crc8_raw(const uint8_t *data, size_t len) {
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++)
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0xE6) : (uint8_t)(crc << 1);
    }
    return crc;
}

// CRC-16/CMS, poly 0x8005, init 0xFFFF, no reflection, xorout 0x0000.
static uint16_t crc16_cms(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= ((uint16_t)data[i]) << 8;
        for (int j = 0; j < 8; j++)
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x8005) : (uint16_t)(crc << 1);
    }
    return crc;
}

// ---------------------------------------------------------------------------
// Frame construction (24 logical bytes, decoded domain)
// ---------------------------------------------------------------------------
//
//   [0..3]   preamble  2A AA AA AA
//   [4..7]   sync      D3 91 D3 91
//   [8]      length    0D
//   [9..11]  header    FF FF FE
//   [12]     mode
//   [13..15] device id
//   [16..17] ambient   (BE, x10)
//   [18..19] setpoint  (BE, x10)
//   [20]     call-for-heat (0x00 / 0x64)
//   [21]     CRC-8     over [8..19], then ^0xBE ^byte20
//   [22..23] CRC-16    over [8..21], big-endian
//
static size_t buildFrame(uint8_t *f, float amb_C, float sp_C, uint8_t cfh, uint8_t mode) {
    int16_t amb = (int16_t)lroundf(amb_C * 10.0f);
    int16_t sp  = (int16_t)lroundf(sp_C  * 10.0f);

    uint8_t *p = f;
    *p++ = 0x2A; *p++ = 0xAA; *p++ = 0xAA; *p++ = 0xAA;        // preamble
    *p++ = 0xD3; *p++ = 0x91; *p++ = 0xD3; *p++ = 0x91;        // sync

    uint8_t *app = p;                                          // byte 8 onward
    *p++ = 0x0D;                                               // length
    *p++ = 0xFF; *p++ = 0xFF; *p++ = 0xFE;                     // header
    *p++ = mode;                                               // mode
    *p++ = DEV_ID[0]; *p++ = DEV_ID[1]; *p++ = DEV_ID[2];      // device id
    *p++ = (uint8_t)((amb >> 8) & 0xFF); *p++ = (uint8_t)(amb & 0xFF);
    *p++ = (uint8_t)((sp  >> 8) & 0xFF); *p++ = (uint8_t)(sp  & 0xFF);
    *p++ = cfh;                                                // call-for-heat

    *p++ = (uint8_t)(crc8_raw(app, 12) ^ 0xBE ^ cfh);         // CRC-8
    uint16_t crc16 = crc16_cms(app, 14);
    *p++ = (uint8_t)((crc16 >> 8) & 0xFF);                    // CRC-16 hi
    *p++ = (uint8_t)(crc16 & 0xFF);                           // CRC-16 lo

    return (size_t)(p - f);                                    // 24
}

// ---------------------------------------------------------------------------
// Software Manchester encoder: 1 logical bit -> 2 chips, MSB first.
// ---------------------------------------------------------------------------
static size_t manchesterEncode(const uint8_t *in, size_t nbytes, uint8_t *out) {
    memset(out, 0, nbytes * 2);
    size_t chip = 0;
    for (size_t i = 0; i < nbytes; i++) {
        for (int b = 7; b >= 0; b--) {
            uint8_t bit = (in[i] >> b) & 1;
            uint8_t first, second;
#if MANCHESTER_ONE_IS_10
            first = bit ? 1 : 0; second = bit ? 0 : 1;
#else
            first = bit ? 0 : 1; second = bit ? 1 : 0;
#endif
            if (first)  out[chip >> 3] |= (uint8_t)(0x80 >> (chip & 7));
            chip++;
            if (second) out[chip >> 3] |= (uint8_t)(0x80 >> (chip & 7));
            chip++;
        }
    }
    return nbytes * 2;
}

// ---------------------------------------------------------------------------
// Raw fixed-length OOK transmit: no hardware preamble/sync/CRC/whitening.
// ---------------------------------------------------------------------------
static void cc1101SendRaw(const uint8_t *data, uint8_t len) {
    ELECHOUSE_cc1101.SpiStrobe(CC1101_SIDLE);
    ELECHOUSE_cc1101.SpiStrobe(CC1101_SFTX);                    // flush TX FIFO
    ELECHOUSE_cc1101.SpiWriteReg(CC1101_PKTLEN, len);           // fixed length
    ELECHOUSE_cc1101.SpiWriteBurstReg(CC1101_TXFIFO, (byte *)data, len);
    ELECHOUSE_cc1101.SpiStrobe(CC1101_STX);                     // enter TX

    uint32_t t0 = millis();
    while (millis() - t0 < 300) {                               // wait for IDLE
        uint8_t ms = ELECHOUSE_cc1101.SpiReadStatus(CC1101_MARCSTATE) & 0x1F;
        if (ms == 0x01) break;
        delayMicroseconds(200);
    }
    ELECHOUSE_cc1101.SpiStrobe(CC1101_SIDLE);
    ELECHOUSE_cc1101.SpiStrobe(CC1101_SFTX);
}

// Build, encode and transmit a single frame.
static void sendFrameOnce(float amb, float sp, uint8_t cfh, uint8_t mode) {
    uint8_t frame[24];
    buildFrame(frame, amb, sp, cfh, mode);   // 2a aa aa aa | sync | payload..crc16

    // Mirror the real device exactly: ONE continuous Manchester stream, with NO
    // raw (non-Manchester) lead-in or lead-out. The earlier raw 0x55 lead-in was
    // the real problem: as plain alternating chips at the same rate it could
    // shift the Manchester pairing phase by one chip (the {195} wrong-phase
    // ghost rows you saw), so the Watts slicer never recovered a byte-aligned
    // sync. A looser decoder like oregon happened to slice the good phase, which
    // is why the bytes looked perfect there while Watts stayed silent.
    //
    // Prepend two 0xAA ramp/preamble bytes (Manchester-encoded, so they give the
    // receiver clean symbols to lock phase on and cover the PA ramp) and append
    // one 0xAA trailing byte for the closing edge of the final payload bit (the
    // equivalent of the real device's trailing "193rd bit"). Everything goes out
    // as Manchester; the decoder ignores anything before the sync and after the
    // fixed 128 payload bits.
    uint8_t logical[2 + 24 + 1];
    logical[0] = 0xAA;
    logical[1] = 0xAA;
    memcpy(logical + 2, frame, 24);
    logical[26] = 0xAA;

    uint8_t chips[sizeof(logical) * 2];                        // 54 bytes
    manchesterEncode(logical, sizeof(logical), chips);

    cc1101SendRaw(chips, sizeof(chips));
}

// Emit `count` frames `gap_ms` apart, alternating sp (A) / sp+0.1 (B):
// A, B, A, B, ...  The +0.1 B-frame is the LSB-offset twin; consumers treat the
// lower of a pair as the real setpoint. This is the single transmit path:
//   normal state change -> count=3, gap=400 (A-B-A)
//   pairing             -> count=duration*2, gap=500 (2 Hz A-B-A-B-... stream)
static void sendBurst(float amb, float sp, uint8_t cfh, uint8_t mode,
                      int count, uint32_t gap_ms) {
    for (int i = 0; i < count; i++) {
        sendFrameOnce(amb, (i & 1) ? sp + 0.1f : sp, cfh, mode);
        if (i < count - 1) delay(gap_ms);
    }
}

// ---------------------------------------------------------------------------
// Radio init
// ---------------------------------------------------------------------------
static bool initCC1101() {
    ELECHOUSE_cc1101.setSpiPin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS);
    ELECHOUSE_cc1101.Init();
    if (!ELECHOUSE_cc1101.getCC1101()) {
        Serial.println("CC1101: SPI connection failed");
        return false;
    }

    ELECHOUSE_cc1101.setMHZ(433.92);          // your own RX hears this fine;
                                              // real units sit ~+73 kHz high
    ELECHOUSE_cc1101.setModulation(2);        // OOK
    ELECHOUSE_cc1101.setGDO0(PIN_GDO0);

    // Chip rate. This is the on-air chip rate directly (Manchester is done in
    // software). The Watts decoder uses reset_limit = 900 us, so a Manchester
    // "long" symbol (2 chips) must stay UNDER 900 us or rtl-433 splits the frame
    // mid-packet and never decodes it. At the nominal 2170 chips/s a chip is
    // 461 us and the long symbol is ~922 us, just over the limit. We run a touch
    // faster: ~2380 chips/s -> chip ~420 us -> long symbol ~840 us, a 60 us
    // margin under reset_limit. (DRATE_E=6, DRATE_M=128.) This is also closer to
    // what the real device actually does.
    uint8_t m4 = ELECHOUSE_cc1101.SpiReadStatus(CC1101_MDMCFG4) & 0xF0;
    ELECHOUSE_cc1101.SpiWriteReg(CC1101_MDMCFG4, m4 | 0x06);
    ELECHOUSE_cc1101.SpiWriteReg(CC1101_MDMCFG3, 0x80);

    // MDMCFG2 = 0x30: OOK, Manchester OFF, SYNC_MODE=000 (no hw preamble/sync).
    ELECHOUSE_cc1101.SpiWriteReg(CC1101_MDMCFG2, 0x30);
    ELECHOUSE_cc1101.SpiWriteReg(CC1101_MDMCFG1, 0x02);

    // PKTCTRL0 = 0x00: fixed length, no CRC, no whitening, FIFO mode.
    ELECHOUSE_cc1101.SpiWriteReg(CC1101_PKTCTRL0, 0x00);

    ELECHOUSE_cc1101.SpiStrobe(CC1101_SIDLE);
    Serial.println("CC1101: ready (software Manchester, raw OOK)");
    return true;
}

// ---------------------------------------------------------------------------
// Zigbee2MQTT inventory parsing -> thermostat discovery
// ---------------------------------------------------------------------------
// The inventory is a JSON array of device objects. A thermostat-capable device
// has, in definition.exposes, an entry of type "climate" whose nested features
// include local_temperature (current temp) and occupied_heating_setpoint. We use
// an ArduinoJson filter so only the fields we need survive deserialization --
// the raw inventory is far too big to hold whole on an ESP32.
static void parseZ2MDevices(const char *json, size_t len) {
    JsonDocument filter;
    filter[0]["friendly_name"]          = true;
    filter[0]["ieee_address"]           = true;
    filter[0]["definition"]["exposes"]  = true;

    JsonDocument doc;
    DeserializationError err =
        deserializeJson(doc, json, len, DeserializationOption::Filter(filter));
    if (err) {
        Serial.printf("Z2M: inventory parse error: %s\n", err.c_str());
        return;
    }

    JsonArray devices = doc.as<JsonArray>();
    Serial.printf("Z2M: inventory received (%u devices)\n", devices.size());

    thermostatCount = 0;   // rebuilt from this inventory snapshot
    int found = 0;
    for (JsonObject dev : devices) {
        // definition is null for the coordinator; exposes iterates as empty then.
        for (JsonObject ex : dev["definition"]["exposes"].as<JsonArray>()) {
            const char *type = ex["type"] | "";
            if (strcmp(type, "climate") != 0) continue;

            const char *name = dev["friendly_name"] | "?";
            const char *ieee = dev["ieee_address"]  | "?";
            const char *tempProp = nullptr, *spProp = nullptr, *modeProp = nullptr;
            for (JsonObject feat : ex["features"].as<JsonArray>()) {
                const char *prop = feat["property"] | "";
                if (strcmp(prop, "local_temperature") == 0)      tempProp = prop;
                else if (strcmp(prop, "system_mode") == 0)       modeProp = prop;
                else if (strstr(prop, "setpoint") != nullptr)    spProp   = prop;
            }

            found++;
            Serial.printf("Z2M thermostat: \"%s\" [%s]\n", name, ieee);
            Serial.printf("  temp field:     %s\n", tempProp ? tempProp : "(none)");
            Serial.printf("  setpoint field: %s\n", spProp   ? spProp   : "(none)");
            Serial.printf("  mode field:     %s\n", modeProp ? modeProp : "(none)");

            // Register and subscribe to this thermostat's state topic so we get
            // live local_temperature / occupied_heating_setpoint updates.
            if (thermostatCount < MAX_THERMOSTATS) {
                Thermostat &t = thermostats[thermostatCount++];
                strlcpy(t.name, name, sizeof(t.name));
                strlcpy(t.tempField, tempProp ? tempProp : "local_temperature",
                        sizeof(t.tempField));
                strlcpy(t.spField, spProp ? spProp : "occupied_heating_setpoint",
                        sizeof(t.spField));
                strlcpy(t.modeField, modeProp ? modeProp : "", sizeof(t.modeField));
                char stateTopic[64];
                snprintf(stateTopic, sizeof(stateTopic), "zigbee2mqtt/%s", name);
                uint16_t pid = mqtt.subscribe(stateTopic, 0);
                Serial.printf("  subscribed:     %s (pid %u)\n", stateTopic, pid);
            } else {
                Serial.println("  registry full -- not subscribed");
            }
            break;   // one climate expose per device is enough
        }
    }
    if (found == 0)
        Serial.println("Z2M: no thermostat-capable devices in inventory");
}

// Parse a thermostat state message (zigbee2mqtt/<name>) and log the current
// temperature and setpoint. Z2M publishes the full device state on this topic,
// so both fields are normally present; a field absent from a partial update
// reads back as NaN and is shown as "--".
static void handleStateMessage(const char *topic, const char *payload, size_t len) {
    const char *suffix = topic + strlen("zigbee2mqtt/");   // name part
    Thermostat *t = nullptr;
    for (int i = 0; i < thermostatCount; i++) {
        if (strcmp(suffix, thermostats[i].name) == 0) { t = &thermostats[i]; break; }
    }
    if (!t) return;   // not one of our registered thermostats

    JsonDocument doc;
    if (deserializeJson(doc, payload, len)) return;

    float temp = doc[t->tempField] | NAN;
    float sp   = doc[t->spField]   | NAN;
    // system_mode is a string enum (off/heat/cool/auto), not a number.
    const char *mode = t->modeField[0] ? (doc[t->modeField] | "--") : "n/a";
    char tbuf[8], sbuf[8];
    if (isnan(temp)) strcpy(tbuf, "--"); else snprintf(tbuf, sizeof(tbuf), "%.1f", temp);
    if (isnan(sp))   strcpy(sbuf, "--"); else snprintf(sbuf, sizeof(sbuf), "%.1f", sp);
    Serial.printf("Z2M state %s: temp=%s setpoint=%s mode=%s\n",
                  t->name, tbuf, sbuf, mode);
}

// AsyncMqttClient message callback. Large retained payloads arrive in fragments
// (index..index+len of total); reassemble before parsing.
static void onMqttMessage(char *topic, char *payload,
                          AsyncMqttClientMessageProperties props,
                          size_t len, size_t index, size_t total) {
    // Thermostat state topics are small single-fragment JSON payloads.
    if (strcmp(topic, Z2M_DEVICES_TOPIC) != 0) {
        if (index == 0 && len == total)
            handleStateMessage(topic, payload, len);
        return;
    }

    // The inventory is large and fragmented -- reassemble before parsing.
    if (index == 0) {                 // first fragment: (re)alloc the buffer
        free(z2mBuf);
        z2mBuf   = (char *)malloc(total + 1);
        z2mTotal = total;
    }
    if (!z2mBuf || index + len > z2mTotal) return;   // OOM or stray fragment

    memcpy(z2mBuf + index, payload, len);
    if (index + len < total) return;                 // more fragments coming

    z2mBuf[total] = '\0';
    parseZ2MDevices(z2mBuf, total);
    free(z2mBuf);
    z2mBuf   = nullptr;
    z2mTotal = 0;
}

// ---------------------------------------------------------------------------
// MQTT init (connect + subscribe to the Z2M inventory for discovery)
// ---------------------------------------------------------------------------
// AsyncMqttClient is non-blocking: connect() returns immediately and the result
// arrives on a callback. We report over Serial in the same shape as the CC1101
// init ("MQTT: ..."). On a disconnect we arm a one-shot retry in loop().
static void initMqtt() {
    mqtt.setServer(MQTT_HOST, MQTT_PORT);
    mqtt.setClientId(MQTT_CLIENT_ID);
    if (strlen(MQTT_USER) > 0)
        mqtt.setCredentials(MQTT_USER, MQTT_PASSWORD);

    mqtt.onConnect([](bool sessionPresent) {
        Serial.printf("MQTT: connected to %s:%d\n", MQTT_HOST, MQTT_PORT);
        mqttRetryAt = 0;
        // Subscribe to the retained Z2M inventory; the broker delivers it at once.
        uint16_t pid = mqtt.subscribe(Z2M_DEVICES_TOPIC, 0);
        Serial.printf("MQTT: subscribed to %s (pid %u)\n", Z2M_DEVICES_TOPIC, pid);
    });
    mqtt.onDisconnect([](AsyncMqttClientDisconnectReason reason) {
        Serial.printf("MQTT: disconnected (reason %d), retry in 5s\n", (int)reason);
        mqttRetryAt = millis() + 5000;
    });
    mqtt.onMessage(onMqttMessage);

    Serial.printf("MQTT: connecting to %s:%d\n", MQTT_HOST, MQTT_PORT);
    mqtt.connect();
}

// ---------------------------------------------------------------------------
// HTTP routes
// ---------------------------------------------------------------------------
static void setupRoutes() {
    server.on("/status", HTTP_GET, [](AsyncWebServerRequest *req) {
        JsonDocument doc;
        doc["ip"]   = WiFi.localIP().toString();
        doc["rssi"] = WiFi.RSSI();
        doc["manchester_one_is_10"] = (int)MANCHESTER_ONE_IS_10;
        String body;
        serializeJson(doc, body);
        req->send(200, "application/json", body);
    });

    // Raw lead-in burst, a quick "is the PA keying" smoke test.
    server.on("/tx-test", HTTP_GET, [](AsyncWebServerRequest *req) {
        uint8_t buf[16];
        memset(buf, 0x55, sizeof(buf));
        cc1101SendRaw(buf, sizeof(buf));
        Serial.println("TX test sent");
        req->send(200, "text/plain", "tx sent");
    });

    // Send a Watts A-B-A burst.
    // Params: amb, sp (C), cfh (0 or 100), mode (heat/cool)
    server.on("/tx-watts", HTTP_GET, [](AsyncWebServerRequest *req) {
        float amb = 22.5f, sp = 24.0f;
        uint8_t cfh = 0x00;
        uint8_t mode = MODE_HEAT_NORMAL;

        if (req->hasParam("amb")) amb = req->getParam("amb")->value().toFloat();
        if (req->hasParam("sp"))  sp  = req->getParam("sp")->value().toFloat();
        if (req->hasParam("cfh")) {
            int v = req->getParam("cfh")->value().toInt();
            cfh = (v >= 100) ? 0x64 : 0x00;   // wire only carries 0 or 100
        }
        if (req->hasParam("mode")) {
            String m = req->getParam("mode")->value();
            mode = m.equalsIgnoreCase("cool") ? 0x00 : 0x02;
        }

        sendBurst(amb, sp, cfh, mode, 3, 400);   // A-B-A

        char resp[96];
        snprintf(resp, sizeof(resp),
                 "sent ABA: amb=%.1f sp=%.1f cfh=0x%02x mode=0x%02x",
                 amb, sp, cfh, mode);
        Serial.println(resp);
        req->send(200, "text/plain", resp);
    });

    // Send pairing frames at ~2 Hz for `duration` seconds (default 10, max 120).
    // Byte 12 has bit 0 set (pairing) plus the heat/cool bit from `mode`.
    // Blocks until done; the response is sent only after all frames are sent.
    server.on("/tx-pair", HTTP_GET, [](AsyncWebServerRequest *req) {
        int     duration  = 10;
        float   amb = 0.0f, sp = 0.0f;
        uint8_t mode_base = 0x02;   // heat by default

        if (req->hasParam("duration")) {
            int v = req->getParam("duration")->value().toInt();
            if (v >= 1 && v <= 120) duration = v;
        }
        if (req->hasParam("mode")) {
            String m = req->getParam("mode")->value();
            mode_base = m.equalsIgnoreCase("cool") ? 0x00 : 0x02;
        }
        if (req->hasParam("amb")) amb = req->getParam("amb")->value().toFloat();
        if (req->hasParam("sp"))  sp  = req->getParam("sp")->value().toFloat();

        uint8_t pair_mode = mode_base | 0x01;   // set pairing bit
        int frames = duration * 2;              // 2 Hz for `duration` seconds

        // Arm the job; loop() does the actual ~30 s of transmitting so we never
        // block the AsyncTCP task. Respond right away.
        pairJob.active  = true;
        pairJob.sent    = 0;
        pairJob.total   = frames;
        pairJob.gap_ms  = 500;
        pairJob.next_at = millis();
        pairJob.amb     = amb;
        pairJob.sp      = sp;
        pairJob.mode    = pair_mode;

        char resp[96];
        snprintf(resp, sizeof(resp),
                 "pairing started: %d frames over %ds, mode=0x%02x",
                 frames, duration, pair_mode);
        Serial.println(resp);
        req->send(200, "text/plain", resp);
    });

    server.onNotFound([](AsyncWebServerRequest *req) {
        req->send(404, "text/plain", "not found");
    });
}

void setup() {
    Serial.begin(115200);

    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.print("WiFi connecting");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.printf("\nIP: %s\n", WiFi.localIP().toString().c_str());

    initCC1101();
    initMqtt();
    setupRoutes();
    server.begin();
    Serial.println("HTTP server started");
}

void loop() {
    // One-shot MQTT reconnect: re-arm a connect attempt after a drop.
    if (mqttRetryAt != 0 && (int32_t)(millis() - mqttRetryAt) >= 0) {
        mqttRetryAt = 0;
        if (WiFi.status() == WL_CONNECTED) {
            Serial.println("MQTT: reconnecting");
            mqtt.connect();
        } else {
            mqttRetryAt = millis() + 5000;   // wait for WiFi to come back
        }
    }

    // Non-blocking pairing stream: one A/B frame every gap_ms, alternating
    // sp (A) / sp+0.1 (B), until `total` frames have gone out.
    if (pairJob.active && (int32_t)(millis() - pairJob.next_at) >= 0) {
        bool isB = pairJob.sent & 1;
        sendFrameOnce(pairJob.amb, isB ? pairJob.sp + 0.1f : pairJob.sp,
                      0x00, pairJob.mode);
        pairJob.sent++;
        if (pairJob.sent >= pairJob.total) {
            pairJob.active = false;
            Serial.printf("pairing done: %d frames sent\n", pairJob.sent);
        } else {
            pairJob.next_at = millis() + pairJob.gap_ms;
        }
    }
}