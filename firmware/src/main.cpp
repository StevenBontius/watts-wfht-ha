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
 *   GET /                                        binding config web UI (browser)
 *                                                (setup portal form when unprovisioned)
 *   GET /reset-wifi                              clear net config, reboot into portal
 *   POST /save                                   portal: save WiFi+MQTT, reboot (AP mode)
 *   GET /status                                  radio + config info
 *   GET /thermostats                             discovered Z2M thermostats (JSON)
 *   GET /rx-on  /rx-off                          listen for + decode Watts frames
 *   GET /pair-listen                             arm off-frame device-ID capture
 *   GET /pair-status                             poll capture result (JSON)
 *   GET /pair-cancel                             disarm capture
 *   GET /bind?name=<z2m>&id=349E48               bind a Z2M thermostat -> Watts ID
 *   GET /unbind?name=<z2m>                       drop a binding
 *   GET /bindings                                list bindings + live state (JSON)
 *   GET /tx-test                                 raw OOK smoke test
 *   GET /tx-watts?amb=22.5&sp=24.0&cfh=0&mode=heat   send a Watts A-B-A burst
 *   GET /tx-pair?duration=30&mode=heat&id=349E48 send pairing frames at 2 Hz
 *                                                (id defaults to DEV_ID; the web
 *                                                 UI passes a bound zone's ID)
 *   GET /tx-pair-status                          poll pairing progress (JSON)
 *
 * Copyright (C) 2026. GPLv2 or later.
 */

// ---------------------------------------------------------------------------
// ARCHITECTURE
// ---------------------------------------------------------------------------
// The bridge is mostly a transcoder, not a controller-of-record: it reads
// ambient + setpoint from an HA/Z2M thermostat over MQTT and relays them to the
// Watts receiver as spoofed frames. The scheduler retransmits each bound zone on
// the 154 s heartbeat, or immediately when its source state changes. HA needs
// zero configuration; the ESP owns the bindings (NVS) and all coupling.
//
// It also runs a per-zone call-for-heat P-loop (cfhUpdate, ported from the Python
// emulator) that computes byte 20 live. The WFHC-MASTER does its own temperature
// regulation and IGNORES this flag (confirmed on hardware -- it actuates on the
// transmitted ambient vs. setpoint alone), so the loop has no actuation effect on
// that receiver; it is wired live only for a dumb receiver that acts on byte 20.
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
#include <DNSServer.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <ELECHOUSE_CC1101_SRC_DRV.h>
#include <AsyncMqttClient.h>
#include <Preferences.h>
#include <esp_system.h>
#include <math.h>
#include <string.h>
#include "config.h"
#include "cfh.h"   // call-for-heat P-loop (host-testable; see firmware/test/cfh_test.cpp)

// ---------------------------------------------------------------------------
// Tunables
// ---------------------------------------------------------------------------

// Software Manchester half-bit polarity.
//   1 -> logical 1 sends chips "10", logical 0 sends chips "01" (G.E. Thomas)
//   0 -> the inverse
// Start at 1. If the decoder is silent or CRCs fail, set to 0 and reflash.
#define MANCHESTER_ONE_IS_10 1

// RX diagnostics. When 1, loop() dumps every burst that synced but failed CRC
// (and "no sync" bursts) -- handy for protocol debugging, noisy in normal use.
// When 0, only CRC-valid frames and PAIR events are printed. Set to 1 + reflash
// when you need to inspect frames the decoder rejects.
#define RX_DEBUG 0

// Unauthenticated debug/test HTTP endpoints (/status, /rx-on, /rx-off,
// /tx-test, /tx-watts). These can key the PA and actuate heating from any
// host on the LAN, so they default OFF for production. Define DEBUG_HTTP to 1
// in config.h to expose them while bringing up the radio. The captive portal
// and the binding/pairing UI are always served regardless of this flag.
#ifndef DEBUG_HTTP
#define DEBUG_HTTP 0
#endif

// HTTP auth for the operational web UI. A non-empty password requires a login
// (HTTP Digest, so the password is never sent in cleartext) on every served
// route; the captive setup portal is exempt. Precedence (see loadNetConfig): a
// non-empty HTTP_PASS here is authoritative and overrides NVS every boot; leave
// it "" to manage the password from the captive portal instead (or serve openly).
#ifndef HTTP_USER
#define HTTP_USER "admin"
#endif
#ifndef HTTP_PASS
#define HTTP_PASS ""
#endif

// Reported to HA on the diagnostics blob so you know what's actually flashed
// without a serial console. Bump on each flash you care to distinguish.
#define FW_VERSION "m3-dev"

// Device ID cloned from captures (frame bytes 13..15).
static const uint8_t DEV_ID[3] = {0x34, 0x9E, 0x48};

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
#ifndef CC1101_SRX
#define CC1101_SRX 0x34
#endif
#ifndef CC1101_IOCFG0
#define CC1101_IOCFG0 0x02
#endif

static AsyncWebServer server(80);
static AsyncAuthenticationMiddleware httpAuth;   // gates the operational routes
static AsyncMqttClient mqtt;

// ---------------------------------------------------------------------------
// RX path. Step 1 (capture): CC1101 in async-serial OOK mode pushes the
// demodulated envelope out on GDO0; a pin interrupt times the edges into a ring
// of pulse widths. Step 2 (decode, this layer): loop() groups the pulses into
// per-burst chip streams, software-Manchester-decodes them (the exact mirror of
// the TX encoder), finds the D3 91 D3 91 sync, pulls the 16-byte frame and
// validates both CRCs -- then prints the recovered id / mode / ambient /
// setpoint / call-for-heat. Toggled over HTTP (/rx-on, /rx-off).
//
// Pulse -> chips: each pulse is one constant-level segment of the OOK envelope.
// A ~420 us segment is one chip, a ~840 us segment is two chips of that level
// (threshold 630 us). Level isn't recorded in the ISR; it's reconstructed by
// parity -- the idle gap that delimits a burst is carrier-low, so the first
// in-burst segment is always carrier-high, and segments alternate from there.
// ---------------------------------------------------------------------------
static const uint16_t PULSE_BUF = 512;
static volatile uint16_t pulseBuf[PULSE_BUF];
static volatile uint16_t pulseHead = 0, pulseTail = 0;
static volatile uint32_t lastEdgeUs = 0;
static bool rxActive = false;

// Per-burst chip accumulator (one byte per chip, 0/1), drained/decoded in loop().
static uint8_t rxChips[PULSE_BUF * 2];
static size_t  rxChipN = 0;

// Off-frame pairing capture. A thermostat switched off broadcasts setpoint-0.0
// frames (rare in normal use), so arming this and switching one off ties a
// physical thermostat to its 3-byte device ID. One-shot: the first CRC-valid
// sp==0 frame latches the ID and disarms, so a unit left off doesn't re-trigger.
static bool    pairArmed     = false;
static bool    pairCaptured  = false;
static bool    pairStartedRx = false; // pairing turned RX on -> turn it back off
static uint8_t pairId[3]     = {0, 0, 0};
static float   pairAmb       = NAN;   // ambient on the captured frame, for context

static void IRAM_ATTR gdo0Isr() {
    uint32_t now = micros();
    uint32_t d   = now - lastEdgeUs;   // unsigned, wraps cleanly
    lastEdgeUs   = now;
    uint16_t next = (uint16_t)((pulseHead + 1) % PULSE_BUF);
    if (next != pulseTail) {           // drop on overflow rather than block
        pulseBuf[pulseHead] = (d > 65535) ? 65535 : (uint16_t)d;
        pulseHead = next;
    }
}

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
    // Latest cached values (NAN / "" until the first state message). Z2M
    // republishes the full state on every attribute report (incl. ones we
    // ignore, like humidity), so we cache and only act/log on real changes.
    float    lastTemp;
    float    lastSp;
    char     lastMode[32];
    uint32_t lastUpdateMs;   // millis() of last state message (0 = never), liveness
};
static const int MAX_THERMOSTATS = 8;
static Thermostat thermostats[MAX_THERMOSTATS];
static int        thermostatCount = 0;

// Zone bindings: a discovered Z2M thermostat (by friendly_name) paired with the
// Watts device ID the bridge spoofs for it. Kept separate from thermostats[] on
// purpose -- that array is wiped and rebuilt on every Z2M inventory republish,
// whereas a binding must persist across those rebuilds. The {used, name, devId}
// identity is persisted in NVS (survives reboots); lastTxAt/dirty are runtime
// cadence state, reset on boot. Keyed by name, stable enough for the /bind UX.
struct ZoneBinding {
    bool     used;
    char     name[40];        // z2m friendly_name this binds to
    uint8_t  devId[3];        // Watts device ID to transmit for this zone
    uint32_t lastTxAt;        // millis() of last burst (0 = never sent)  [runtime]
    bool     dirty;           // source state changed -> transmit asap    [runtime]
    bool     stale;           // source went quiet -> TX dropped          [runtime]
    CfhState ctrl;            // call-for-heat P-loop state (cfh.h)        [runtime]
};
static ZoneBinding bindings[MAX_THERMOSTATS];

static ZoneBinding *findBinding(const char *name) {
    for (int i = 0; i < MAX_THERMOSTATS; i++)
        if (bindings[i].used && strcmp(bindings[i].name, name) == 0)
            return &bindings[i];
    return nullptr;
}

// The call-for-heat P-loop (constants, CfhState, cfhDuty/cfhOnDuration/cfhUpdate)
// lives in cfh.h -- a hardware-free unit so it can be host-tested with simulated
// time. serviceControlLoops below drives one CfhState per bound zone.

// Per-zone HA state publishing lives with the MQTT layer further down, but the
// scheduler and the Z2M message handler (both above it) trigger it on every TX
// and on a source change/recovery -- so it's forward-declared here. Returns the
// publish packet-id (0 = the send buffer was full and nothing was queued).
static uint16_t publishZoneState(const ZoneBinding *b);

// The discovered thermostat backing a binding (defined with the MQTT layer);
// the control loop above the MQTT layer reads its cached state, so forward it.
static Thermostat *zoneThermostat(const ZoneBinding *b);

// Request a paced (re)advertise of every retained config (see the sequencer near
// the MQTT layer). Triggered on connect and on /bind; loop() drains it.
static void startAdvertise();

// NVS persistence for bindings. The whole array is stored as one blob, guarded by
// a version key so a future ZoneBinding layout change ignores stale data rather
// than loading garbage. Writes happen only on /bind and /unbind (infrequent).
static Preferences   prefs;
static const char   *NVS_NS  = "watts";
static const char   *NVS_KEY = "bindings";
static const uint32_t NVS_VER = 2;   // bumped: ZoneBinding gained P-loop runtime fields

static void saveBindings() {
    prefs.begin(NVS_NS, false);
    prefs.putUInt("ver", NVS_VER);
    prefs.putBytes(NVS_KEY, bindings, sizeof(bindings));
    prefs.end();
}

static void loadBindings() {
    prefs.begin(NVS_NS, true);   // read-only
    if (prefs.getUInt("ver", 0) == NVS_VER &&
        prefs.getBytesLength(NVS_KEY) == sizeof(bindings)) {
        prefs.getBytes(NVS_KEY, bindings, sizeof(bindings));
    }
    prefs.end();
    int n = 0;
    for (int i = 0; i < MAX_THERMOSTATS; i++) {
        bindings[i].lastTxAt  = 0;          // runtime fields are meaningless after a
        bindings[i].dirty     = false;      // reboot -- start each zone fresh
        bindings[i].stale     = false;
        bindings[i].ctrl      = CfhState{}; // P-loop re-anchors on first data (idle, un-init)
        if (bindings[i].used) {
            n++;
            Serial.printf("NVS binding: \"%s\" -> %02X%02X%02X\n", bindings[i].name,
                          bindings[i].devId[0], bindings[i].devId[1], bindings[i].devId[2]);
        }
    }
    Serial.printf("NVS: loaded %d binding(s)\n", n);
}

// ---------------------------------------------------------------------------
// Network config (WiFi + MQTT) -- runtime-provisioned via the captive portal
// ---------------------------------------------------------------------------
// Stored in NVS (same namespace as bindings, separate key + version) so the
// device is field-provisionable without a recompile. config.h still defines the
// macros below, but they now act only as first-boot seed defaults: once the
// portal writes a config, NVS is authoritative and config.h is ignored. An empty
// wifiSsid is the signal to open the portal on boot.
struct NetConfig {
    char     wifiSsid[33];
    char     wifiPass[65];
    char     mqttHost[64];
    uint16_t mqttPort;
    char     mqttUser[33];
    char     mqttPass[65];
    char     mqttClientId[33];
    char     httpUser[33];   // web-UI login; empty httpPass disables auth
    char     httpPass[65];
};
static NetConfig      netCfg;
static const uint32_t NET_VER = 2;   // bumped for httpUser/httpPass -- old blobs reseed

static void saveNetConfig() {
    prefs.begin(NVS_NS, false);
    prefs.putUInt("netver", NET_VER);
    prefs.putBytes("netcfg", &netCfg, sizeof(netCfg));
    prefs.end();
}

static void loadNetConfig() {
    bool ok = false;
    prefs.begin(NVS_NS, true);   // read-only
    if (prefs.getUInt("netver", 0) == NET_VER &&
        prefs.getBytesLength("netcfg") == sizeof(netCfg)) {
        prefs.getBytes("netcfg", &netCfg, sizeof(netCfg));
        ok = true;
    }
    prefs.end();
    if (!ok) {
        // First boot (or layout change): seed from compile-time config.h.
        memset(&netCfg, 0, sizeof(netCfg));
        strlcpy(netCfg.wifiSsid,     WIFI_SSID,      sizeof(netCfg.wifiSsid));
        strlcpy(netCfg.wifiPass,     WIFI_PASSWORD,  sizeof(netCfg.wifiPass));
        strlcpy(netCfg.mqttHost,     MQTT_HOST,      sizeof(netCfg.mqttHost));
        netCfg.mqttPort = MQTT_PORT;
        strlcpy(netCfg.mqttUser,     MQTT_USER,      sizeof(netCfg.mqttUser));
        strlcpy(netCfg.mqttPass,     MQTT_PASSWORD,  sizeof(netCfg.mqttPass));
        strlcpy(netCfg.mqttClientId, MQTT_CLIENT_ID, sizeof(netCfg.mqttClientId));
        strlcpy(netCfg.httpUser,     HTTP_USER,      sizeof(netCfg.httpUser));
        strlcpy(netCfg.httpPass,     HTTP_PASS,      sizeof(netCfg.httpPass));
        Serial.println("NVS: no net config -- seeded from config.h defaults");
    } else {
        Serial.printf("NVS: net config loaded (ssid=\"%s\" mqtt=%s:%u)\n",
                      netCfg.wifiSsid, netCfg.mqttHost, netCfg.mqttPort);
    }

    // The web login is the one field config.h stays authoritative for: a non-empty
    // HTTP_PASS overrides whatever NVS holds (applied in RAM each boot, not saved),
    // so editing config.h + reflashing always takes effect without wiping NVS. Leave
    // HTTP_PASS "" to instead manage the password from the captive portal.
    if (strlen(HTTP_PASS) > 0) {
        strlcpy(netCfg.httpUser, HTTP_USER, sizeof(netCfg.httpUser));
        strlcpy(netCfg.httpPass, HTTP_PASS, sizeof(netCfg.httpPass));
        Serial.println("NET: web login overridden from config.h HTTP_PASS");
    }
}

// /reset-wifi: persist an empty config (valid version, blank SSID) so the next
// boot loads it cleanly and opens the portal -- rather than removing the key,
// which would just reseed from config.h and silently reconnect to the old net.
static void clearNetConfig() {
    memset(&netCfg, 0, sizeof(netCfg));
    netCfg.mqttPort = MQTT_PORT;
    saveNetConfig();
}

// Steady-state transmit cadence: mirror a real thermostat's 154 s heartbeat, and
// fire immediately when a bound zone's source state changes (dirty). One burst at
// a time, >= MIN_BURST_GAP apart, so no zone hogs the shared radio.
static const uint32_t TX_PERIOD_MS    = 154000;
static const uint32_t MIN_BURST_GAP_MS = 2000;
static uint32_t       lastBurstAt      = 0;

// Stale-source failsafe: if a bound zone's MQTT source goes quiet for this long,
// stop transmitting it entirely rather than relay a frozen ambient/setpoint
// forever (which the temperature-regulating receiver would happily keep acting
// on). Going silent makes the receiver see a lost thermostat and fall back to its
// own built-in handling (it beeps). MUST exceed the source's longest quiet
// interval -- raise it if a healthy zone gets dropped (nuisance beep). The W100
// reports periodically even when idle, so this is generous on purpose.
static const uint32_t ZONE_STALE_MS = 60UL * 60 * 1000;   // 60 min

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
    uint8_t  devId[3] = {DEV_ID[0], DEV_ID[1], DEV_ID[2]};  // ID the receiver learns
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
// RX frame decode (Step 2): chips -> Manchester bits -> sync -> frame -> CRC
// ---------------------------------------------------------------------------

// Assemble `nbytes` MSB-first bytes starting at bit offset `off` of the packed
// logical-bit array `bits` (nbits valid). Returns false if the window runs off
// the end.
static bool bitsToBytes(const uint8_t *bits, size_t nbits, size_t off,
                        uint8_t *out, size_t nbytes) {
    if (off + nbytes * 8 > nbits) return false;
    for (size_t i = 0; i < nbytes; i++) {
        uint8_t b = 0;
        for (int k = 0; k < 8; k++) {
            size_t bi = off + i * 8 + k;
            b = (uint8_t)((b << 1) | ((bits[bi >> 3] >> (7 - (bi & 7))) & 1));
        }
        out[i] = b;
    }
    return true;
}

// Decode one captured burst of chips into a Watts frame. Two things are unknown
// and brute-forced: the chip-pairing phase (both offsets tried) and the byte
// alignment (every bit offset tried). A candidate is only accepted when BOTH
// CRCs check, so wrong phase/alignment and noise are rejected outright -- the
// search keeps going on a CRC miss rather than reporting garbage. Returns true
// and prints once a CRC-valid frame is recovered.
//
// Sync is matched on just the second sync word (D3 91); the frame's length byte
// then sits at off+16. Matching 2 bytes instead of the full D3 91 D3 91 needs
// only half as many consecutive clean chips and gives two lock points per frame
// (a bit error in the first sync word no longer loses the frame). The first
// sync word, if also matched, decodes to a bogus length and fails CRC, so the
// scan simply advances to the real one -- CRC keeps it honest.
// Diagnostics for a burst that synced but never passed CRC. Lets loop() dump
// the recovered bytes so we can inspect frames the decoder can't yet validate
// (e.g. the suspected off/standby frames). Cleared at the top of every decode.
struct RxDiag {
    bool     sawSync;
    uint8_t  fr[16];
    uint8_t  crc8_calc, crc8_rx;
    uint16_t crc16_calc, crc16_rx;
};

// A decoded, CRC-valid frame handed back to the caller. Keeping decode free of
// policy: decodeBurst just produces one of these, and loop() prints it and
// decides what to do with it (logging, off-frame pairing capture, ...).
struct DecodedFrame {
    uint8_t id[3];
    uint8_t mode;
    float   amb;
    float   sp;
    uint8_t cfh;
};

// Pull the 16-byte frame at bit offset `foff` and validate both CRCs. On success
// fill `*out` and return true; on a CRC miss, stash the bytes in `diag` (for
// loop() to dump) and return false. `foff` is where the length byte should sit.
static bool validateFrameAt(const uint8_t *bits, size_t nbits, size_t foff,
                            DecodedFrame *out, RxDiag *diag) {
    uint8_t fr[16];
    if (!bitsToBytes(bits, nbits, foff, fr, 16)) return false;

    uint8_t  cfh      = fr[12];
    uint8_t  crc8     = (uint8_t)(crc8_raw(fr, 12) ^ 0xBE ^ cfh);
    uint16_t crc16    = crc16_cms(fr, 14);
    uint16_t crc16_rx = (uint16_t)((fr[14] << 8) | fr[15]);
    if (crc8 != fr[13] || crc16 != crc16_rx) {
        diag->sawSync    = true;
        memcpy(diag->fr, fr, 16);
        diag->crc8_calc  = crc8;  diag->crc8_rx  = fr[13];
        diag->crc16_calc = crc16; diag->crc16_rx = crc16_rx;
        return false;
    }

    out->id[0] = fr[5]; out->id[1] = fr[6]; out->id[2] = fr[7];
    out->mode  = fr[4];
    out->amb   = (int16_t)((fr[8]  << 8) | fr[9])  / 10.0f;
    out->sp    = (int16_t)((fr[10] << 8) | fr[11]) / 10.0f;
    out->cfh   = cfh;
    return true;
}

static bool decodeBurst(const uint8_t *chips, size_t nchips,
                        DecodedFrame *out, RxDiag *diag) {
    diag->sawSync = false;
    static uint8_t bits[96];               // packed logical bits (max ~512)
    for (int phase = 0; phase < 2; phase++) {
        memset(bits, 0, sizeof(bits));
        size_t nbits = 0;
        for (size_t i = phase; i + 1 < nchips; i += 2) {
            uint8_t a = chips[i], b = chips[i + 1];
            uint8_t bit;
#if MANCHESTER_ONE_IS_10
            if      (a == 1 && b == 0) bit = 1;
            else if (a == 0 && b == 1) bit = 0;
            else                       bit = 0;   // invalid pair (wrong phase/noise)
#else
            if      (a == 0 && b == 1) bit = 1;
            else if (a == 1 && b == 0) bit = 0;
            else                       bit = 0;
#endif
            if (nbits >= sizeof(bits) * 8) break;
            if (bit) bits[nbits >> 3] |= (uint8_t)(0x80 >> (nbits & 7));
            nbits++;
        }

        // Sync is D3 91 D3 91. Slide a window for one D3 91 and try the frame at
        // both byte offsets it could imply: +16 if this was the second sync word
        // (frame right after), +32 if it was the first (frame after the second
        // word). Either sync word surviving intact is then enough -- a bit error
        // in one no longer loses the frame. CRC gates both, so wrong guesses and
        // noise are rejected and the scan keeps going for a clean copy.
        for (size_t off = 0; off + 16 <= nbits; off++) {
            uint8_t s[2];
            bitsToBytes(bits, nbits, off, s, 2);
            if (s[0] != 0xD3 || s[1] != 0x91) continue;
            if (validateFrameAt(bits, nbits, off + 16, out, diag)) return true;
            if (validateFrameAt(bits, nbits, off + 32, out, diag)) return true;
        }
    }
    return false;   // no valid frame in either phase -- noise or a partial burst
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
static size_t buildFrame(uint8_t *f, const uint8_t *devId,
                         float amb_C, float sp_C, uint8_t cfh, uint8_t mode) {
    int16_t amb = (int16_t)lroundf(amb_C * 10.0f);
    int16_t sp  = (int16_t)lroundf(sp_C  * 10.0f);

    uint8_t *p = f;
    *p++ = 0x2A; *p++ = 0xAA; *p++ = 0xAA; *p++ = 0xAA;        // preamble
    *p++ = 0xD3; *p++ = 0x91; *p++ = 0xD3; *p++ = 0x91;        // sync

    uint8_t *app = p;                                          // byte 8 onward
    *p++ = 0x0D;                                               // length
    *p++ = 0xFF; *p++ = 0xFF; *p++ = 0xFE;                     // header
    *p++ = mode;                                               // mode
    *p++ = devId[0]; *p++ = devId[1]; *p++ = devId[2];         // device id
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

// Build, encode and transmit a single frame for device `devId`.
static void sendFrameOnce(const uint8_t *devId, float amb, float sp,
                          uint8_t cfh, uint8_t mode) {
    uint8_t frame[24];
    buildFrame(frame, devId, amb, sp, cfh, mode);   // 2a aa aa aa | sync | payload..crc16

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
static void sendBurst(const uint8_t *devId, float amb, float sp, uint8_t cfh,
                      uint8_t mode, int count, uint32_t gap_ms) {
    for (int i = 0; i < count; i++) {
        sendFrameOnce(devId, amb, (i & 1) ? sp + 0.1f : sp, cfh, mode);
        if (i < count - 1) delay(gap_ms);
    }
}

// ---------------------------------------------------------------------------
// Steady-state per-zone scheduler
// ---------------------------------------------------------------------------

// Map a bound zone's cached Z2M state to the Watts wire values. heat -> mode
// 0x02, cool -> 0x00 (byte-12 bit1). "off" keeps the last heat/cool bit but
// zeroes the setpoint -- exactly what a real thermostat transmits when switched
// off, which the temperature-regulating receiver reads as "no demand".
static void zoneTxValues(const Thermostat *t, float *outSp, uint8_t *outMode) {
    *outMode = (strcmp(t->lastMode, "cool") == 0) ? 0x00 : 0x02;
    *outSp   = (strcmp(t->lastMode, "off") == 0) ? 0.0f : t->lastSp;
}

// --- Call-for-heat P-loop driver (logic in cfh.h) --------------------------

// Advance every bound zone's call-for-heat loop once per loop() tick. A flag flip
// marks the zone dirty so the scheduler transmits the new state promptly rather
// than waiting for the 154 s heartbeat. Stale zones aren't transmitted, so their
// loop is paused and re-anchored on recovery (handled in serviceScheduler).
static void serviceControlLoops() {
    uint32_t now = millis();
    for (int i = 0; i < MAX_THERMOSTATS; i++) {
        ZoneBinding &b = bindings[i];
        if (!b.used || b.stale) continue;
        Thermostat *t = zoneThermostat(&b);
        if (!t || isnan(t->lastTemp) || isnan(t->lastSp)) continue;
        float sp; uint8_t mode;
        zoneTxValues(t, &sp, &mode);   // P-loop uses the on-wire setpoint
        (void)mode;
        uint8_t prev = b.ctrl.flag;
        if (cfhUpdate(&b.ctrl, sp, t->lastTemp, now) != prev) b.dirty = true;
    }
}

// Transmit at most one A-B-A burst per call, >= MIN_BURST_GAP apart so no zone
// hogs the shared radio. A zone whose source just changed (dirty) jumps ahead of
// zones merely due for their 154 s heartbeat. Skipped while RX is active -- the
// radio is in async-OOK receive config then (e.g. during pairing), and steady
// heartbeats resume once RX is off. Call every loop().
static void serviceScheduler() {
    if (rxActive) return;
    uint32_t now = millis();
    if (now - lastBurstAt < MIN_BURST_GAP_MS) return;

    Thermostat  *zt = nullptr;
    ZoneBinding *zb = nullptr;

    // Pass 0: a bound zone whose state changed. Pass 1: one due for a heartbeat.
    for (int pass = 0; pass < 2 && !zb; pass++) {
        for (int i = 0; i < thermostatCount; i++) {
            Thermostat &t = thermostats[i];
            if (isnan(t.lastTemp) || isnan(t.lastSp)) continue;   // no data yet
            ZoneBinding *b = findBinding(t.name);
            if (!b) continue;                                     // unbound

            // Stale-source failsafe: stop transmitting a zone whose source has
            // gone quiet, rather than relay frozen values. Going silent lets the
            // receiver flag a lost thermostat and fall back on its own.
            //
            // lastUpdateMs is written by the Z2M state callback on the AsyncTCP
            // task, concurrently with this loop. Snapshot it once, and compute the
            // age signed: a message landing just after we sampled `now` leaves
            // lastUpdateMs a few ms ahead of `now`, and an unsigned (now - lu)
            // would underflow to ~2^32 and spuriously trip the failsafe (the
            // "stale (4294967s)" flapping). A negative signed age == fresh.
            uint32_t lu  = t.lastUpdateMs;            // single concurrent read
            int32_t  age = (int32_t)(now - lu);
            if (lu != 0 && age > (int32_t)ZONE_STALE_MS) {
                if (!b->stale) {
                    b->stale          = true;
                    b->ctrl.ctrlInit  = false;   // re-anchor the P-loop on recovery
                    b->ctrl.flag      = CFH_IDLE;
                    Serial.printf("zone \"%s\": source stale (%lus) -- dropping TX; "
                                  "receiver will flag lost thermostat\n",
                                  t.name, (unsigned long)(age / 1000));
                    publishZoneState(b);   // reflect the drop in HA immediately
                }
                continue;
            }

            bool ready = (pass == 0)
                         ? b->dirty
                         : (b->lastTxAt == 0 || now - b->lastTxAt >= TX_PERIOD_MS);
            if (ready) { zt = &t; zb = b; break; }
        }
    }
    if (!zb) return;

    float sp; uint8_t mode;
    zoneTxValues(zt, &sp, &mode);
    sendBurst(zb->devId, zt->lastTemp, sp, zb->ctrl.flag, mode, 3, 400);   // A-B-A
    zb->lastTxAt = millis();
    zb->dirty    = false;
    lastBurstAt  = millis();
    Serial.printf("TX zone \"%s\": id=%02X%02X%02X amb=%.1f sp=%.1f mode=0x%02X cfh=0x%02X\n",
                  zt->name, zb->devId[0], zb->devId[1], zb->devId[2],
                  zt->lastTemp, sp, mode, zb->ctrl.flag);
    publishZoneState(zb);   // reflect the just-sent values in HA
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
// RX mode control
// ---------------------------------------------------------------------------
static void rxEnable() {
    ELECHOUSE_cc1101.SpiStrobe(CC1101_SIDLE);
    // GDO0 -> asynchronous serial data output (raw demodulated OOK envelope).
    ELECHOUSE_cc1101.SpiWriteReg(CC1101_IOCFG0, 0x0D);
    // PKTCTRL0 = 0x30: asynchronous serial mode, no CRC, no whitening.
    ELECHOUSE_cc1101.SpiWriteReg(CC1101_PKTCTRL0, 0x30);
    // MDMCFG4 = 0x86: RX channel BW ~203 kHz (covers the ~+73 kHz device
    // offset), keep DRATE_E=6 to match the TX chip rate.
    ELECHOUSE_cc1101.SpiWriteReg(CC1101_MDMCFG4, 0x86);

    pulseHead = pulseTail = 0;
    lastEdgeUs = micros();
    pinMode(PIN_GDO0, INPUT);
    attachInterrupt(digitalPinToInterrupt(PIN_GDO0), gdo0Isr, CHANGE);
    ELECHOUSE_cc1101.SpiStrobe(CC1101_SRX);
    rxActive = true;
    Serial.println("RX: listening (async OOK, raw pulse dump)");
}

static void rxDisable() {
    detachInterrupt(digitalPinToInterrupt(PIN_GDO0));
    ELECHOUSE_cc1101.SpiStrobe(CC1101_SIDLE);
    // Restore the TX-capable packet config (FIFO, fixed length, no CRC).
    ELECHOUSE_cc1101.SpiWriteReg(CC1101_PKTCTRL0, 0x00);
    ELECHOUSE_cc1101.SpiWriteReg(CC1101_IOCFG0, 0x2E);   // GDO0 high-impedance
    rxActive = false;
    Serial.println("RX: stopped");
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
                t.lastTemp = NAN;
                t.lastSp   = NAN;
                t.lastMode[0] = '\0';
                t.lastUpdateMs = 0;
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

    // Any message on this topic -- even a humidity-only republish -- proves the
    // thermostat is alive, so it's the liveness signal for the stale failsafe.
    // If the zone had been dropped as stale, the source just came back.
    t->lastUpdateMs = millis();
    ZoneBinding *b  = findBinding(t->name);
    if (b && b->stale) {
        b->stale = false;
        Serial.printf("zone \"%s\": source recovered -- resuming TX\n", t->name);
        publishZoneState(b);   // a recovery on an idle (unchanged) report still
                               // matters -- surface it before the early-return below
    }

    float temp = doc[t->tempField] | NAN;
    float sp   = doc[t->spField]   | NAN;
    // system_mode is a string enum (off/heat/cool/auto), not a number.
    const char *mode = t->modeField[0] ? (doc[t->modeField] | "") : "";

    // Update the cache, tracking whether anything we care about actually moved.
    // NaN means the field was absent from this (partial) update -- keep the old
    // value. Comparisons against the NaN sentinel are always true, so the first
    // real reading counts as a change.
    bool changed = false;
    if (!isnan(temp) && temp != t->lastTemp) { t->lastTemp = temp; changed = true; }
    if (!isnan(sp)   && sp   != t->lastSp)   { t->lastSp   = sp;   changed = true; }
    if (mode[0] && strcmp(mode, t->lastMode) != 0) {
        strlcpy(t->lastMode, mode, sizeof(t->lastMode));
        changed = true;
    }
    if (!changed) return;   // redundant republish (e.g. humidity-only) -- ignore

    // A real change on a bound zone means the receiver needs the new values now,
    // not at the next 154 s heartbeat -- flag it for the scheduler. Also push the
    // new source values to HA right away (the burst follows within MIN_BURST_GAP,
    // but the state blob shouldn't wait on the radio).
    if (b) { b->dirty = true; publishZoneState(b); }

    char tbuf[8], sbuf[8];
    if (isnan(t->lastTemp)) strcpy(tbuf, "--"); else snprintf(tbuf, sizeof(tbuf), "%.1f", t->lastTemp);
    if (isnan(t->lastSp))   strcpy(sbuf, "--"); else snprintf(sbuf, sizeof(sbuf), "%.1f", t->lastSp);
    Serial.printf("Z2M state %s: temp=%s setpoint=%s mode=%s\n",
                  t->name, tbuf, sbuf, t->modeField[0] ? t->lastMode : "n/a");
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
// Bridge observability over MQTT
// ---------------------------------------------------------------------------
// Two topics, both retained:
//   AVAIL_TOPIC -- online/offline liveness. We publish "online" on connect and
//     register "offline" as the LWT, so the broker announces an ungraceful death
//     (brownout, WiFi drop, crash) on our behalf -- exactly when we can't.
//   DIAG_TOPIC  -- a single JSON blob of bridge health, republished every
//     DIAG_PERIOD_MS. One publish backs many HA sensors via value_template, and
//     being retained it survives a reconnect.
static const char *AVAIL_TOPIC = "watts-bridge/status";
static const char *DIAG_TOPIC  = "watts-bridge/diag";
static const uint32_t DIAG_PERIOD_MS = 30000;
static uint32_t       lastDiagAt     = 0;
static const char    *resetReasonStr = "unknown";   // latched once at boot

static const char *resetReasonName(esp_reset_reason_t r) {
    switch (r) {
        case ESP_RST_POWERON:   return "power-on";
        case ESP_RST_SW:        return "sw-reset";
        case ESP_RST_PANIC:     return "panic";
        case ESP_RST_INT_WDT:   return "int-wdt";
        case ESP_RST_TASK_WDT:  return "task-wdt";
        case ESP_RST_WDT:       return "wdt";
        case ESP_RST_BROWNOUT:  return "brownout";
        case ESP_RST_DEEPSLEEP: return "deepsleep";
        case ESP_RST_EXT:       return "ext";
        default:                return "unknown";
    }
}

// Publish the current health blob. last_tx_age is the domain-specific field: -1
// until the first burst, then seconds since lastBurstAt -- crossing ~160 s while
// "online" still holds means the radio is wedged though MQTT/WiFi are fine.
static uint16_t publishDiag() {
    JsonDocument doc;
    doc["rssi"]         = WiFi.RSSI();
    doc["uptime"]       = millis() / 1000;           // wraps at ~49 days; fine here
    doc["reset_reason"] = resetReasonStr;
    doc["last_tx_age"]  = lastBurstAt ? (int32_t)((millis() - lastBurstAt) / 1000)
                                      : -1;
    doc["ip"]           = WiFi.localIP().toString();
    doc["fw"]           = FW_VERSION;
    String body;
    serializeJson(doc, body);
    return mqtt.publish(DIAG_TOPIC, 1, true, body.c_str());
}

// One diagnostic sensor's HA discovery config, sourced from the DIAG blob.
static uint16_t publishSensorDiscovery(const char *object, const char *name,
                                       const char *valueKey, const char *devClass,
                                       const char *unit) {
    JsonDocument doc;
    doc["name"] = name;
    char uid[48];
    snprintf(uid, sizeof(uid), "watts_bridge_%s", object);
    doc["unique_id"]   = uid;
    doc["state_topic"] = DIAG_TOPIC;
    char tmpl[48];
    snprintf(tmpl, sizeof(tmpl), "{{ value_json.%s }}", valueKey);
    doc["value_template"] = tmpl;
    if (devClass) doc["device_class"]        = devClass;
    if (unit)     doc["unit_of_measurement"] = unit;
    doc["entity_category"]       = "diagnostic";
    doc["availability_topic"]    = AVAIL_TOPIC;
    doc["payload_available"]     = "online";
    doc["payload_not_available"] = "offline";
    doc["device"]["identifiers"][0] = "watts_bridge";
    char topic[80];
    snprintf(topic, sizeof(topic),
             "homeassistant/sensor/watts_bridge/%s/config", object);
    String body;
    serializeJson(doc, body);
    return mqtt.publish(topic, 1, true, body.c_str());
}

// The bridge's connectivity binary_sensor: it carries the full device block (the
// diagnostic sensors reference it by shared identifier) and uses the LWT topic,
// so HA greys every entity out together the moment the bridge dies. Returns the
// publish packet-id (0 = send buffer full).
static uint16_t publishBridgeConnectivity() {
    JsonDocument doc;
    doc["name"]         = nullptr;   // null -> HA shows it as the device's main
                                     // entity ("Watts WFHT Bridge"), no doubled name
    doc["unique_id"]    = "watts_bridge_status";
    doc["device_class"] = "connectivity";
    doc["state_topic"]  = AVAIL_TOPIC;
    doc["payload_on"]   = "online";
    doc["payload_off"]  = "offline";
    doc["entity_category"] = "diagnostic";
    JsonObject dev = doc["device"].to<JsonObject>();
    dev["identifiers"][0] = "watts_bridge";
    dev["name"]           = "Watts WFHT Bridge";
    dev["manufacturer"]   = "DIY";
    dev["model"]          = "ESP32+CC1101";
    String body;
    serializeJson(doc, body);
    return mqtt.publish("homeassistant/binary_sensor/watts_bridge/status/config",
                        1, true, body.c_str());
}

// The bridge's diagnostic sensors, all backed by the retained DIAG blob. A table
// (rather than inline calls) so the paced re-advertise sequencer can index them.
struct BridgeSensor { const char *object, *name, *valueKey, *devClass, *unit; };
static const BridgeSensor BRIDGE_SENSORS[] = {
    {"rssi",         "WiFi signal",       "rssi",         "signal_strength", "dBm"},
    {"uptime",       "Uptime",            "uptime",       "duration",        "s"},
    {"reset_reason", "Reset reason",      "reset_reason", nullptr,           nullptr},
    {"last_tx_age",  "Last transmit age", "last_tx_age",  "duration",        "s"},
};
static const int BRIDGE_SENSOR_COUNT = sizeof(BRIDGE_SENSORS) / sizeof(BRIDGE_SENSORS[0]);

// Publish one bridge config message by global step index: 0 = the connectivity
// binary_sensor (carries the device block), 1.. = the diagnostic sensors.
static uint16_t advertiseBridgeStep(int step) {
    if (step == 0) return publishBridgeConnectivity();
    const BridgeSensor &s = BRIDGE_SENSORS[step - 1];
    return publishSensorDiscovery(s.object, s.name, s.valueKey, s.devClass, s.unit);
}
static const int BRIDGE_STEP_COUNT = 1 + BRIDGE_SENSOR_COUNT;

// Per-bound-zone state -> HA. Each zone surfaces as its own HA device nested
// under the bridge (via_device), carrying what the bridge is actually
// transmitting to the Watts receiver for that zone -- distinct from the W100's
// own HA entity, which shows the source. State is a retained per-zone JSON blob;
// discovery is (re)published on bind and on (re)connect, and cleared on unbind
// so the entities disappear from HA cleanly.

// friendly_name -> a topic/object-id-safe slug (lowercase alnum + underscore).
static void zoneSlug(const char *name, char *out, size_t n) {
    size_t j = 0;
    for (size_t i = 0; name[i] && j + 1 < n; i++) {
        char c = name[i];
        if (c >= 'A' && c <= 'Z') c += 32;
        out[j++] = ((c >= 'a' && c <= 'z') || (c >= '0' && c <= '9')) ? c : '_';
    }
    out[j] = 0;
}

// The discovered thermostat backing a binding (NULL until Z2M first reports it).
static Thermostat *zoneThermostat(const ZoneBinding *b) {
    for (int i = 0; i < thermostatCount; i++)
        if (strcmp(thermostats[i].name, b->name) == 0) return &thermostats[i];
    return nullptr;
}

// Publish (retained) what the bridge is transmitting for one zone + its health.
// status: pending (bound, no source data yet) / active (transmitting) / stale
// (source went quiet, TX dropped by the failsafe). tx_* are the wire values the
// next burst would carry.
static uint16_t publishZoneState(const ZoneBinding *b) {
    if (!mqtt.connected()) return 0;
    char slug[40];  zoneSlug(b->name, slug, sizeof(slug));
    char topic[64]; snprintf(topic, sizeof(topic), "watts-bridge/zone/%s", slug);

    JsonDocument doc;
    char id[7];
    snprintf(id, sizeof(id), "%02X%02X%02X", b->devId[0], b->devId[1], b->devId[2]);
    doc["watts_id"]    = id;
    doc["last_tx_age"] = b->lastTxAt ? (int32_t)((millis() - b->lastTxAt) / 1000) : -1;

    Thermostat *t = zoneThermostat(b);
    if (!t || isnan(t->lastTemp) || isnan(t->lastSp)) {
        doc["status"] = "pending";
        doc["mode"]   = "unknown";
    } else {
        float sp; uint8_t mode;
        zoneTxValues(t, &sp, &mode);
        doc["status"]        = b->stale ? "stale" : "active";
        doc["tx_ambient"]    = t->lastTemp;
        doc["tx_setpoint"]   = sp;
        doc["mode"]          = t->lastMode[0] ? t->lastMode : "heat";
        doc["call_for_heat"] = (!b->stale && b->ctrl.flag == CFH_CALL) ? "calling" : "idle";
    }
    String body;
    serializeJson(doc, body);
    return mqtt.publish(topic, 1, true, body.c_str());
}

// One sensor's HA discovery config for a zone, sourced from the zone state blob.
static uint16_t publishZoneSensor(const char *slug, const char *zoneName,
                                  const char *object, const char *label,
                                  const char *valueKey, const char *devClass,
                                  const char *unit, bool diagnostic) {
    JsonDocument doc;
    doc["name"] = label;
    char uid[64];
    snprintf(uid, sizeof(uid), "watts_zone_%s_%s", slug, object);
    doc["unique_id"] = uid;
    char st[64];
    snprintf(st, sizeof(st), "watts-bridge/zone/%s", slug);
    doc["state_topic"] = st;
    char tmpl[48];
    snprintf(tmpl, sizeof(tmpl), "{{ value_json.%s }}", valueKey);
    doc["value_template"] = tmpl;
    if (devClass) doc["device_class"]        = devClass;
    if (unit)     doc["unit_of_measurement"] = unit;
    if (diagnostic) doc["entity_category"]   = "diagnostic";
    doc["availability_topic"]    = AVAIL_TOPIC;
    doc["payload_available"]     = "online";
    doc["payload_not_available"] = "offline";
    JsonObject dev = doc["device"].to<JsonObject>();
    char devId[48];
    snprintf(devId, sizeof(devId), "watts_zone_%s", slug);
    dev["identifiers"][0] = devId;
    char devName[80];
    snprintf(devName, sizeof(devName), "Watts zone: %s", zoneName);
    dev["name"]       = devName;
    dev["via_device"] = "watts_bridge";   // nests this zone under the bridge device
    char topic[96];
    snprintf(topic, sizeof(topic),
             "homeassistant/sensor/watts_zone_%s/%s/config", slug, object);
    String body;
    serializeJson(doc, body);
    return mqtt.publish(topic, 1, true, body.c_str());
}

// The set of sensors making up a zone's HA device, backed by the retained zone
// state blob. The single source of truth for publish (the re-advertise
// sequencer), clear (on unbind), and the object list -- so they can't drift apart.
struct ZoneSensor {
    const char *object, *label, *valueKey, *devClass, *unit;
    bool diagnostic;
};
static const ZoneSensor ZONE_SENSORS[] = {
    {"status",   "Status",            "status",      nullptr,       nullptr, false},
    {"setpoint", "Setpoint",          "tx_setpoint", "temperature", "°C",    false},
    {"ambient",  "Ambient",           "tx_ambient",  "temperature", "°C",    false},
    {"mode",     "Mode",              "mode",          nullptr,     nullptr, false},
    {"cfh",      "Call for heat",     "call_for_heat", nullptr,     nullptr, false},
    {"tx_age",   "Last transmit age", "last_tx_age",   "duration",  "s",     true},
    {"watts_id", "Watts ID",          "watts_id",    nullptr,       nullptr, true},
};
static const int ZONE_SENSOR_COUNT = sizeof(ZONE_SENSORS) / sizeof(ZONE_SENSORS[0]);

// Publish one of a zone's discovery configs by index. Returns the packet-id
// (0 = send buffer full) so the sequencer knows whether it landed.
static uint16_t advertiseZoneSensorStep(const ZoneBinding *b, int step) {
    char slug[40]; zoneSlug(b->name, slug, sizeof(slug));
    const ZoneSensor &s = ZONE_SENSORS[step];
    return publishZoneSensor(slug, b->name, s.object, s.label, s.valueKey,
                             s.devClass, s.unit, s.diagnostic);
}

// Remove a zone's entities from HA: an empty retained payload on each config
// topic deletes the entity, and we clear the retained state blob too.
static void clearZoneDiscovery(const ZoneBinding *b) {
    if (!mqtt.connected()) return;
    char slug[40]; zoneSlug(b->name, slug, sizeof(slug));
    char topic[96];
    for (const ZoneSensor &s : ZONE_SENSORS) {
        snprintf(topic, sizeof(topic),
                 "homeassistant/sensor/watts_zone_%s/%s/config", slug, s.object);
        mqtt.publish(topic, 1, true, "");
    }
    snprintf(topic, sizeof(topic), "watts-bridge/zone/%s", slug);
    mqtt.publish(topic, 1, true, "");
}

// Refresh just the state blobs (keeps last_tx_age moving) -- used on the heartbeat.
static void publishAllZoneStates() {
    for (int i = 0; i < MAX_THERMOSTATS; i++)
        if (bindings[i].used) publishZoneState(&bindings[i]);
}

// ---------------------------------------------------------------------------
// Paced re-advertise
// ---------------------------------------------------------------------------
// On (re)connect we must (re)publish ~7 retained configs per zone plus the
// bridge's own -- 20+ messages for a few zones. Firing them all back-to-back in
// the onConnect callback overran the AsyncTCP send buffer: AsyncMqttClient drops
// the overflow (publish() returns packet-id 0) and, since publishAllZones ran
// last, the later zones' entities never registered in HA -- visible as a zone
// transmitting on-air (rtl_433 sees it) but stuck "stale" in HA. Instead we
// publish ONE message per loop() tick and only advance the cursor when the broker
// accepted it (pid != 0); a 0 holds the cursor and retries next tick, so the
// burst self-paces to the socket's drain rate and nothing is silently lost.
enum AdvPhase : uint8_t { ADV_IDLE, ADV_BRIDGE, ADV_DIAG, ADV_ZONES, ADV_DONE };
static AdvPhase advPhase     = ADV_IDLE;
static int      advBridgeStep = 0;   // 0..BRIDGE_STEP_COUNT-1
static int      advZoneIdx    = 0;   // index into bindings[]
static int      advZoneStep   = 0;   // 0..ZONE_SENSOR_COUNT  (last = state blob)

// Request a full paced (re)advertise from the start. Cheap: just rewinds the
// cursor; loop()'s serviceAdvertise() does the work. Safe to call mid-advertise.
static void startAdvertise() {
    advPhase      = ADV_BRIDGE;
    advBridgeStep = 0;
    advZoneIdx    = 0;
    advZoneStep   = 0;
}

// Publish at most one pending retained config per call. Drained from loop().
static void serviceAdvertise() {
    if (advPhase == ADV_IDLE || advPhase == ADV_DONE) return;
    if (!mqtt.connected()) { advPhase = ADV_IDLE; return; }   // reconnect re-arms it

    switch (advPhase) {
        case ADV_BRIDGE:
            if (advertiseBridgeStep(advBridgeStep))
                if (++advBridgeStep >= BRIDGE_STEP_COUNT) advPhase = ADV_DIAG;
            return;
        case ADV_DIAG:
            if (publishDiag()) { lastDiagAt = millis(); advPhase = ADV_ZONES; }
            return;
        case ADV_ZONES:
            while (advZoneIdx < MAX_THERMOSTATS && !bindings[advZoneIdx].used)
                advZoneIdx++;
            if (advZoneIdx >= MAX_THERMOSTATS) { advPhase = ADV_DONE; return; }
            if (advZoneStep < ZONE_SENSOR_COUNT) {
                if (advertiseZoneSensorStep(&bindings[advZoneIdx], advZoneStep))
                    advZoneStep++;
            } else {                              // last step: seed the state blob
                if (publishZoneState(&bindings[advZoneIdx])) {
                    advZoneStep = 0;
                    advZoneIdx++;
                }
            }
            return;
        default:
            return;
    }
}

// ---------------------------------------------------------------------------
// MQTT init (connect + subscribe to the Z2M inventory for discovery)
// ---------------------------------------------------------------------------
// AsyncMqttClient is non-blocking: connect() returns immediately and the result
// arrives on a callback. We report over Serial in the same shape as the CC1101
// init ("MQTT: ..."). On a disconnect we arm a one-shot retry in loop().
static void initMqtt() {
    // Settings come from runtime NetConfig (NVS), not the config.h macros --
    // AsyncMqttClient stores the pointers, so netCfg's static buffers must outlive
    // the connection (they're file-scope globals, so they do).
    mqtt.setServer(netCfg.mqttHost, netCfg.mqttPort);
    mqtt.setClientId(netCfg.mqttClientId);
    // LWT: the broker publishes "offline" (retained) if we drop ungracefully.
    mqtt.setWill(AVAIL_TOPIC, 1, true, "offline");
    if (strlen(netCfg.mqttUser) > 0)
        mqtt.setCredentials(netCfg.mqttUser, netCfg.mqttPass);

    mqtt.onConnect([](bool sessionPresent) {
        Serial.printf("MQTT: connected to %s:%u\n", netCfg.mqttHost, netCfg.mqttPort);
        mqttRetryAt = 0;
        // Overwrite the LWT's stale "offline" from any prior death immediately,
        // then kick off a paced re-advertise of the bridge + every bound zone's
        // retained configs (restored from NVS before connect). Pacing is essential:
        // firing them all here overran the send buffer and silently dropped the
        // later zones -- see serviceAdvertise(). loop() drains it one msg per tick.
        mqtt.publish(AVAIL_TOPIC, 1, true, "online");
        startAdvertise();
        // Subscribe to the retained Z2M inventory; the broker delivers it at once.
        uint16_t pid = mqtt.subscribe(Z2M_DEVICES_TOPIC, 0);
        Serial.printf("MQTT: subscribed to %s (pid %u)\n", Z2M_DEVICES_TOPIC, pid);
    });
    mqtt.onDisconnect([](AsyncMqttClientDisconnectReason reason) {
        Serial.printf("MQTT: disconnected (reason %d), retry in 5s\n", (int)reason);
        mqttRetryAt = millis() + 5000;
    });
    mqtt.onMessage(onMqttMessage);

    Serial.printf("MQTT: connecting to %s:%u\n", netCfg.mqttHost, netCfg.mqttPort);
    mqtt.connect();
}

// ---------------------------------------------------------------------------
// HTTP routes
// ---------------------------------------------------------------------------

// Parse a 6-hex-digit device ID ("349E48") into 3 bytes. False on bad input.
static bool parseHexId(const String &s, uint8_t out[3]) {
    if (s.length() != 6) return false;
    for (int i = 0; i < 3; i++) {
        char  buf[3] = { s[i * 2], s[i * 2 + 1], 0 };
        char *end;
        long  v = strtol(buf, &end, 16);
        if (*end != 0) return false;
        out[i] = (uint8_t)v;
    }
    return true;
}

// ---------------------------------------------------------------------------
// Captive portal (first-boot / no-creds WiFi + MQTT provisioning)
// ---------------------------------------------------------------------------
// When there are no usable WiFi creds (empty SSID) or STA association times out,
// the device comes up as its own open AP with a DNS catch-all so any phone/laptop
// that joins is bounced to a setup form. Saving writes NetConfig to NVS and the
// device reboots into normal STA mode. Reuses the one AsyncWebServer; portal and
// normal routes are mutually exclusive (setup() returns early into one or other).
static DNSServer dnsServer;
static bool      portalActive  = false;
static uint32_t  portalRebootAt = 0;   // deferred restart so the HTTP reply flushes

static const char PORTAL_HTML[] PROGMEM = R"HTML(<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Watts Bridge setup</title>
<style>
 :root{color-scheme:dark light}
 body{font:15px/1.5 system-ui,sans-serif;margin:0;padding:1.2rem;max-width:480px}
 h1{font-size:1.3rem;margin:0 0 .2rem}
 .sub{color:#888;margin:0 0 1.2rem;font-size:.85rem}
 fieldset{border:1px solid #8884;border-radius:8px;padding:1rem;margin:0 0 1.2rem}
 legend{color:#888;font-size:.8rem;padding:0 .4rem}
 label{display:block;font-size:.8rem;color:#888;margin:.6rem 0 .2rem}
 input{font:inherit;padding:.5rem;width:100%;box-sizing:border-box;
   border:1px solid #8886;border-radius:6px;background:transparent;color:inherit}
 button{font:inherit;padding:.6rem 1rem;border:0;border-radius:6px;cursor:pointer;
   background:#1976d2;color:#fff;width:100%}
 .row{display:flex;gap:.8rem}.row>div{flex:1}
 .pw{position:relative}
 .pw input{padding-right:2.6rem}
 .pw button{position:absolute;right:.2rem;top:50%;transform:translateY(-50%);
   width:auto;background:none;color:#888;padding:.3rem;display:flex}
 .pw button svg{width:18px;height:18px;display:block}
</style></head><body>
<h1>Watts Bridge setup</h1>
<p class="sub">Connect this bridge to your WiFi and MQTT broker.</p>
<form method="post" action="/save">
 <fieldset><legend>WiFi</legend>
  <label>Network (SSID)</label><input name="ssid" required autofocus>
  <label>Password</label>
  <div class="pw"><input name="pass" type="password">
   <button type="button" onclick="tog(this)" aria-label="Show password"></button></div>
 </fieldset>
 <fieldset><legend>MQTT broker</legend>
  <div class="row">
   <div><label>Host</label><input name="mhost" placeholder="192.168.1.10"></div>
   <div style="flex:0 0 90px"><label>Port</label><input name="mport" value="1883"></div>
  </div>
  <label>Username (blank = anonymous)</label><input name="muser">
  <label>Password</label>
  <div class="pw"><input name="mpass" type="password">
   <button type="button" onclick="tog(this)" aria-label="Show password"></button></div>
 </fieldset>
 <fieldset><legend>Web UI login</legend>
  <label>Username</label><input name="huser" value="admin">
  <label>Password (blank = no login required)</label>
  <div class="pw"><input name="hpass" type="password">
   <button type="button" onclick="tog(this)" aria-label="Show password"></button></div>
 </fieldset>
 <button type="submit">Save &amp; reboot</button>
</form>
<script>
const EYE='<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><path d="M1 12s4-8 11-8 11 8 11 8-4 8-11 8-11-8-11-8z"/><circle cx="12" cy="12" r="3"/></svg>';
const EYEOFF='<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><path d="M17.94 17.94A10.07 10.07 0 0 1 12 20c-7 0-11-8-11-8a18.45 18.45 0 0 1 5.06-5.94M9.9 4.24A9.12 9.12 0 0 1 12 4c7 0 11 8 11 8a18.5 18.5 0 0 1-2.16 3.19m-6.72-1.07a3 3 0 1 1-4.24-4.24"/><line x1="1" y1="1" x2="23" y2="23"/></svg>';
document.querySelectorAll(".pw button").forEach(b=>b.innerHTML=EYE);
function tog(b){const i=b.previousElementSibling;const s=i.type==="password";
 i.type=s?"text":"password";b.innerHTML=s?EYEOFF:EYE;}
</script>
</body></html>)HTML";

static void setupPortalRoutes() {
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *req) {
        req->send(200, "text/html", PORTAL_HTML);
    });
    server.on("/save", HTTP_POST, [](AsyncWebServerRequest *req) {
        auto field = [&](const char *k) -> String {
            return req->hasParam(k, true) ? req->getParam(k, true)->value() : String();
        };
        String ssid = field("ssid");
        if (ssid.isEmpty()) { req->send(400, "text/plain", "ssid required"); return; }

        memset(&netCfg, 0, sizeof(netCfg));
        strlcpy(netCfg.wifiSsid, ssid.c_str(),          sizeof(netCfg.wifiSsid));
        strlcpy(netCfg.wifiPass, field("pass").c_str(), sizeof(netCfg.wifiPass));
        strlcpy(netCfg.mqttHost, field("mhost").c_str(),sizeof(netCfg.mqttHost));
        int mp = field("mport").toInt();
        netCfg.mqttPort = (mp > 0 && mp <= 65535) ? (uint16_t)mp : 1883;
        strlcpy(netCfg.mqttUser, field("muser").c_str(),sizeof(netCfg.mqttUser));
        strlcpy(netCfg.mqttPass, field("mpass").c_str(),sizeof(netCfg.mqttPass));
        strlcpy(netCfg.mqttClientId, MQTT_CLIENT_ID,    sizeof(netCfg.mqttClientId));
        strlcpy(netCfg.httpUser, field("huser").c_str(),sizeof(netCfg.httpUser));
        strlcpy(netCfg.httpPass, field("hpass").c_str(),sizeof(netCfg.httpPass));
        saveNetConfig();
        Serial.printf("PORTAL: saved ssid=\"%s\" mqtt=%s:%u -- rebooting\n",
                      netCfg.wifiSsid, netCfg.mqttHost, netCfg.mqttPort);

        String body = "<!doctype html><meta charset=utf-8>"
                      "<meta http-equiv=refresh content='10;url=/'>"
                      "<body style=\"font:15px system-ui;padding:1.2rem\">"
                      "Saved. Rebooting and connecting to <b>" + ssid + "</b>…<br>"
                      "If it can't join, the setup AP will reappear.</body>";
        req->send(200, "text/html", body);
        portalRebootAt = millis() + 1500;   // let the reply flush, then restart in loop()
    });
    // Captive-portal probes (and anything else): bounce to the form so the OS
    // pops its "sign in to network" sheet instead of reporting no internet.
    server.onNotFound([](AsyncWebServerRequest *req) {
        req->redirect("/");
    });
}

static void startPortal() {
    portalActive = true;
    WiFi.mode(WIFI_AP);
    char ap[32];
    snprintf(ap, sizeof(ap), "watts-bridge-%04X",
             (uint16_t)(ESP.getEfuseMac() >> 32));
    WiFi.softAP(ap);
    IPAddress ip = WiFi.softAPIP();
    dnsServer.start(53, "*", ip);   // catch-all so every lookup resolves to us
    setupPortalRoutes();
    server.begin();
    Serial.printf("PORTAL: open AP \"%s\" -> http://%s/\n", ap, ip.toString().c_str());
}

// Self-contained binding-config UI served at GET /. No external assets (the ESP
// isn't internet-facing for a CDN) -- it just drives the JSON endpoints already
// here: /thermostats, /bindings, /pair-listen, /pair-status, /bind, /unbind.
// The pairing flow auto-fills the Watts ID so binding needs no typing.
static const char INDEX_HTML[] PROGMEM = R"HTML(<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Watts WFHT Bridge</title>
<style>
 :root{color-scheme:dark light}
 body{font:15px/1.5 system-ui,sans-serif;margin:0;padding:1.2rem;max-width:760px}
 h1{font-size:1.3rem;margin:0 0 .2rem}
 .sub{color:#888;margin:0 0 1.2rem;font-size:.85rem}
 section{border:1px solid #8884;border-radius:8px;padding:1rem;margin-bottom:1.2rem}
 h2{font-size:1rem;margin:0 0 .8rem}
 table{width:100%;border-collapse:collapse;font-size:.9rem}
 th,td{text-align:left;padding:.4rem .5rem;border-bottom:1px solid #8883}
 th{color:#888;font-weight:600}
 .id{font-family:ui-monospace,monospace}
 .badge{font-size:.72rem;padding:.1rem .45rem;border-radius:10px;background:#2e7d32;color:#fff}
 .badge.stale{background:#c62828}
 label{display:block;font-size:.8rem;color:#888;margin:.6rem 0 .2rem}
 select,input{font:inherit;padding:.45rem;width:100%;box-sizing:border-box;
   border:1px solid #8886;border-radius:6px;background:transparent;color:inherit}
 .row{display:flex;gap:.8rem;flex-wrap:wrap}
 .row>div{flex:1;min-width:160px}
 button{font:inherit;padding:.5rem .9rem;border:0;border-radius:6px;cursor:pointer;
   background:#1976d2;color:#fff;margin-top:.9rem}
 button.sec{background:#5557}
 button.sm{padding:.3rem .6rem;margin:0 .3rem 0 0;font-size:.8rem}
 button.danger{background:#b3261e;padding:.3rem .6rem;margin:0;font-size:.8rem}
 button:disabled{opacity:.5;cursor:default}
 #msg{min-height:1.2rem;font-size:.85rem;margin-top:.6rem}
 .ok{color:#2e7d32}.err{color:#c62828}.muted{color:#888}
</style></head><body>
<h1>Watts WFHT Bridge</h1>
<p class="sub">Bind a Home Assistant / Z2M thermostat to a Watts device ID.</p>

<section>
 <h2>Current bindings</h2>
 <table><thead><tr><th>Thermostat</th><th>Watts ID</th><th>Temp</th>
   <th>Set</th><th>Mode</th><th>TX age</th><th></th></tr></thead>
 <tbody id="rows"><tr><td colspan="7" class="muted">loading…</td></tr></tbody></table>
</section>

<section>
 <h2>Add / update binding</h2>
 <div class="row">
  <div><label>Thermostat</label><select id="name"></select></div>
  <div><label>Watts device ID (6 hex)</label>
   <input id="id" placeholder="349E48" maxlength="6" autocapitalize="characters"></div>
 </div>
 <button class="sec" id="cap">Capture ID from thermostat</button>
 <button id="bind">Bind</button>
 <div id="msg"></div>
</section>

<section>
 <h2>Device</h2>
 <button class="danger" onclick="resetWifi()">Reset WiFi / MQTT setup</button>
 <p class="muted" style="font-size:.8rem;margin:.6rem 0 0">Clears stored network
  config and reboots into the setup portal (join WiFi <span class="id">watts-bridge-…</span>).</p>
</section>

<script>
const $=s=>document.querySelector(s);
function msg(t,c){const m=$("#msg");m.textContent=t;m.className=c||"";}
async function jget(u){const r=await fetch(u);if(!r.ok)throw new Error(await r.text());
  return r.headers.get("content-type")?.includes("json")?r.json():r.text();}

function age(s){return s<0?"—":s<120?s+"s":Math.round(s/60)+"m";}
async function loadBindings(){
 try{const b=await jget("/bindings");
  $("#rows").innerHTML=b.length?b.map(z=>`<tr>
   <td>${z.name}</td><td class="id">${z.id}</td>
   <td>${z.temp??"—"}</td><td>${z.setpoint??"—"}</td><td>${z.mode??"—"}</td>
   <td>${age(z.last_tx_age_s)} ${z.stale?'<span class="badge stale">stale</span>':''}</td>
   <td style="white-space:nowrap">
     <button class="sec sm" onclick="pair('${z.id}',${z.temp??'null'},${z.setpoint??'null'})">Pair</button>
     <button class="danger" onclick="unbind('${z.name}')">Unbind</button></td>
   </tr>`).join(""):'<tr><td colspan="7" class="muted">no bindings yet</td></tr>';
 }catch(e){$("#rows").innerHTML=`<tr><td colspan="7" class="err">${e.message}</td></tr>`;}
}
async function loadThermostats(){
 try{const t=await jget("/thermostats");const sel=$("#name");const cur=sel.value;
  sel.innerHTML=t.length?t.map(x=>`<option value="${x.name}">${x.name}${x.bound?" (bound)":""}</option>`).join("")
    :'<option value="">none discovered yet</option>';
  if(cur)sel.value=cur;
 }catch(e){msg(e.message,"err");}
}
async function unbind(name){
 try{await jget("/unbind?name="+encodeURIComponent(name));msg("Unbound "+name,"ok");
  refresh();}catch(e){msg(e.message,"err");}
}
let pairTimer=null;
async function pair(id,amb,sp){
 if(pairTimer)return; // a pairing run is already streaming
 if(!confirm("Put the Watts receiver zone into pairing/learn mode first, then OK.\n\n"
   +"The bridge will transmit pairing frames for "+id+" for 30 s."))return;
 let q="/tx-pair?id="+id+"&duration=30";        // carry the zone's live values so the
 if(amb!=null)q+="&amb="+amb;                    // receiver learns a sane frame, not 0/0
 if(sp!=null)q+="&sp="+sp;
 try{await jget(q);}catch(e){return msg(e.message,"err");}
 document.querySelectorAll("button").forEach(b=>b.disabled=true);
 pairTimer=setInterval(async()=>{
  try{const s=await jget("/tx-pair-status");
   if(s.active){msg("pairing "+s.id+" — "+s.remaining_s+"s left","muted");}
   else{clearInterval(pairTimer);pairTimer=null;
    document.querySelectorAll("button").forEach(b=>b.disabled=false);
    msg("pairing frames sent for "+id+" — check the receiver","ok");}
  }catch(e){clearInterval(pairTimer);pairTimer=null;
   document.querySelectorAll("button").forEach(b=>b.disabled=false);
   msg(e.message,"err");}
 },1000);
}
$("#bind").onclick=async()=>{
 const name=$("#name").value, id=$("#id").value.trim().toUpperCase();
 if(!name)return msg("pick a thermostat","err");
 if(!/^[0-9A-F]{6}$/.test(id))return msg("ID must be 6 hex digits","err");
 try{const r=await jget(`/bind?name=${encodeURIComponent(name)}&id=${id}`);
  msg(r,"ok");$("#id").value="";refresh();}catch(e){msg(e.message,"err");}
};
let capTimer=null;
$("#cap").onclick=async()=>{
 if(capTimer){clearInterval(capTimer);capTimer=null;await jget("/pair-cancel");
  $("#cap").textContent="Capture ID from thermostat";msg("capture cancelled","muted");return;}
 try{await jget("/pair-listen");}catch(e){return msg(e.message,"err");}
 $("#cap").textContent="Cancel — switch a thermostat OFF…";
 msg("listening — set the Watts thermostat to OFF to capture its ID","muted");
 capTimer=setInterval(async()=>{
  try{const s=await jget("/pair-status");
   if(s.captured){clearInterval(capTimer);capTimer=null;
    $("#id").value=s.id;$("#cap").textContent="Capture ID from thermostat";
    const t=(s.amb!=null&&!isNaN(s.amb))?` (reads ${s.amb} °C — check it matches the room)`:"";
    msg("captured "+s.id+t+" — select the correct thermostat then Bind","ok");}
  }catch(e){clearInterval(capTimer);capTimer=null;msg(e.message,"err");}
 },1000);
};
async function resetWifi(){
 if(!confirm("Clear WiFi + MQTT config and reboot into the setup portal?"))return;
 try{await fetch("/reset-wifi");
  msg("rebooting into setup portal — reconnect to the watts-bridge-… WiFi","muted");
 }catch(e){msg(e.message,"err");}
}
function refresh(){loadBindings();loadThermostats();}
refresh();setInterval(()=>{if(!pairTimer)loadBindings();},5000);
</script></body></html>)HTML";

static void setupRoutes() {
    // Require a login on every operational route. Credentials come from the
    // runtime NetConfig (set in the captive portal, seeded from config.h); an
    // empty password serves the UI openly. Added before any server.on() so the
    // middleware wraps all handlers registered below.
    if (strlen(netCfg.httpPass) > 0) {
        httpAuth.setUsername(netCfg.httpUser[0] ? netCfg.httpUser : "admin");
        httpAuth.setPassword(netCfg.httpPass);
        httpAuth.setRealm("Watts Bridge");
        httpAuth.setAuthType(AsyncAuthType::AUTH_DIGEST);
        httpAuth.setAuthFailureMessage("Authentication required");
        httpAuth.generateHash();        // precompute so each request skips hashing
        server.addMiddleware(&httpAuth);
        Serial.println("HTTP: digest auth enabled");
    } else {
        Serial.println("HTTP: auth DISABLED (HTTP_PASS empty) -- UI served openly");
    }

    server.on("/", HTTP_GET, [](AsyncWebServerRequest *req) {
        req->send(200, "text/html", INDEX_HTML);
    });

    // Clear stored WiFi/MQTT config and reboot into the captive setup portal.
    server.on("/reset-wifi", HTTP_GET, [](AsyncWebServerRequest *req) {
        clearNetConfig();
        Serial.println("RESET: net config cleared -- rebooting into portal");
        req->send(200, "text/html",
                  "<!doctype html><meta charset=utf-8>"
                  "<body style=\"font:15px system-ui;padding:1.2rem\">"
                  "Network config cleared. Rebooting into the setup portal "
                  "(join WiFi <b>watts-bridge-…</b>).</body>");
        portalRebootAt = millis() + 1500;   // deferred restart, handled in loop()
    });

#if DEBUG_HTTP
    server.on("/status", HTTP_GET, [](AsyncWebServerRequest *req) {
        JsonDocument doc;
        doc["ip"]   = WiFi.localIP().toString();
        doc["rssi"] = WiFi.RSSI();
        doc["manchester_one_is_10"] = (int)MANCHESTER_ONE_IS_10;
        doc["rx_active"]   = rxActive;
        doc["pair_armed"]  = pairArmed;
        doc["pair_captured"] = pairCaptured;
        if (pairCaptured) {
            char id[7];
            snprintf(id, sizeof(id), "%02X%02X%02X", pairId[0], pairId[1], pairId[2]);
            doc["pair_id"] = id;
        }
        String body;
        serializeJson(doc, body);
        req->send(200, "application/json", body);
    });

    // RX raw capture on/off (Step 1 listening test).
    server.on("/rx-on", HTTP_GET, [](AsyncWebServerRequest *req) {
        rxEnable();
        req->send(200, "text/plain", "rx on -- watch serial for pulse bursts");
    });
    server.on("/rx-off", HTTP_GET, [](AsyncWebServerRequest *req) {
        rxDisable();
        req->send(200, "text/plain", "rx off");
    });
#endif  // DEBUG_HTTP

    // Off-frame pairing capture. Arm, then switch a thermostat off: the first
    // CRC-valid setpoint-0.0 frame latches its device ID. Auto-enables RX.
    server.on("/pair-listen", HTTP_GET, [](AsyncWebServerRequest *req) {
        pairStartedRx = !rxActive;   // remember if we had to enable RX ourselves
        if (!rxActive) rxEnable();
        pairArmed    = true;
        pairCaptured = false;
        pairId[0] = pairId[1] = pairId[2] = 0;
        pairAmb      = NAN;
        Serial.println("PAIR: armed -- switch a thermostat OFF to capture its ID");
        req->send(200, "text/plain",
                  "pairing armed -- switch a thermostat OFF, then poll /pair-status");
    });
    server.on("/pair-status", HTTP_GET, [](AsyncWebServerRequest *req) {
        JsonDocument doc;
        doc["armed"]    = pairArmed;
        doc["captured"] = pairCaptured;
        if (pairCaptured) {
            char id[7];
            snprintf(id, sizeof(id), "%02X%02X%02X", pairId[0], pairId[1], pairId[2]);
            doc["id"]  = id;
            doc["amb"] = pairAmb;
        }
        String body;
        serializeJson(doc, body);
        req->send(200, "application/json", body);
    });
    server.on("/pair-cancel", HTTP_GET, [](AsyncWebServerRequest *req) {
        pairArmed = false;
        Serial.println("PAIR: cancelled");
        req->send(200, "text/plain", "pairing cancelled");
    });

    // Zone bindings: tie a discovered Z2M thermostat (friendly_name) to the Watts
    // device ID the scheduler should spoof for it. Persisted in NVS on change.
    server.on("/bind", HTTP_GET, [](AsyncWebServerRequest *req) {
        if (!req->hasParam("name") || !req->hasParam("id")) {
            req->send(400, "text/plain", "usage: /bind?name=<z2m-name>&id=AABBCC");
            return;
        }
        String  name = req->getParam("name")->value();
        uint8_t id[3];
        if (!parseHexId(req->getParam("id")->value(), id)) {
            req->send(400, "text/plain", "id must be 6 hex digits, e.g. 349E48");
            return;
        }
        ZoneBinding *b = findBinding(name.c_str());
        if (!b)                                         // allocate a free slot
            for (int i = 0; i < MAX_THERMOSTATS; i++)
                if (!bindings[i].used) { b = &bindings[i]; break; }
        if (!b) { req->send(507, "text/plain", "binding table full"); return; }

        b->used = true;
        strlcpy(b->name, name.c_str(), sizeof(b->name));
        memcpy(b->devId, id, 3);
        b->lastTxAt = 0;
        b->dirty    = true;                             // transmit asap
        b->stale    = false;
        saveBindings();                                 // persist to NVS
        startAdvertise();                               // paced (re)advertise so the
                                                        // new zone's HA entities land
                                                        // without overrunning the buffer
        char resp[96];
        snprintf(resp, sizeof(resp), "bound \"%s\" -> %02X%02X%02X",
                 b->name, id[0], id[1], id[2]);
        Serial.println(resp);
        req->send(200, "text/plain", resp);
    });
    server.on("/unbind", HTTP_GET, [](AsyncWebServerRequest *req) {
        if (!req->hasParam("name")) {
            req->send(400, "text/plain", "usage: /unbind?name=<z2m-name>");
            return;
        }
        ZoneBinding *b = findBinding(req->getParam("name")->value().c_str());
        if (!b) { req->send(404, "text/plain", "no such binding"); return; }
        clearZoneDiscovery(b);                          // remove the HA entities
        b->used = false;
        saveBindings();                                 // persist to NVS
        Serial.printf("unbound \"%s\"\n", b->name);
        req->send(200, "text/plain", "unbound");
    });
    server.on("/bindings", HTTP_GET, [](AsyncWebServerRequest *req) {
        JsonDocument doc;
        JsonArray    arr = doc.to<JsonArray>();
        uint32_t     now = millis();
        for (int i = 0; i < MAX_THERMOSTATS; i++) {
            if (!bindings[i].used) continue;
            JsonObject o = arr.add<JsonObject>();
            o["name"] = bindings[i].name;
            char id[7];
            snprintf(id, sizeof(id), "%02X%02X%02X",
                     bindings[i].devId[0], bindings[i].devId[1], bindings[i].devId[2]);
            o["id"]            = id;
            o["last_tx_age_s"] = bindings[i].lastTxAt
                                 ? (int)((now - bindings[i].lastTxAt) / 1000) : -1;
            o["stale"]         = bindings[i].stale;
            // Fold in the live source values if the thermostat is discovered.
            for (int j = 0; j < thermostatCount; j++) {
                if (strcmp(thermostats[j].name, bindings[i].name) != 0) continue;
                if (!isnan(thermostats[j].lastTemp)) o["temp"] = thermostats[j].lastTemp;
                if (!isnan(thermostats[j].lastSp))   o["setpoint"] = thermostats[j].lastSp;
                if (thermostats[j].lastMode[0])      o["mode"] = thermostats[j].lastMode;
                break;
            }
        }
        String body;
        serializeJson(doc, body);
        req->send(200, "application/json", body);
    });

    // Discovered Z2M thermostats (the inventory the web UI's dropdown reads).
    // Distinct from /bindings, which lists only zones already tied to a Watts ID.
    server.on("/thermostats", HTTP_GET, [](AsyncWebServerRequest *req) {
        JsonDocument doc;
        JsonArray    arr = doc.to<JsonArray>();
        for (int i = 0; i < thermostatCount; i++) {
            JsonObject o = arr.add<JsonObject>();
            o["name"]  = thermostats[i].name;
            o["bound"] = findBinding(thermostats[i].name) != nullptr;
        }
        String body;
        serializeJson(doc, body);
        req->send(200, "application/json", body);
    });

#if DEBUG_HTTP
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

        sendBurst(DEV_ID, amb, sp, cfh, mode, 3, 400);   // A-B-A

        char resp[96];
        snprintf(resp, sizeof(resp),
                 "sent ABA: amb=%.1f sp=%.1f cfh=0x%02x mode=0x%02x",
                 amb, sp, cfh, mode);
        Serial.println(resp);
        req->send(200, "text/plain", resp);
    });
#endif  // DEBUG_HTTP

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

        // Which device ID the receiver should learn. Defaults to the firmware
        // DEV_ID; the web UI passes a bound zone's ID so the receiver associates
        // that manifold valve with the bridge's spoof of that ID.
        uint8_t id[3] = {DEV_ID[0], DEV_ID[1], DEV_ID[2]};
        if (req->hasParam("id") &&
            !parseHexId(req->getParam("id")->value(), id)) {
            req->send(400, "text/plain", "id must be 6 hex digits, e.g. 349E48");
            return;
        }

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
        memcpy(pairJob.devId, id, 3);

        char resp[112];
        snprintf(resp, sizeof(resp),
                 "pairing started: %d frames over %ds, id=%02X%02X%02X, mode=0x%02x",
                 frames, duration, id[0], id[1], id[2], pair_mode);
        Serial.println(resp);
        req->send(200, "text/plain", resp);
    });
    // Progress poll for the web UI's pairing button. The pairing stream runs in
    // loop(); the page polls this to show a countdown and to re-enable the button.
    server.on("/tx-pair-status", HTTP_GET, [](AsyncWebServerRequest *req) {
        JsonDocument doc;
        doc["active"] = pairJob.active;
        doc["sent"]   = pairJob.sent;
        doc["total"]  = pairJob.total;
        int left = pairJob.active && pairJob.total > pairJob.sent
                       ? (pairJob.total - pairJob.sent) / 2 : 0;
        doc["remaining_s"] = left;
        char id[7];
        snprintf(id, sizeof(id), "%02X%02X%02X",
                 pairJob.devId[0], pairJob.devId[1], pairJob.devId[2]);
        doc["id"] = id;
        String body;
        serializeJson(doc, body);
        req->send(200, "application/json", body);
    });

    server.onNotFound([](AsyncWebServerRequest *req) {
        req->send(404, "text/plain", "not found");
    });
}

void setup() {
    Serial.begin(115200);

    // Latch why we last rebooted before anything else can mask it; reported in
    // the diagnostics blob (distinguishes a clean OTA from a brownout/watchdog).
    resetReasonStr = resetReasonName(esp_reset_reason());

    loadNetConfig();

    // No SSID provisioned yet -> straight to the captive portal.
    if (netCfg.wifiSsid[0] == '\0') {
        Serial.println("WiFi: no SSID configured -- starting setup portal");
        startPortal();
        return;
    }

    WiFi.mode(WIFI_STA);
    WiFi.begin(netCfg.wifiSsid, netCfg.wifiPass);
    Serial.printf("WiFi connecting to \"%s\"", netCfg.wifiSsid);
    uint32_t startedAt = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - startedAt < 20000) {
        delay(250);
        Serial.print(".");
    }
    if (WiFi.status() != WL_CONNECTED) {
        // Bad creds or AP gone -> fall back to the portal so it can be re-provisioned.
        Serial.println("\nWiFi: connect timed out -- starting setup portal");
        startPortal();
        return;
    }
    Serial.printf("\nIP: %s\n", WiFi.localIP().toString().c_str());

    loadBindings();   // restore zone bindings from NVS before MQTT/discovery
    initCC1101();
    initMqtt();
    setupRoutes();
    server.begin();
    Serial.println("HTTP server started");
}

void loop() {
    // Deferred restart (portal save / /reset-wifi): fire once the HTTP reply has
    // had time to flush. Checked in both portal and normal modes.
    if (portalRebootAt != 0 && (int32_t)(millis() - portalRebootAt) >= 0)
        ESP.restart();

    // Captive-portal mode: just service DNS so probes resolve to us; none of the
    // MQTT / radio / scheduler machinery runs until we're provisioned and rebooted.
    if (portalActive) {
        dnsServer.processNextRequest();
        return;
    }

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

    // Drain the paced retained-config (re)advertise: one message per tick, only
    // advancing when the broker accepted it, so the connect-time burst can't
    // overrun the send buffer and silently drop a zone's HA entities.
    serviceAdvertise();

    // RX decode: drain timed edges into a per-burst chip stream, then on the
    // long idle gap that delimits a frame, Manchester-decode + CRC-check it. A
    // genuine frame is ~100-300 pulses of ~420/840 us; a >=1500 us gap delimits
    // it; bursts under 40 pulses are noise. `level` is reconstructed by parity:
    // the delimiting gap is carrier-low, so the next segment is carrier-high.
    if (rxActive) {
        static uint16_t acc   = 0;        // pulses in the current burst
        static uint8_t  level = 1;        // first in-burst segment is high
        while (pulseTail != pulseHead) {
            uint16_t w = pulseBuf[pulseTail];
            pulseTail = (uint16_t)((pulseTail + 1) % PULSE_BUF);
            if (w >= 1500) {              // inter-frame / idle gap -> flush
                // A frame ending in a '1' bit is Manchester "10": a high chip
                // then a low chip. That trailing low chip has no edge after it --
                // it merges into this idle gap and is never timed, so the frame's
                // last bit (crc16 LSB) is lost. Append low chips unconditionally
                // to restore it. This is parity-independent on purpose: a single
                // spurious/missed pulse mid-burst flips the global chip parity, so
                // an "append only when odd" rule fires at the wrong time. Padding
                // is safe regardless -- the frame is anchored at the sync, so any
                // extra trailing chips are ignored on a frame that was complete.
                for (int i = 0; i < 2 && rxChipN < sizeof(rxChips); i++)
                    rxChips[rxChipN++] = 0;
                DecodedFrame f;
                RxDiag diag;
                if (acc >= 40 && decodeBurst(rxChips, rxChipN, &f, &diag)) {
                    Serial.printf("RX frame: id=%02X%02X%02X mode=0x%02X amb=%.1f "
                                  "sp=%.1f cfh=0x%02X\n",
                                  f.id[0], f.id[1], f.id[2], f.mode,
                                  f.amb, f.sp, f.cfh);

                    // Off-frame pairing capture: a setpoint of exactly 0.0 means
                    // the thermostat was switched off (its A-frame; the B-frame
                    // twin carries 0.1). Latch the first one while armed.
                    if (pairArmed && !pairCaptured && f.sp == 0.0f) {
                        memcpy(pairId, f.id, 3);
                        pairAmb      = f.amb;
                        pairCaptured = true;
                        pairArmed    = false;
                        Serial.printf("PAIR: captured off-frame -> id=%02X%02X%02X "
                                      "(amb=%.1f)\n",
                                      pairId[0], pairId[1], pairId[2], pairAmb);
                        // Stop listening once we have the ID, but only if pairing
                        // is what turned RX on -- leave a manual /rx-on session be.
                        if (pairStartedRx) { rxDisable(); pairStartedRx = false; }
                    }
                }
#if RX_DEBUG
                else if (acc >= 40) {
                    if (diag.sawSync) {
                        // Synced but failed CRC -- print the bytes so we can see
                        // how this frame differs from the ones that validate.
                        Serial.printf("RX burst: %u pulses, sync ok CRC fail; bytes:", acc);
                        for (int i = 0; i < 16; i++) Serial.printf(" %02X", diag.fr[i]);
                        Serial.printf("  crc8 calc=%02X rx=%02X  crc16 calc=%04X rx=%04X\n",
                                      diag.crc8_calc, diag.crc8_rx,
                                      diag.crc16_calc, diag.crc16_rx);
                    } else {
                        Serial.printf("RX burst: %u pulses, no sync\n", acc);
                    }
                }
#endif
                rxChipN = 0; acc = 0; level = 1;
            } else {
                uint8_t n = (w < 630) ? 1 : 2;   // ~420 us = 1 chip, ~840 us = 2
                while (n-- && rxChipN < sizeof(rxChips)) rxChips[rxChipN++] = level;
                level ^= 1;
                acc++;
            }
        }
    }

    // Non-blocking pairing stream: one A/B frame every gap_ms, alternating
    // sp (A) / sp+0.1 (B), until `total` frames have gone out.
    if (pairJob.active && (int32_t)(millis() - pairJob.next_at) >= 0) {
        bool isB = pairJob.sent & 1;
        sendFrameOnce(pairJob.devId, pairJob.amb, isB ? pairJob.sp + 0.1f : pairJob.sp,
                      0x00, pairJob.mode);
        pairJob.sent++;
        if (pairJob.sent >= pairJob.total) {
            pairJob.active = false;
            Serial.printf("pairing done: %d frames sent\n", pairJob.sent);
        } else {
            pairJob.next_at = millis() + pairJob.gap_ms;
        }
    }

    // Advance each zone's call-for-heat P-loop, then relay each bound zone's
    // ambient + setpoint (+ the computed cfh flag) to the Watts receiver on the
    // 154 s heartbeat, or immediately on a state change (incl. a cfh flip).
    serviceControlLoops();
    serviceScheduler();

    // Bridge health heartbeat: republish the retained diagnostics blob so HA's
    // RSSI / uptime / last-tx-age sensors stay fresh. Connect seeds the first one.
    if (mqtt.connected() &&
        (int32_t)(millis() - lastDiagAt) >= (int32_t)DIAG_PERIOD_MS) {
        lastDiagAt = millis();
        publishDiag();
        publishAllZoneStates();   // keep each zone's last_tx_age fresh
    }
}