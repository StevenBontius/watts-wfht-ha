#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <ELECHOUSE_CC1101_SRC_DRV.h>
#include <math.h>
#include "config.h"

static AsyncWebServer server(80);

// ---- CRC-8 (poly=0xE6, init=0x00, XorOut=0xBE) over bytes 8..19 ----
static uint8_t crc8_watts(const uint8_t *data, size_t len) {
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++)
            crc = (crc & 0x80) ? (crc << 1) ^ 0xE6 : (crc << 1);
    }
    return crc ^ 0xBE;
}

// Device ID cloned from captures (bytes 13-15)
static const uint8_t DEV_ID[3] = {0x34, 0x9E, 0x48};

// Build 13-byte payload (bytes 9..21 of the Watts frame).
// SendData() prepends the length byte (0x0D = 13), making byte 8.
// The CC1101 then appends CRC-16/CMS over bytes 8..21 automatically.
static void buildPayload(uint8_t *buf, float ambient_C, float setpoint_C, uint8_t cfh) {
    int16_t amb = (int16_t)roundf(ambient_C * 10.0f);
    int16_t sp  = (int16_t)roundf(setpoint_C * 10.0f);

    buf[0]  = 0xFF; buf[1] = 0xFF; buf[2] = 0xFE;    // protocol header
    buf[3]  = 0x02;                                    // mode: heat, normal
    buf[4]  = DEV_ID[0]; buf[5] = DEV_ID[1]; buf[6] = DEV_ID[2];
    buf[7]  = (amb >> 8) & 0xFF; buf[8]  = amb & 0xFF;
    buf[9]  = (sp  >> 8) & 0xFF; buf[10] = sp  & 0xFF;
    buf[11] = cfh;

    // CRC-8 covers [length_byte=0x0D, buf[0..10]] = bytes 8..19 (12 bytes)
    uint8_t crc_in[12];
    crc_in[0] = 0x0D;
    memcpy(crc_in + 1, buf, 11);
    buf[12] = crc8_watts(crc_in, 12) ^ cfh;    // final XOR with byte 20
}

static void printFrame(const char *label, const uint8_t *payload) {
    Serial.printf("%s: 0d", label);
    for (int i = 0; i < 13; i++) Serial.printf("%02x", payload[i]);
    Serial.println();
}

static void sendABA(float ambient_C, float setpoint_C, uint8_t cfh) {
    uint8_t frameA[13], frameB[13];
    buildPayload(frameA, ambient_C, setpoint_C,         cfh);
    buildPayload(frameB, ambient_C, setpoint_C + 0.1f,  cfh);   // B: SP+0.1°C

    printFrame("A", frameA);
    printFrame("B", frameB);

    ELECHOUSE_cc1101.SendData(frameA, 13);
    delay(400);
    ELECHOUSE_cc1101.SendData(frameB, 13);
    delay(400);
    ELECHOUSE_cc1101.SendData(frameA, 13);
    ELECHOUSE_cc1101.SetRx();
}

static void dumpCC1101() {
    struct { uint8_t addr; const char *name; } regs[] = {
        {CC1101_IOCFG0,   "IOCFG0  "},
        {CC1101_MDMCFG4,  "MDMCFG4 "},
        {CC1101_MDMCFG3,  "MDMCFG3 "},
        {CC1101_MDMCFG2,  "MDMCFG2 "},
        {CC1101_MDMCFG1,  "MDMCFG1 "},
        {CC1101_SYNC1,    "SYNC1   "},
        {CC1101_SYNC0,    "SYNC0   "},
        {CC1101_PKTCTRL0, "PKTCTRL0"},
        {CC1101_PKTLEN,   "PKTLEN  "},
        {CC1101_FREND0,   "FREND0  "},
    };
    Serial.println("CC1101 registers:");
    for (auto &r : regs)
        Serial.printf("  %s = 0x%02X\n", r.name,
                      ELECHOUSE_cc1101.SpiReadReg(r.addr));
}

static bool initCC1101() {
    ELECHOUSE_cc1101.setSpiPin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS);
    ELECHOUSE_cc1101.Init();
    if (!ELECHOUSE_cc1101.getCC1101()) {
        Serial.println("CC1101: SPI connection failed");
        return false;
    }

    ELECHOUSE_cc1101.setMHZ(433.92);
    ELECHOUSE_cc1101.setModulation(2);  // OOK

    // GDO0 = assert-on-sync-word (0x06): lets SendData() detect TX start/end correctly.
    // Default ccmode=false leaves GDO0 as carrier-sense (0x0D), which breaks the
    // GDO0 wait in SendData() and can cause SFTX to fire mid-transmission.
    ELECHOUSE_cc1101.SpiWriteReg(CC1101_IOCFG0, 0x06);
    ELECHOUSE_cc1101.setGDO0(PIN_GDO0);

    // Chip rate: 2170 chips/s  (DRATE_E=6, DRATE_M=94 → 2170 chips/s)
    // Hardware Manchester: each data bit = 2 chips → 1085 bps effective
    uint8_t m4 = ELECHOUSE_cc1101.SpiReadReg(CC1101_MDMCFG4) & 0xF0;
    ELECHOUSE_cc1101.SpiWriteReg(CC1101_MDMCFG4, m4 | 0x06);
    ELECHOUSE_cc1101.SpiWriteReg(CC1101_MDMCFG3, 0x5E);

    // 8 preamble bytes — gives the Manchester demodulator enough run-up to lock
    ELECHOUSE_cc1101.SpiWriteReg(CC1101_MDMCFG1, 0x42);

    // Manchester encoding + 30/32 sync (transmits sync word twice: D391D391)
    uint8_t m2 = ELECHOUSE_cc1101.SpiReadReg(CC1101_MDMCFG2) & 0xF0;
    ELECHOUSE_cc1101.SpiWriteReg(CC1101_MDMCFG2, m2 | 0x0B);

    // Sync word 0xD391
    ELECHOUSE_cc1101.SpiWriteReg(CC1101_SYNC1, 0xD3);
    ELECHOUSE_cc1101.SpiWriteReg(CC1101_SYNC0, 0x91);

    // Variable-length packet mode, hardware CRC-16/CMS enabled, no whitening
    ELECHOUSE_cc1101.SpiWriteReg(CC1101_PKTCTRL0, 0x05);

    // Max variable-length packet = 61 bytes (TX FIFO size); overrides library's 0x00
    ELECHOUSE_cc1101.SpiWriteReg(CC1101_PKTLEN, 0x3D);

    dumpCC1101();
    Serial.println("CC1101: ready");
    return true;
}

static void setupRoutes() {
    server.on("/status", HTTP_GET, [](AsyncWebServerRequest *req) {
        JsonDocument doc;
        doc["ip"]   = WiFi.localIP().toString();
        doc["rssi"] = WiFi.RSSI();
        String body;
        serializeJson(doc, body);
        req->send(200, "application/json", body);
    });

    // Quick radio smoke-test: raw OOK burst
    server.on("/tx-test", HTTP_GET, [](AsyncWebServerRequest *req) {
        byte buf[8] = {0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA};
        ELECHOUSE_cc1101.SetTx();
        ELECHOUSE_cc1101.SendData(buf, sizeof(buf));
        ELECHOUSE_cc1101.SetRx();
        Serial.println("TX test sent");
        req->send(200, "text/plain", "tx sent");
    });

    // Send a Watts A-B-A burst.  Query params: amb, sp (°C), cfh (0 or 100)
    server.on("/tx-watts", HTTP_GET, [](AsyncWebServerRequest *req) {
        float amb = 22.5f, sp = 24.0f;
        uint8_t cfh = 0x00;
        if (req->hasParam("amb")) amb = req->getParam("amb")->value().toFloat();
        if (req->hasParam("sp"))  sp  = req->getParam("sp")->value().toFloat();
        if (req->hasParam("cfh")) cfh = (uint8_t)req->getParam("cfh")->value().toInt();

        sendABA(amb, sp, cfh);

        char resp[80];
        snprintf(resp, sizeof(resp), "sent ABA: amb=%.1f sp=%.1f cfh=0x%02x", amb, sp, cfh);
        Serial.println(resp);
        req->send(200, "text/plain", resp);
    });

    server.onNotFound([](AsyncWebServerRequest *req) {
        req->send(404, "text/plain", "not found");
    });
}

void setup() {
    Serial.begin(115200);

    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.print("WiFi connecting");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.printf("\nIP: %s\n", WiFi.localIP().toString().c_str());

    initCC1101();
    setupRoutes();
    server.begin();
    Serial.println("HTTP server started");
}

void loop() {
    // main control loop goes here
}
