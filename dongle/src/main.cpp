#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include "protocol.h"

//hardcoded lfr robot mac address (b4:bf:e9:0f:13:c0)
static const uint8_t DEFAULT_LFR_MAC[6] = {0xB4, 0xBF, 0xE9, 0x0F, 0x13, 0xC0};
static uint8_t lfrMac[6] = {0xB4, 0xBF, 0xE9, 0x0F, 0x13, 0xC0};
static bool lfrKnown = true;
static String serialRxBuf = "";

//state name lookup
static const char* getStateName(uint8_t st) {
    switch (st) {
        case STATE_START_EXIT:      return "START_EXIT";
        case STATE_NORMAL_PID:     return "NORMAL_PID";
        case STATE_INVERTED_PID:   return "INVERTED_PID";
        case STATE_SHARP_TURN:     return "SHARP_TURN";
        case STATE_GAP_OR_DEADEND: return "GAP_OR_DEADEND";
        case STATE_UTURN_180:      return "UTURN_180";
        case STATE_ALL_BLACK_CHECK:return "ALL_BLACK_CHECK";
        case STATE_STOPPED:        return "STOPPED";
        case STATE_CALIBRATING:    return "CALIBRATING";
        default:                   return "UNKNOWN";
    }
}

//register peer cleanly if not already added
static void registerLfrPeer(const uint8_t *mac) {
    memcpy(lfrMac, mac, 6);
    lfrKnown = true;

    if (!esp_now_is_peer_exist(mac)) {
        esp_now_peer_info_t peer = {};
        memcpy(peer.peer_addr, mac, 6);
        peer.channel = 1;
        peer.encrypt = false;
        esp_now_add_peer(&peer);
    }
}

//esp-now rx callback
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
static void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
    const uint8_t *srcMac = info->src_addr;
#else
static void onDataRecv(const uint8_t *mac, const uint8_t *data, int len) {
    const uint8_t *srcMac = mac;
#endif
    registerLfrPeer(srcMac);

    if (len == sizeof(TelemetryPacket)) {
        const TelemetryPacket *telem = reinterpret_cast<const TelemetryPacket*>(data);
        if (telem->header != 0xAA) return;

        uint8_t expected = calcProtocolChecksum(data, sizeof(TelemetryPacket) - 1);
        if (telem->checksum != expected) return;

        //forward directly as compact json line over usb serial
        Serial.print("{\"t\":\"telem\",\"adc\":[");
        for (uint8_t i = 0; i < 16; i++) {
            Serial.print(telem->rawAdc[i]);
            if (i < 15) Serial.print(",");
        }
        Serial.printf("],\"mask\":%u,\"err\":%ld,\"l\":%ld,\"r\":%ld,\"lpwm\":%d,\"rpwm\":%d,\"st\":\"%s\",\"mode\":%u,\"flags\":%u}\n",
                      telem->sensorBlackMask,
                      (long)telem->positionError,
                      (long)telem->leftTicks,
                      (long)telem->rightTicks,
                      telem->leftPwm,
                      telem->rightPwm,
                      getStateName(telem->navState),
                      telem->runMode,
                      telem->modeFlags);
        return;
    }

    if (len == sizeof(ConfigSyncPacket)) {
        const ConfigSyncPacket *cfg = reinterpret_cast<const ConfigSyncPacket*>(data);
        if (cfg->header != 0xCC) return;

        uint8_t expected = calcProtocolChecksum(data, sizeof(ConfigSyncPacket) - 1);
        if (cfg->checksum != expected) return;

        Serial.printf("{\"t\":\"cfg\",\"base\":%d,\"min\":%d,\"max\":%d,\"turn\":%d,\"thr\":%d,\"kp\":%.3f,\"kd\":%.3f,\"enc\":%.1f,\"mode\":%u,\"inv\":%u,\"skip\":%u,\"ack\":%u}\n",
                      cfg->baseSpeed,
                      cfg->minSpeed,
                      cfg->maxSpeed,
                      cfg->turnSpeed,
                      cfg->threshEnter,
                      cfg->pidKp,
                      cfg->pidKd,
                      cfg->encKp,
                      cfg->runMode,
                      cfg->enableInverted,
                      cfg->skipStartExit,
                      cfg->ackEvent);
        return;
    }
}

static void sendCommand(const CommandPacket &cmd) {
    if (!lfrKnown) {
        //broadcast if lfr mac not learned yet
        uint8_t bcast[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
        if (!esp_now_is_peer_exist(bcast)) {
            esp_now_peer_info_t peer = {};
            memcpy(peer.peer_addr, bcast, 6);
            peer.channel = 1;
            peer.encrypt = false;
            esp_now_add_peer(&peer);
        }
        esp_now_send(bcast, reinterpret_cast<const uint8_t*>(&cmd), sizeof(cmd));
        return;
    }

    esp_now_send(lfrMac, reinterpret_cast<const uint8_t*>(&cmd), sizeof(cmd));
}

//parse incoming command line from webserial
static void handleSerialCommand(const String &line) {
    int cmdIdx = line.indexOf("\"cmd\":");
    if (cmdIdx == -1) return;

    uint8_t cmdType = line.substring(cmdIdx + 6).toInt();

    CommandPacket pkt = {};
    pkt.header  = 0x55;
    pkt.cmdType = cmdType;

    //defaults if not provided
    pkt.baseSpeed      = 140;
    pkt.minSpeed       = 80;
    pkt.maxSpeed       = 220;
    pkt.turnSpeed      = 120;
    pkt.threshEnter    = 500;
    pkt.pidKp          = 0.045f;
    pkt.pidKd          = 0.280f;
    pkt.encKp          = 2.0f;
    pkt.runMode        = 0;
    pkt.enableInverted = 0;
    pkt.skipStartExit  = 1;
    pkt.manualLeft     = 0;
    pkt.manualRight    = 0;

    auto extractInt = [&](const char *key, int defaultVal) -> int {
        int idx = line.indexOf(key);
        if (idx == -1) return defaultVal;
        return line.substring(idx + strlen(key)).toInt();
    };

    auto extractFloat = [&](const char *key, float defaultVal) -> float {
        int idx = line.indexOf(key);
        if (idx == -1) return defaultVal;
        return line.substring(idx + strlen(key)).toFloat();
    };

    if (cmdType == CMD_SET_PARAMS) {
        pkt.baseSpeed      = extractInt("\"base\":", 140);
        pkt.minSpeed       = extractInt("\"min\":",  80);
        pkt.maxSpeed       = extractInt("\"max\":",  220);
        pkt.turnSpeed      = extractInt("\"turn\":", 120);
        pkt.threshEnter    = extractInt("\"thr\":",  500);
        pkt.pidKp          = extractFloat("\"kp\":", 0.045f);
        pkt.pidKd          = extractFloat("\"kd\":", 0.280f);
        pkt.encKp          = extractFloat("\"enc\":", 2.0f);
        pkt.runMode        = extractInt("\"mode\":", 0);
        pkt.enableInverted = extractInt("\"inv\":",  0);
        pkt.skipStartExit  = extractInt("\"skip\":", 1);
    } else if (cmdType == CMD_MANUAL_DRIVE) {
        pkt.manualLeft     = extractInt("\"l\":", 0);
        pkt.manualRight    = extractInt("\"r\":", 0);
    }

    pkt.checksum = calcProtocolChecksum(reinterpret_cast<const uint8_t*>(&pkt), sizeof(pkt) - 1);
    sendCommand(pkt);

    //local ack back to webserial
    Serial.printf("{\"t\":\"ack\",\"cmd\":%u}\n", cmdType);
}

void setup() {
    Serial.begin(115200);

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    //lock wifi channel to 1
    esp_wifi_set_promiscuous(true);
    esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
    esp_wifi_set_promiscuous(false);

    if (esp_now_init() != ESP_OK) {
        Serial.println("{\"t\":\"err\",\"msg\":\"esp-now init failed\"}");
        return;
    }

    esp_now_register_recv_cb(onDataRecv);

    //pre-register lfr robot peer (b4:bf:e9:0f:13:c0)
    registerLfrPeer(DEFAULT_LFR_MAC);

    Serial.println("{\"t\":\"ready\",\"dongle\":\"LFR USB-ESP Bridge Ready (Peer: b4:bf:e9:0f:13:c0)\"}");
}

void loop() {
    //read incoming serial lines from webserial
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\n' || c == '\r') {
            if (serialRxBuf.length() > 0) {
                handleSerialCommand(serialRxBuf);
                serialRxBuf = "";
            }
        } else {
            if (serialRxBuf.length() < 512) {
                serialRxBuf += c;
            }
        }
    }
}
