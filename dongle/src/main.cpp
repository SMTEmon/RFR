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
static uint8_t lastSens[16] = {100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100};
static uint8_t lastSlew = 3;
static uint8_t lastJunc = 0;
static uint8_t lastUturn = 0;

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

        memcpy(lastSens, cfg->sensorSens, 16);
        lastSlew = cfg->accelSlew;
        lastJunc = cfg->junctionMode;
        lastUturn = cfg->allowUturn;

        Serial.printf("{\"t\":\"cfg\",\"base\":%d,\"min\":%d,\"max\":%d,\"turn\":%d,\"thr\":%d,\"kp\":%.3f,\"kd\":%.3f,\"enc\":%.1f,\"mode\":%u,\"inv\":%u,\"skip\":%u,\"slew\":%u,\"junc\":%u,\"uturn\":%u,\"ack\":%u,\"sens\":[",
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
                      cfg->accelSlew,
                      cfg->junctionMode,
                      cfg->allowUturn,
                      cfg->ackEvent);
        for (uint8_t i = 0; i < 16; i++) {
            Serial.print(cfg->sensorSens[i]);
            if (i < 15) Serial.print(",");
        }
        Serial.println("]}");
        return;
    }

    if (len == sizeof(MapPathPacket)) {
        const MapPathPacket *path = reinterpret_cast<const MapPathPacket*>(data);
        if (path->header != 0xB1) return;
        uint8_t expected = calcProtocolChecksum(data, sizeof(MapPathPacket) - 1);
        if (path->checksum != expected) return;

        Serial.printf("{\"t\":\"map_path\",\"len\":%u,\"wind\":%d,\"gaps\":%u,\"thr\":%d,\"slew\":%u,\"pts\":[",
                      path->totalLengthCm, path->windingSign, path->hasGaps, path->optThresh, path->optAccelSlew);
        for (uint8_t i = 0; i < path->numPoints; i++) {
            Serial.print(path->dThetaScaled[i]);
            if (i < path->numPoints - 1) Serial.print(",");
        }
        Serial.print("],\"sens\":[");
        for (uint8_t i = 0; i < 16; i++) {
            Serial.print(path->optSens[i]);
            if (i < 15) Serial.print(",");
        }
        Serial.println("]}");
        return;
    }

    if (len == sizeof(MapSegmentsPacket)) {
        const MapSegmentsPacket *segs = reinterpret_cast<const MapSegmentsPacket*>(data);
        if (segs->header != 0xB2) return;
        uint8_t expected = calcProtocolChecksum(data, sizeof(MapSegmentsPacket) - 1);
        if (segs->checksum != expected) return;

        Serial.printf("{\"t\":\"map_segs\",\"n\":%u,\"segs\":[", segs->numSegments);
        for (uint8_t i = 0; i < segs->numSegments; i++) {
            Serial.printf("{\"type\":%u,\"start\":%u,\"len\":%u,\"rad\":%d,\"pwm\":%u}",
                          segs->segs[i].type, segs->segs[i].startCm, segs->segs[i].lengthCm,
                          segs->segs[i].radiusCm, segs->segs[i].maxSafePwm);
            if (i < segs->numSegments - 1) Serial.print(",");
        }
        Serial.println("]}");
        return;
    }

    if (len == sizeof(TuneStatusPacket)) {
        const TuneStatusPacket *st = reinterpret_cast<const TuneStatusPacket*>(data);
        if (st->header != 0xB3) return;
        uint8_t expected = calcProtocolChecksum(data, sizeof(TuneStatusPacket) - 1);
        if (st->checksum != expected) return;

        Serial.printf("{\"t\":\"tune_st\",\"st\":%u,\"tier\":%u,\"n_tier\":%u,\"sub\":%u,\"lap\":%u,\"seg\":%u,\"match\":%u,\"dist\":%u,\"tot\":%u,\"hdg\":%d,\"spwm\":%d,\"cpwm\":%d,\"kp\":%.4f,\"kd\":%.3f,\"rms\":%u,\"max\":%u,\"score\":%u,\"lift\":%u,\"flags\":%u}\n",
                      st->tuneState, st->currentTier, st->totalTiers, st->subTrial, st->lapNumber,
                      st->currentSegIdx, st->overlapMatchPct, st->currentDistCm, st->totalLengthCm,
                      st->netHeadingDeg, st->straightPwm, st->cornerPwm, st->activeKp, st->activeKd,
                      st->lastRmsErr01Mm, st->lastMaxErr01Mm, st->lastQualityScore, st->lastLiftPeak, st->statusFlags);
        return;
    }

    if (len == sizeof(GainTablePacket)) {
        const GainTablePacket *tbl = reinterpret_cast<const GainTablePacket*>(data);
        if (tbl->header != 0xB4) return;
        uint8_t expected = calcProtocolChecksum(data, sizeof(GainTablePacket) - 1);
        if (tbl->checksum != expected) return;

        Serial.printf("{\"t\":\"gain_lut\",\"n\":%u,\"tiers\":[", tbl->numTiers);
        for (uint8_t i = 0; i < tbl->numTiers; i++) {
            Serial.printf("{\"spwm\":%d,\"cpwm\":%d,\"kp\":%.4f,\"kd\":%.3f,\"rms\":%u,\"score\":%u,\"st\":%u}",
                          tbl->tiers[i].straightPwm, tbl->tiers[i].cornerPwm,
                          tbl->tiers[i].kpX10000 * 0.0001f, tbl->tiers[i].kdX1000 * 0.001f,
                          tbl->tiers[i].rmsErr01Mm, tbl->tiers[i].qualityScore, tbl->tiers[i].state);
            if (i < tbl->numTiers - 1) Serial.print(",");
        }
        Serial.println("]}");
        return;
    }
}

static void sendCommand(const CommandPacket &cmd) {
    if (!lfrKnown) {
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
    memcpy(pkt.sensorSens, lastSens, 16);
    pkt.accelSlew      = lastSlew;
    pkt.junctionMode   = lastJunc;
    pkt.allowUturn     = lastUturn;
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
        pkt.accelSlew      = extractInt("\"slew\":", 3);
        pkt.junctionMode   = extractInt("\"junc\":", 0);
        pkt.allowUturn     = extractInt("\"uturn\":", 0);
        lastSlew = pkt.accelSlew;
        lastJunc = pkt.junctionMode;
        lastUturn = pkt.allowUturn;

        int sensIdx = line.indexOf("\"sens\":[");
        if (sensIdx != -1) {
            int start = sensIdx + 8;
            for (uint8_t i = 0; i < 16; i++) {
                int nextComma = line.indexOf(',', start);
                int nextClose = line.indexOf(']', start);
                int end = (nextComma != -1 && (nextClose == -1 || nextComma < nextClose)) ? nextComma : nextClose;
                if (end != -1) {
                    uint8_t val = line.substring(start, end).toInt();
                    pkt.sensorSens[i] = val;
                    lastSens[i] = val;
                    start = end + 1;
                }
            }
        }
    } else if (cmdType == CMD_MANUAL_DRIVE) {
        pkt.manualLeft     = extractInt("\"l\":", 0);
        pkt.manualRight    = extractInt("\"r\":", 0);
    } else if (cmdType == CMD_MAP_START_CRAWL) {
        pkt.enableInverted = extractInt("\"fig8\":", 0);
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
            if (serialRxBuf.length() < 1024) {
                serialRxBuf += c;
            }
        }
    }
}
