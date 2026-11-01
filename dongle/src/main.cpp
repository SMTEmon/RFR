#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#pragma pack(push, 1)
struct TelemetryPacket {
    uint8_t header;           //0xAA
    uint16_t rawAdc[16];      //32 bytes
    uint16_t sensorBlackMask; //2 bytes bitmask
    int32_t positionError;    //4 bytes
    int32_t leftTicks;        //4 bytes
    int32_t rightTicks;       //4 bytes
    uint8_t navState;         //1 byte
    uint8_t modeFlags;        //1 byte
    uint8_t checksum;
};

struct CommandPacket {
    uint8_t header;           //0x55
    uint8_t cmdType;          //1=STOP, 2=START, 3=SET_PARAMS, 4=SAVE_NVS, 5=RESET_ENC
    int16_t baseSpeed;
    int16_t minSpeed;
    int16_t maxSpeed;
    int16_t turnSpeed;
    float pidKp;
    float pidKd;
    float encKp;
    uint8_t runMode;          //0=NORMAL, 1=BENCH_DEBUG, 2=MOTOR_TEST
    uint8_t enableInverted;   //0=false, 1=true
    uint8_t checksum;
};
#pragma pack(pop)

static uint8_t lfrMac[6] = {0};
static bool lfrKnown = false;
static String serialRxBuf = "";

static uint8_t calcChecksum(const uint8_t *data, size_t len) {
    uint8_t c = 0;
    for (size_t i = 0; i < len; i++) {
        c ^= data[i];
    }
    return c;
}

//state name lookup
static const char* getStateName(uint8_t st) {
    switch (st) {
        case 0: return "START_EXIT";
        case 1: return "NORMAL_PID";
        case 2: return "INVERTED_PID";
        case 3: return "SHARP_TURN";
        case 4: return "GAP_OR_DEADEND";
        case 5: return "UTURN_180";
        case 6: return "ALL_BLACK_CHECK";
        case 7: return "STOPPED";
        default: return "UNKNOWN";
    }
}

//register sender peer dynamically
static void registerLfrPeer(const uint8_t *mac) {
    if (!lfrKnown || memcmp(lfrMac, mac, 6) != 0) {
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
}

//esp-now rx callback
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
static void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
    const uint8_t *srcMac = info->src_addr;
#else
static void onDataRecv(const uint8_t *mac, const uint8_t *data, int len) {
    const uint8_t *srcMac = mac;
#endif
    if (len != sizeof(TelemetryPacket)) return;

    const TelemetryPacket *telem = reinterpret_cast<const TelemetryPacket*>(data);
    if (telem->header != 0xAA) return;

    uint8_t expected = calcChecksum(data, sizeof(TelemetryPacket) - 1);
    if (telem->checksum != expected) return;

    registerLfrPeer(srcMac);

    //forward directly as compact json line over usb serial
    Serial.print("{\"t\":\"telem\",\"adc\":[");
    for (uint8_t i = 0; i < 16; i++) {
        Serial.print(telem->rawAdc[i]);
        if (i < 15) Serial.print(",");
    }
    Serial.printf("],\"mask\":%u,\"err\":%ld,\"l\":%ld,\"r\":%ld,\"st\":\"%s\",\"flags\":%u}\n",
                  telem->sensorBlackMask,
                  (long)telem->positionError,
                  (long)telem->leftTicks,
                  (long)telem->rightTicks,
                  getStateName(telem->navState),
                  telem->modeFlags);
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
    pkt.baseSpeed = 140;
    pkt.minSpeed  = 80;
    pkt.maxSpeed  = 220;
    pkt.turnSpeed = 120;
    pkt.pidKp     = 0.045f;
    pkt.pidKd     = 0.280f;
    pkt.encKp     = 2.0f;
    pkt.runMode   = 0;
    pkt.enableInverted = 0;

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

    if (cmdType == 3) { //set params
        pkt.baseSpeed      = extractInt("\"base\":", 140);
        pkt.minSpeed       = extractInt("\"min\":",  80);
        pkt.maxSpeed       = extractInt("\"max\":",  220);
        pkt.turnSpeed      = extractInt("\"turn\":", 120);
        pkt.pidKp          = extractFloat("\"kp\":", 0.045f);
        pkt.pidKd          = extractFloat("\"kd\":", 0.280f);
        pkt.encKp          = extractFloat("\"enc\":", 2.0f);
        pkt.runMode        = extractInt("\"mode\":", 0);
        pkt.enableInverted = extractInt("\"inv\":",  0);
    }

    pkt.checksum = calcChecksum(reinterpret_cast<const uint8_t*>(&pkt), sizeof(pkt) - 1);
    sendCommand(pkt);

    //ack back to webserial
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

    Serial.println("{\"t\":\"ready\",\"dongle\":\"LFR USB-ESP Bridge Ready\"}");
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
            serialRxBuf += c;
        }
    }
}
