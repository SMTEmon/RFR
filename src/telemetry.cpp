#include "telemetry.h"
#include "config.h"
#include "sensors.h"
#include "encoders.h"
#include "motors.h"
#include "navigator.h"
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Preferences.h>

RuntimeConfig g_cfg;
volatile bool g_emergencyStop = false;
static Preferences prefs;
static uint32_t lastTelemTime = 0;
static bool peerAdded = false;

//default fallback values from config.h
static void loadDefaultConfig() {
    g_cfg.baseSpeed       = BASE_SPEED;
    g_cfg.minSpeed        = MIN_SPEED;
    g_cfg.maxSpeed        = MAX_SPEED;
    g_cfg.turnSpeed       = TURN_SPEED;
    g_cfg.pidKp           = PID_KP;
    g_cfg.pidKd           = PID_KD;
    g_cfg.encKp           = ENC_KP;
    g_cfg.benchDebugMode  = BENCH_DEBUG_MODE;
    g_cfg.motorTestMode   = MOTOR_TEST_MODE;
    g_cfg.enableInverted  = ENABLE_INVERTED_ZONE;
}

void loadConfigFromNVS() {
    loadDefaultConfig();

    if (!prefs.begin("lfr_cfg", true)) { //read-only mode
        return;
    }

    if (prefs.isKey("saved")) {
        g_cfg.baseSpeed       = prefs.getShort("base", g_cfg.baseSpeed);
        g_cfg.minSpeed        = prefs.getShort("min",  g_cfg.minSpeed);
        g_cfg.maxSpeed        = prefs.getShort("max",  g_cfg.maxSpeed);
        g_cfg.turnSpeed       = prefs.getShort("turn", g_cfg.turnSpeed);
        g_cfg.pidKp           = prefs.getFloat("kp",   g_cfg.pidKp);
        g_cfg.pidKd           = prefs.getFloat("kd",   g_cfg.pidKd);
        g_cfg.encKp           = prefs.getFloat("enc",  g_cfg.encKp);
        g_cfg.benchDebugMode  = prefs.getBool("bench", g_cfg.benchDebugMode);
        g_cfg.motorTestMode   = prefs.getBool("mtest", g_cfg.motorTestMode);
        g_cfg.enableInverted  = prefs.getBool("inv",   g_cfg.enableInverted);
        Serial.println("//loaded custom tunings from nvs");
    }
    prefs.end();
}

void saveConfigToNVS() {
    if (!prefs.begin("lfr_cfg", false)) { //read-write mode
        return;
    }

    prefs.putShort("base", g_cfg.baseSpeed);
    prefs.putShort("min",  g_cfg.minSpeed);
    prefs.putShort("max",  g_cfg.maxSpeed);
    prefs.putShort("turn", g_cfg.turnSpeed);
    prefs.putFloat("kp",   g_cfg.pidKp);
    prefs.putFloat("kd",   g_cfg.pidKd);
    prefs.putFloat("enc",  g_cfg.encKp);
    prefs.putBool("bench", g_cfg.benchDebugMode);
    prefs.putBool("mtest", g_cfg.motorTestMode);
    prefs.putBool("inv",   g_cfg.enableInverted);
    prefs.putBool("saved", true);
    prefs.end();
    Serial.println("//saved current tunings to nvs flash");
}

static uint8_t calcChecksum(const uint8_t *data, size_t len) {
    uint8_t c = 0;
    for (size_t i = 0; i < len; i++) {
        c ^= data[i];
    }
    return c;
}

//esp-now receive callback (handles core v2 & v3 signatures)
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
static void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
#else
static void onDataRecv(const uint8_t *mac, const uint8_t *data, int len) {
#endif
    if (len != sizeof(CommandPacket)) return;

    const CommandPacket *cmd = reinterpret_cast<const CommandPacket*>(data);
    if (cmd->header != 0x55) return;

    //verify checksum
    uint8_t expected = calcChecksum(data, sizeof(CommandPacket) - 1);
    if (cmd->checksum != expected) return;

    switch (cmd->cmdType) {
        case 1: //emergency stop
            g_emergencyStop = true;
            shortBrake();
            motorsArm(false);
            break;

        case 2: //start / resume
            g_emergencyStop = false;
            motorsArm(true);
            break;

        case 3: //set params live
            g_cfg.baseSpeed      = cmd->baseSpeed;
            g_cfg.minSpeed       = cmd->minSpeed;
            g_cfg.maxSpeed       = cmd->maxSpeed;
            g_cfg.turnSpeed      = cmd->turnSpeed;
            g_cfg.pidKp          = cmd->pidKp;
            g_cfg.pidKd          = cmd->pidKd;
            g_cfg.encKp          = cmd->encKp;
            g_cfg.benchDebugMode = (cmd->runMode == 1);
            g_cfg.motorTestMode  = (cmd->runMode == 2);
            g_cfg.enableInverted = (cmd->enableInverted != 0);
            break;

        case 4: //commit current params to nvs
            saveConfigToNVS();
            break;

        case 5: //reset wheel encoders
            resetEncoders();
            break;

        default:
            break;
    }
}

void telemetryInit() {
    loadConfigFromNVS();

    //setup wifi in station mode without connecting to any ap
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    //lock wifi channel to 1 for esp-now
    esp_wifi_set_promiscuous(true);
    esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
    esp_wifi_set_promiscuous(false);

    if (esp_now_init() != ESP_OK) {
        Serial.println("//esp-now init failed");
        return;
    }

    //register rx callback
    esp_now_register_recv_cb(onDataRecv);

    //add dongle as peer
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, DONGLE_MAC, 6);
    peerInfo.channel = 1;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) == ESP_OK) {
        peerAdded = true;
        Serial.println("//esp-now peer added: d0:cf:13:e5:c5:24");
    } else {
        Serial.println("//failed to add esp-now peer");
    }
}

void telemetrySendNow() {
    if (!peerAdded) return;

    TelemetryPacket pkt = {};
    pkt.header = 0xAA;

    uint16_t mask = 0;
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        pkt.rawAdc[i] = getSensorRaw(i);
        if (isSensorBlack(i)) {
            mask |= (1 << i);
        }
    }
    pkt.sensorBlackMask = mask;
    pkt.positionError   = computePositionError(false);
    pkt.leftTicks       = getLeftTicks();
    pkt.rightTicks      = getRightTicks();
    pkt.navState        = static_cast<uint8_t>(getNavState());

    uint8_t flags = 0;
    if (g_cfg.benchDebugMode)  flags |= 0x01;
    if (g_cfg.motorTestMode)   flags |= 0x02;
    if (g_cfg.enableInverted)  flags |= 0x04;
    if (g_emergencyStop)       flags |= 0x08;
    if (hasLeftWing())         flags |= 0x10;
    if (hasRightWing())        flags |= 0x20;
    pkt.modeFlags = flags;

    pkt.checksum = calcChecksum(reinterpret_cast<const uint8_t*>(&pkt), sizeof(pkt) - 1);

    esp_now_send(DONGLE_MAC, reinterpret_cast<const uint8_t*>(&pkt), sizeof(pkt));
}

void telemetryUpdate() {
    //transmit telemetry at 25hz (every 40ms)
    uint32_t now = millis();
    if (now - lastTelemTime >= 40) {
        lastTelemTime = now;
        telemetrySendNow();
    }
}
