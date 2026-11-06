#include "telemetry.h"
#include "config.h"
#include "sensors.h"
#include "encoders.h"
#include "motors.h"
#include "navigator.h"
#include "autotune.h"
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Preferences.h>

RuntimeConfig g_cfg;
volatile bool g_armed = false; //safe disarmed by default on boot
volatile int16_t g_manualLeft = 0;
volatile int16_t g_manualRight = 0;
volatile uint32_t g_lastManualCmdMs = 0;

static Preferences prefs;
static uint32_t lastTelemTime = 0;
static uint32_t lastCfgSyncTime = 0;
static uint32_t lastTuneStatusTime = 0;
static bool peerAdded = false;

static portMUX_TYPE cmdMux = portMUX_INITIALIZER_UNLOCKED;
static CommandPacket cmdMailbox = {};
static volatile bool cmdPending = false;

//default fallback values from config.h
static void loadDefaultConfig() {
    g_cfg.baseSpeed       = BASE_SPEED;
    g_cfg.minSpeed        = MIN_SPEED;
    g_cfg.maxSpeed        = MAX_SPEED;
    g_cfg.turnSpeed       = TURN_SPEED;
    g_cfg.threshEnter     = THRESH_BLACK_ENTER;
    g_cfg.pidKp           = PID_KP;
    g_cfg.pidKd           = PID_KD;
    g_cfg.encKp           = ENC_KP;
    g_cfg.runMode         = BENCH_DEBUG_MODE ? MODE_BENCH_DIAG : (MOTOR_TEST_MODE ? MODE_MOTOR_TEST : MODE_AUTONOMOUS);
    g_cfg.enableInverted  = ENABLE_INVERTED_ZONE;
    g_cfg.skipStartExit   = DEFAULT_SKIP_START_EXIT;
    g_cfg.accelSlew       = DEFAULT_ACCEL_SLEW;
    g_cfg.junctionMode    = DEFAULT_JUNCTION_MODE;
    g_cfg.allowUturn      = DEFAULT_ALLOW_UTURN;
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        g_cfg.sensorSens[i] = DEFAULT_SENSOR_SENS[i];
    }
}

void loadConfigFromNVS() {
    loadDefaultConfig();

    if (!prefs.begin("lfr_cfg", true)) { //read-only mode
        setSensorThreshold(g_cfg.threshEnter);
        setAllSensorSensitivities(g_cfg.sensorSens);
        setMotorAccelSlew(g_cfg.accelSlew);
        return;
    }

    if (prefs.isKey("saved")) {
        g_cfg.baseSpeed       = prefs.getShort("base", g_cfg.baseSpeed);
        g_cfg.minSpeed        = prefs.getShort("min",  g_cfg.minSpeed);
        g_cfg.maxSpeed        = prefs.getShort("max",  g_cfg.maxSpeed);
        g_cfg.turnSpeed       = prefs.getShort("turn", g_cfg.turnSpeed);
        g_cfg.threshEnter     = prefs.getShort("thr",  g_cfg.threshEnter);
        g_cfg.pidKp           = prefs.getFloat("kp",   g_cfg.pidKp);
        g_cfg.pidKd           = prefs.getFloat("kd",   g_cfg.pidKd);
        g_cfg.encKp           = prefs.getFloat("enc",  g_cfg.encKp);
        g_cfg.runMode         = prefs.getUChar("mode", g_cfg.runMode);
        g_cfg.enableInverted  = prefs.getBool("inv",   g_cfg.enableInverted);
        g_cfg.skipStartExit   = prefs.getBool("skip",  g_cfg.skipStartExit);
        g_cfg.accelSlew       = prefs.getUChar("slew", g_cfg.accelSlew);
        g_cfg.junctionMode    = prefs.getUChar("junc", g_cfg.junctionMode);
        g_cfg.allowUturn      = prefs.getBool("uturn", g_cfg.allowUturn);
        if (prefs.isKey("sens")) {
            prefs.getBytes("sens", g_cfg.sensorSens, sizeof(g_cfg.sensorSens));
        }
        Serial.println("//loaded custom tunings from nvs");
    }
    prefs.end();

    setSensorThreshold(g_cfg.threshEnter);
    setAllSensorSensitivities(g_cfg.sensorSens);
    setMotorAccelSlew(g_cfg.accelSlew);
}

void saveConfigToNVS() {
    if (!prefs.begin("lfr_cfg", false)) { //read-write mode
        return;
    }

    prefs.putShort("base", g_cfg.baseSpeed);
    prefs.putShort("min",  g_cfg.minSpeed);
    prefs.putShort("max",  g_cfg.maxSpeed);
    prefs.putShort("turn", g_cfg.turnSpeed);
    prefs.putShort("thr",  g_cfg.threshEnter);
    prefs.putFloat("kp",   g_cfg.pidKp);
    prefs.putFloat("kd",   g_cfg.pidKd);
    prefs.putFloat("enc",  g_cfg.encKp);
    prefs.putUChar("mode", g_cfg.runMode);
    prefs.putBool("inv",   g_cfg.enableInverted);
    prefs.putBool("skip",  g_cfg.skipStartExit);
    prefs.putUChar("slew", g_cfg.accelSlew);
    prefs.putUChar("junc", g_cfg.junctionMode);
    prefs.putBool("uturn", g_cfg.allowUturn);
    prefs.putBytes("sens", g_cfg.sensorSens, sizeof(g_cfg.sensorSens));
    prefs.putBool("saved", true);
    prefs.end();
    Serial.println("//saved current tunings to nvs flash");
}

//esp-now receive callback (runs in core 0 wifi isr context)
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
static void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
#else
static void onDataRecv(const uint8_t *mac, const uint8_t *data, int len) {
#endif
    if (len != sizeof(CommandPacket)) return;

    const CommandPacket *cmd = reinterpret_cast<const CommandPacket*>(data);
    if (cmd->header != 0x55) return;

    //verify checksum
    uint8_t expected = calcProtocolChecksum(data, sizeof(CommandPacket) - 1);
    if (cmd->checksum != expected) return;

    //store into mailbox atomically
    portENTER_CRITICAL_ISR(&cmdMux);
    cmdMailbox = *cmd;
    cmdPending = true;
    portEXIT_CRITICAL_ISR(&cmdMux);
}

//drain and execute mailbox commands on core 1
void telemetryProcessCommands() {
    if (!cmdPending) return;

    CommandPacket cmd;
    portENTER_CRITICAL(&cmdMux);
    cmd = cmdMailbox;
    cmdPending = false;
    portEXIT_CRITICAL(&cmdMux);

    switch (cmd.cmdType) {
        case CMD_STOP:
            g_armed = false;
            break;

        case CMD_START:
            g_armed = true;
            if (g_cfg.runMode == MODE_AUTONOMOUS) {
                navigatorInit(g_cfg.skipStartExit);
            }
            break;

        case CMD_SET_PARAMS: {
            uint8_t prevMode = g_cfg.runMode;
            g_cfg.baseSpeed      = cmd.baseSpeed;
            g_cfg.minSpeed       = cmd.minSpeed;
            g_cfg.maxSpeed       = cmd.maxSpeed;
            g_cfg.turnSpeed      = cmd.turnSpeed;
            g_cfg.threshEnter    = cmd.threshEnter;
            g_cfg.pidKp          = cmd.pidKp;
            g_cfg.pidKd          = cmd.pidKd;
            g_cfg.encKp          = cmd.encKp;
            g_cfg.runMode        = cmd.runMode;
            g_cfg.enableInverted = (cmd.enableInverted != 0);
            g_cfg.skipStartExit  = (cmd.skipStartExit != 0);
            g_cfg.accelSlew       = cmd.accelSlew;
            g_cfg.junctionMode    = cmd.junctionMode;
            g_cfg.allowUturn      = (cmd.allowUturn != 0);
            memcpy(g_cfg.sensorSens, cmd.sensorSens, sizeof(g_cfg.sensorSens));

            //disarm on mode switch for safety
            if (prevMode != g_cfg.runMode) {
                g_armed = false;
            }

            setSensorThreshold(g_cfg.threshEnter);
            setAllSensorSensitivities(g_cfg.sensorSens);
            setMotorAccelSlew(g_cfg.accelSlew);
            telemetrySendConfig(ACK_PARAMS_OK);
            break;
        }

        case CMD_SAVE_NVS:
            saveConfigToNVS();
            telemetrySendConfig(ACK_NVS_SAVED);
            break;

        case CMD_RESET_ENC:
            resetEncoders();
            telemetrySendConfig(ACK_ENC_RESET);
            break;

        case CMD_MANUAL_DRIVE:
            g_manualLeft = cmd.manualLeft;
            g_manualRight = cmd.manualRight;
            g_lastManualCmdMs = millis();
            break;

        case CMD_SYNC_CFG:
            telemetrySendConfig(ACK_NONE);
            break;

        case CMD_CALIB_START:
            g_armed = false;
            sensorsStartCalibration();
            break;

        case CMD_CALIB_DEFAULT:
            sensorsResetCalibrationDefault();
            telemetrySendConfig(ACK_CALIB_DEFAULT);
            break;

        case CMD_MAP_START_CRAWL:
            g_cfg.runMode = MODE_MAP_AND_TUNE;
            g_armed = true;
            autotuneStartCrawl(cmd.enableInverted != 0);
            break;

        case CMD_MAP_MARK_LAP:
            autotuneMarkLapNow();
            break;

        case CMD_TUNE_VALIDATE:
            g_armed = true;
            autotuneValidateAndStartSweep();
            break;

        case CMD_TUNE_SAVE_ALL:
            autotuneSaveToNVS();
            break;

        case CMD_MAP_REQUEST:
            autotuneRequestSync();
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

    esp_now_register_recv_cb(onDataRecv);

    //add dongle as peer
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, DONGLE_MAC, 6);
    peerInfo.channel = 1;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) == ESP_OK) {
        peerAdded = true;
        Serial.println("//esp-now peer added: 38:3e:51:58:16:70");
    } else {
        Serial.println("//failed to add esp-now peer");
    }

    //broadcast initial config on boot
    telemetrySendConfig(ACK_NONE);
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
    pkt.positionError   = computePositionError(isNavigatorInverted());
    pkt.leftTicks       = getLeftTicks();
    pkt.rightTicks      = getRightTicks();
    pkt.leftPwm         = getLastLeftPwm();
    pkt.rightPwm        = getLastRightPwm();
    pkt.navState        = static_cast<uint8_t>(isSensorCalibrating() ? STATE_CALIBRATING : getNavState());
    pkt.runMode         = g_cfg.runMode;

    uint8_t flags = 0;
    if (g_cfg.runMode == MODE_BENCH_DIAG) flags |= 0x01;
    if (g_cfg.runMode == MODE_MOTOR_TEST) flags |= 0x02;
    if (isNavigatorInverted())            flags |= 0x04;
    if (g_armed)                          flags |= 0x08;
    if (hasLeftWing())                    flags |= 0x10;
    if (hasRightWing())                   flags |= 0x20;
    if (isSensorCalibrating())            flags |= 0x40;
    pkt.modeFlags = flags;

    pkt.checksum = calcProtocolChecksum(reinterpret_cast<const uint8_t*>(&pkt), sizeof(pkt) - 1);

    esp_now_send(DONGLE_MAC, reinterpret_cast<const uint8_t*>(&pkt), sizeof(pkt));
}

void telemetrySendConfig(uint8_t ackEvent) {
    if (!peerAdded) return;

    ConfigSyncPacket pkt = {};
    pkt.header         = 0xCC;
    pkt.baseSpeed      = g_cfg.baseSpeed;
    pkt.minSpeed       = g_cfg.minSpeed;
    pkt.maxSpeed       = g_cfg.maxSpeed;
    pkt.turnSpeed      = g_cfg.turnSpeed;
    pkt.threshEnter    = g_cfg.threshEnter;
    pkt.pidKp          = g_cfg.pidKp;
    pkt.pidKd          = g_cfg.pidKd;
    pkt.encKp          = g_cfg.encKp;
    pkt.runMode        = g_cfg.runMode;
    pkt.enableInverted = g_cfg.enableInverted ? 1 : 0;
    pkt.skipStartExit  = g_cfg.skipStartExit ? 1 : 0;
    memcpy(pkt.sensorSens, g_cfg.sensorSens, sizeof(pkt.sensorSens));
    pkt.accelSlew      = g_cfg.accelSlew;
    pkt.junctionMode   = g_cfg.junctionMode;
    pkt.allowUturn     = g_cfg.allowUturn ? 1 : 0;
    pkt.ackEvent       = ackEvent;

    pkt.checksum = calcProtocolChecksum(reinterpret_cast<const uint8_t*>(&pkt), sizeof(pkt) - 1);

    esp_now_send(DONGLE_MAC, reinterpret_cast<const uint8_t*>(&pkt), sizeof(pkt));
}

void telemetryBroadcastMapPath() {
    if (!peerAdded) return;
    const MapPathPacket &pkt = getMapPath();
    esp_now_send(DONGLE_MAC, reinterpret_cast<const uint8_t*>(&pkt), sizeof(pkt));
}

void telemetryBroadcastMapSegments() {
    if (!peerAdded) return;
    const MapSegmentsPacket &pkt = getMapSegments();
    esp_now_send(DONGLE_MAC, reinterpret_cast<const uint8_t*>(&pkt), sizeof(pkt));
}

void telemetryBroadcastTuneStatus() {
    if (!peerAdded) return;
    const TuneStatusPacket &pkt = getTuneStatus();
    esp_now_send(DONGLE_MAC, reinterpret_cast<const uint8_t*>(&pkt), sizeof(pkt));
}

void telemetryBroadcastGainTable() {
    if (!peerAdded) return;
    const GainTablePacket &pkt = getGainTable();
    esp_now_send(DONGLE_MAC, reinterpret_cast<const uint8_t*>(&pkt), sizeof(pkt));
}

void telemetryUpdate() {
    uint32_t now = millis();

    //transmit telemetry at 25hz (every 40ms)
    if (now - lastTelemTime >= 40) {
        lastTelemTime = now;
        telemetrySendNow();
    }

    //heartbeat config sync every 1500ms
    if (now - lastCfgSyncTime >= 1500) {
        lastCfgSyncTime = now;
        telemetrySendConfig(ACK_NONE);
    }

    //tune status broadcast at 4hz during mapping / tuning / racing
    if (g_cfg.runMode == MODE_MAP_AND_TUNE || g_cfg.runMode == MODE_MAPPED_RACE) {
        if (now - lastTuneStatusTime >= 250) {
            lastTuneStatusTime = now;
            telemetryBroadcastTuneStatus();
        }
    }
}
