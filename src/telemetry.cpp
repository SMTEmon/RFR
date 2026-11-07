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
#include <esp_system.h>

RuntimeConfig g_cfg;
volatile bool g_armed = false; //safe disarmed by default on boot
volatile int16_t g_manualLeft = 0;
volatile int16_t g_manualRight = 0;
volatile uint32_t g_lastManualCmdMs = 0;

//reset tracking across warm reboots
RTC_NOINIT_ATTR static uint32_t s_bootCount;
RTC_NOINIT_ATTR static uint32_t s_brownoutCount;
RTC_NOINIT_ATTR static uint32_t s_magicMarker;

static esp_reset_reason_t s_lastResetReason = ESP_RST_UNKNOWN;
static bool s_isAbnormalReset = false;

static Preferences prefs;
static uint32_t lastTelemTime = 0;
static uint32_t lastCfgSyncTime = 0;
static uint32_t lastTuneStatusTime = 0;
static uint32_t lastSerialTelemTime = 0;
static uint32_t lastSerialStatusTime = 0;
static bool peerAdded = false;

static portMUX_TYPE cmdMux = portMUX_INITIALIZER_UNLOCKED;
static CommandPacket cmdMailbox = {};
static volatile bool cmdPending = false;

static char s_serialBuf[256];
static uint8_t s_serialLen = 0;

static void telemetrySendNowUSB();
static void telemetrySendConfigUSB(uint8_t ackEvent);
static void parseSerialCommand(const char *line);

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
        uint8_t cfgVer = prefs.getUChar("ver", 1);
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
        if (cfgVer < 2 && g_cfg.accelSlew > 3) {
            g_cfg.accelSlew = DEFAULT_ACCEL_SLEW;
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

    prefs.putUChar("ver", 2);
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
    portENTER_CRITICAL(&cmdMux);
    cmdMailbox = *cmd;
    cmdPending = true;
    portEXIT_CRITICAL(&cmdMux);
}

static void parseSerialCommand(const char *line) {
    const char *cmdPtr = strstr(line, "\"cmd\":");
    if (!cmdPtr) return;
    uint8_t cmdType = static_cast<uint8_t>(atoi(cmdPtr + 6));

    CommandPacket pkt = {};
    pkt.header = 0x55;
    pkt.cmdType = cmdType;
    pkt.baseSpeed = g_cfg.baseSpeed;
    pkt.minSpeed = g_cfg.minSpeed;
    pkt.maxSpeed = g_cfg.maxSpeed;
    pkt.turnSpeed = g_cfg.turnSpeed;
    pkt.threshEnter = g_cfg.threshEnter;
    pkt.pidKp = g_cfg.pidKp;
    pkt.pidKd = g_cfg.pidKd;
    pkt.encKp = g_cfg.encKp;
    pkt.runMode = g_cfg.runMode;
    pkt.enableInverted = g_cfg.enableInverted ? 1 : 0;
    pkt.skipStartExit = g_cfg.skipStartExit ? 1 : 0;
    memcpy(pkt.sensorSens, g_cfg.sensorSens, sizeof(pkt.sensorSens));
    pkt.accelSlew = g_cfg.accelSlew;
    pkt.junctionMode = g_cfg.junctionMode;
    pkt.allowUturn = g_cfg.allowUturn ? 1 : 0;
    pkt.manualLeft = 0;
    pkt.manualRight = 0;

    auto extractInt = [&](const char *key, int defaultVal) -> int {
        const char *p = strstr(line, key);
        if (!p) return defaultVal;
        return atoi(p + strlen(key));
    };
    auto extractFloat = [&](const char *key, float defaultVal) -> float {
        const char *p = strstr(line, key);
        if (!p) return defaultVal;
        return static_cast<float>(atof(p + strlen(key)));
    };

    if (cmdType == CMD_SET_PARAMS) {
        pkt.baseSpeed = extractInt("\"base\":", pkt.baseSpeed);
        pkt.minSpeed = extractInt("\"min\":", pkt.minSpeed);
        pkt.maxSpeed = extractInt("\"max\":", pkt.maxSpeed);
        pkt.turnSpeed = extractInt("\"turn\":", pkt.turnSpeed);
        pkt.threshEnter = extractInt("\"thr\":", pkt.threshEnter);
        pkt.pidKp = extractFloat("\"kp\":", pkt.pidKp);
        pkt.pidKd = extractFloat("\"kd\":", pkt.pidKd);
        pkt.encKp = extractFloat("\"enc\":", pkt.encKp);
        pkt.runMode = extractInt("\"mode\":", pkt.runMode);
        pkt.enableInverted = extractInt("\"inv\":", pkt.enableInverted);
        pkt.skipStartExit = extractInt("\"skip\":", pkt.skipStartExit);
        pkt.accelSlew = extractInt("\"slew\":", pkt.accelSlew);
        pkt.junctionMode = extractInt("\"junc\":", pkt.junctionMode);
        pkt.allowUturn = extractInt("\"uturn\":", pkt.allowUturn);

        const char *sensPtr = strstr(line, "\"sens\":[");
        if (sensPtr) {
            const char *p = sensPtr + 8;
            for (uint8_t i = 0; i < NUM_SENSORS && *p; i++) {
                pkt.sensorSens[i] = static_cast<uint8_t>(atoi(p));
                const char *comma = strchr(p, ',');
                const char *close = strchr(p, ']');
                if (comma && (!close || comma < close)) p = comma + 1;
                else break;
            }
        }
    } else if (cmdType == CMD_MANUAL_DRIVE) {
        pkt.manualLeft = extractInt("\"l\":", 0);
        pkt.manualRight = extractInt("\"r\":", 0);
    } else if (cmdType == CMD_MAP_START_CRAWL) {
        pkt.enableInverted = extractInt("\"fig8\":", 0);
    }

    portENTER_CRITICAL(&cmdMux);
    cmdMailbox = pkt;
    cmdPending = true;
    portEXIT_CRITICAL(&cmdMux);
}

static void telemetryProcessSerial() {
    while (Serial.available()) {
        char c = static_cast<char>(Serial.read());
        if (c == '\n' || c == '\r') {
            if (s_serialLen > 0) {
                s_serialBuf[s_serialLen] = '\0';
                parseSerialCommand(s_serialBuf);
                s_serialLen = 0;
            }
        } else if (s_serialLen < sizeof(s_serialBuf) - 1) {
            s_serialBuf[s_serialLen++] = c;
        }
    }
}

//drain and execute mailbox commands on core 1
void telemetryProcessCommands() {
    telemetryProcessSerial();
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

    //evaluate hardware reset reason
    if (s_magicMarker != 0xBEEFCAFE) {
        s_magicMarker = 0xBEEFCAFE;
        s_bootCount = 0;
        s_brownoutCount = 0;
    }
    s_bootCount++;
    s_lastResetReason = esp_reset_reason();
    if (s_lastResetReason == ESP_RST_BROWNOUT) {
        s_brownoutCount++;
        s_isAbnormalReset = true;
    } else if (s_lastResetReason == ESP_RST_PANIC || s_lastResetReason == ESP_RST_INT_WDT || s_lastResetReason == ESP_RST_TASK_WDT || s_lastResetReason == ESP_RST_WDT) {
        s_isAbnormalReset = true;
    }

    //let power rail settle after power-up before enabling rf
    delay(150);

    WiFi.persistent(false);
    WiFi.setSleep(false);
    WiFi.mode(WIFI_STA);
    WiFi.setTxPower(WIFI_POWER_8_5dBm); //cap tx power to 8.5dBm to avoid supply sag
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

    //broadcast initial config on boot with boot diagnostic ack
    uint8_t bootAck = ACK_NONE;
    if (s_lastResetReason == ESP_RST_BROWNOUT) {
        bootAck = ACK_BOOT_BROWNOUT;
    } else if (s_isAbnormalReset) {
        bootAck = ACK_BOOT_PANIC;
    }
    telemetrySendConfig(bootAck);
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
    if (s_isAbnormalReset)                flags |= 0x80;
    pkt.modeFlags = flags;

    pkt.checksum = calcProtocolChecksum(reinterpret_cast<const uint8_t*>(&pkt), sizeof(pkt) - 1);

    esp_now_send(DONGLE_MAC, reinterpret_cast<const uint8_t*>(&pkt), sizeof(pkt));
}

static void telemetrySendNowUSB() {
    if (Serial.availableForWrite() < 240) return;

    uint16_t mask = 0;
    uint16_t rawAdc[NUM_SENSORS];
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        rawAdc[i] = getSensorRaw(i);
        if (isSensorBlack(i)) {
            mask |= (1 << i);
        }
    }

    uint8_t flags = 0;
    if (g_cfg.runMode == MODE_BENCH_DIAG) flags |= 0x01;
    if (g_cfg.runMode == MODE_MOTOR_TEST) flags |= 0x02;
    if (isNavigatorInverted())            flags |= 0x04;
    if (g_armed)                          flags |= 0x08;
    if (hasLeftWing())                    flags |= 0x10;
    if (hasRightWing())                   flags |= 0x20;
    if (isSensorCalibrating())            flags |= 0x40;
    if (s_isAbnormalReset)                flags |= 0x80;

    Serial.print("{\"t\":\"telem\",\"adc\":[");
    for (uint8_t i = 0; i < 16; i++) {
        Serial.print(rawAdc[i]);
        if (i < 15) Serial.print(",");
    }
    Serial.printf("],\"mask\":%u,\"err\":%ld,\"l\":%ld,\"r\":%ld,\"lpwm\":%d,\"rpwm\":%d,\"st\":\"%s\",\"mode\":%u,\"flags\":%u}\n",
                  mask,
                  (long)computePositionError(isNavigatorInverted()),
                  (long)getLeftTicks(),
                  (long)getRightTicks(),
                  getLastLeftPwm(),
                  getLastRightPwm(),
                  navStateToStr(isSensorCalibrating() ? STATE_CALIBRATING : getNavState()),
                  g_cfg.runMode,
                  flags);
}

static void telemetrySendConfigUSB(uint8_t ackEvent) {
    if (Serial.availableForWrite() < 240) return;
    Serial.printf("{\"t\":\"cfg\",\"base\":%d,\"min\":%d,\"max\":%d,\"turn\":%d,\"thr\":%d,\"kp\":%.3f,\"kd\":%.3f,\"enc\":%.1f,\"mode\":%u,\"inv\":%u,\"skip\":%u,\"slew\":%u,\"junc\":%u,\"uturn\":%u,\"ack\":%u,\"sens\":[",
                  g_cfg.baseSpeed,
                  g_cfg.minSpeed,
                  g_cfg.maxSpeed,
                  g_cfg.turnSpeed,
                  g_cfg.threshEnter,
                  g_cfg.pidKp,
                  g_cfg.pidKd,
                  g_cfg.encKp,
                  g_cfg.runMode,
                  g_cfg.enableInverted ? 1 : 0,
                  g_cfg.skipStartExit ? 1 : 0,
                  g_cfg.accelSlew,
                  g_cfg.junctionMode,
                  g_cfg.allowUturn ? 1 : 0,
                  ackEvent);
    for (uint8_t i = 0; i < 16; i++) {
        Serial.print(g_cfg.sensorSens[i]);
        if (i < 15) Serial.print(",");
    }
    Serial.println("]}");
}

void telemetrySendConfig(uint8_t ackEvent) {
    telemetrySendConfigUSB(ackEvent);

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

    //send 5hz json telemetry over usb serial for direct dashboard connection
    if (now - lastSerialTelemTime >= 200) {
        lastSerialTelemTime = now;
        telemetrySendNowUSB();
    }

    //print 1hz readable status line if serial monitor is connected
    if (now - lastSerialStatusTime >= 1000) {
        lastSerialStatusTime = now;
        if (Serial.availableForWrite() >= 120) {
            Serial.printf("[LFR] rst=%d (bo=%u) | st=%-14s err=%5ld pwm=(%4d,%4d) enc=(%5ld,%5ld) armed=%d\n",
                          (int)s_lastResetReason, (unsigned)s_brownoutCount,
                          getNavStateName(), (long)computePositionError(isNavigatorInverted()),
                          getLastLeftPwm(), getLastRightPwm(),
                          (long)getLeftTicks(), (long)getRightTicks(), g_armed ? 1 : 0);
        }
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
