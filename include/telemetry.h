#pragma once
#include <Arduino.h>
#include "protocol.h"
#include "config.h"

//esp-now peer mac address for the dongle (38:3e:51:58:16:70)
constexpr uint8_t DONGLE_MAC[6] = {0x38, 0x3E, 0x51, 0x58, 0x16, 0x70};

//dynamic runtime parameters that can be tuned live and saved to nvs
struct RuntimeConfig {
    int16_t baseSpeed;
    int16_t minSpeed;
    int16_t maxSpeed;
    int16_t turnSpeed;
    int16_t threshEnter;
    float   pidKp;
    float   pidKd;
    float   encKp;
    uint8_t runMode;                 //RunMode enum
    bool    enableInverted;          //true when track has white-on-black zone
    bool    skipStartExit;           //true to follow line immediately without 12cm blind drive
    uint8_t sensorSens[NUM_SENSORS]; //per-sensor sensitivity percentage 0..180
    uint8_t accelSlew;               //acceleration slew limiter (1..25 pwm/tick)
    uint8_t junctionMode;            //0=loop straight, 1=lhr, 2=rhr
    bool    allowUturn;              //true to allow 180 u-turn on line loss, false to stop
};

//global active configuration and state instances
extern RuntimeConfig g_cfg;
extern volatile bool g_armed;
extern volatile int16_t g_manualLeft;
extern volatile int16_t g_manualRight;
extern volatile uint32_t g_lastManualCmdMs;

//functions
void telemetryInit();
void telemetryUpdate();
void telemetryProcessCommands();
void telemetrySendNow();
void telemetrySendConfig(uint8_t ackEvent = ACK_NONE);
void telemetryBroadcastMapPath();
void telemetryBroadcastMapSegments();
void telemetryBroadcastTuneStatus();
void telemetryBroadcastGainTable();
void loadConfigFromNVS();
void saveConfigToNVS();
