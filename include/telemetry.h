#pragma once
#include <Arduino.h>
#include "protocol.h"

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
    uint8_t runMode;        //RunMode enum (0=auto, 1=bench, 2=mtest, 3=wasd)
    bool    enableInverted; //true when track has white-on-black zone
    bool    skipStartExit;  //true to follow line immediately without 12cm blind drive
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
void loadConfigFromNVS();
void saveConfigToNVS();
