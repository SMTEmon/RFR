#pragma once
#include <Arduino.h>

//esp-now peer mac address for the dongle (d0:cf:13:e5:c5:24)
constexpr uint8_t DONGLE_MAC[6] = {0xD0, 0xCF, 0x13, 0xE5, 0xC5, 0x24};

//dynamic runtime parameters that can be tuned live and saved to nvs
struct RuntimeConfig {
    int16_t baseSpeed;
    int16_t minSpeed;
    int16_t maxSpeed;
    int16_t turnSpeed;
    float pidKp;
    float pidKd;
    float encKp;
    bool benchDebugMode;
    bool motorTestMode;
    bool enableInverted;
};

//telemetry packet sent from lfr to dongle (~25hz)
#pragma pack(push, 1)
struct TelemetryPacket {
    uint8_t header;           //0xAA
    uint16_t rawAdc[16];      //32 bytes
    uint16_t sensorBlackMask; //2 bytes bitmask (bit0=S00 ... bit15=S15)
    int32_t positionError;    //4 bytes lateral pid error in 0.01mm
    int32_t leftTicks;        //4 bytes
    int32_t rightTicks;       //4 bytes
    uint8_t navState;         //1 byte state enum
    uint8_t modeFlags;        //1 byte (bit0=debug, bit1=motor_test, bit2=inverted_active)
    uint8_t checksum;         //simple xor checksum
};

//command packet sent from dongle to lfr
struct CommandPacket {
    uint8_t header;           //0x55
    uint8_t cmdType;          //0=NOP, 1=STOP, 2=START, 3=SET_PARAMS, 4=SAVE_NVS, 5=RESET_ENC
    int16_t baseSpeed;
    int16_t minSpeed;
    int16_t maxSpeed;
    int16_t turnSpeed;
    float pidKp;
    float pidKd;
    float encKp;
    uint8_t runMode;          //0=NORMAL_RUN, 1=BENCH_DEBUG, 2=MOTOR_TEST
    uint8_t enableInverted;   //0=false, 1=true
    uint8_t checksum;
};
#pragma pack(pop)

//global active configuration instance
extern RuntimeConfig g_cfg;
extern volatile bool g_emergencyStop;

//functions
void telemetryInit();
void telemetryUpdate();
void telemetrySendNow();
void loadConfigFromNVS();
void saveConfigToNVS();
