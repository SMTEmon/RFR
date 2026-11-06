#pragma once
#include <Arduino.h>

//shared communication protocol between lfr robot and dongle
//all structs packed 1-byte aligned to match exactly across esp32 targets

enum RunMode : uint8_t {
    MODE_AUTONOMOUS = 0,
    MODE_BENCH_DIAG = 1,
    MODE_MOTOR_TEST = 2,
    MODE_MANUAL_WASD = 3
};

enum CmdType : uint8_t {
    CMD_NOP           = 0,
    CMD_STOP          = 1,
    CMD_START         = 2,
    CMD_SET_PARAMS    = 3,
    CMD_SAVE_NVS      = 4,
    CMD_RESET_ENC     = 5,
    CMD_MANUAL_DRIVE  = 6,
    CMD_SYNC_CFG      = 7,
    CMD_CALIB_START   = 8,
    CMD_CALIB_DEFAULT = 9
};

enum AckEvent : uint8_t {
    ACK_NONE          = 0,
    ACK_PARAMS_OK     = 1,
    ACK_NVS_SAVED     = 2,
    ACK_CALIB_DONE    = 3,
    ACK_CALIB_FALLBACK= 4,
    ACK_CALIB_DEFAULT = 5,
    ACK_ENC_RESET     = 6
};

enum NavState : uint8_t {
    STATE_START_EXIT      = 0,
    STATE_NORMAL_PID      = 1,
    STATE_INVERTED_PID    = 2,
    STATE_SHARP_TURN      = 3,
    STATE_GAP_OR_DEADEND  = 4,
    STATE_UTURN_180       = 5,
    STATE_ALL_BLACK_CHECK = 6,
    STATE_STOPPED         = 7,
    STATE_CALIBRATING     = 8
};

#pragma pack(push, 1)

//25hz telemetry packet sent from lfr to dongle
struct TelemetryPacket {
    uint8_t  header;          //0xAA
    uint16_t rawAdc[16];     //32 bytes
    uint16_t sensorBlackMask;//2 bytes bitmask (bit0=S00 ... bit15=S15)
    int32_t  positionError;  //4 bytes lateral pid error in 0.01mm
    int32_t  leftTicks;      //4 bytes
    int32_t  rightTicks;     //4 bytes
    int16_t  leftPwm;        //2 bytes (-255..255)
    int16_t  rightPwm;       //2 bytes (-255..255)
    uint8_t  navState;       //1 byte NavState enum
    uint8_t  runMode;        //1 byte RunMode enum
    uint8_t  modeFlags;      //1 byte bitmask
                             //bit0: bench_diag_active
                             //bit1: motor_test_active
                             //bit2: inverted_zone_active
                             //bit3: armed (motors enabled)
                             //bit4: left_wing_detected
                             //bit5: right_wing_detected
                             //bit6: calibrating_active
    uint8_t  checksum;       //1 byte xor
};
static_assert(sizeof(TelemetryPacket) == 55, "TelemetryPacket size mismatch");

//config sync packet sent from lfr on connect, on save, and as 1.5s heartbeat
struct ConfigSyncPacket {
    uint8_t header;          //0xCC
    int16_t baseSpeed;       //2 bytes
    int16_t minSpeed;        //2 bytes
    int16_t maxSpeed;        //2 bytes
    int16_t turnSpeed;       //2 bytes
    int16_t threshEnter;     //2 bytes
    float   pidKp;           //4 bytes
    float   pidKd;           //4 bytes
    float   encKp;           //4 bytes
    uint8_t runMode;         //1 byte RunMode enum
    uint8_t enableInverted;  //1 byte (0/1)
    uint8_t skipStartExit;   //1 byte (0/1)
    uint8_t sensorSens[16];  //16 bytes: 0=off, 20..180% sensitivity
    uint8_t ackEvent;        //1 byte AckEvent enum
    uint8_t checksum;        //1 byte xor
};
static_assert(sizeof(ConfigSyncPacket) == 44, "ConfigSyncPacket size mismatch");

//command packet sent from dongle to lfr
struct CommandPacket {
    uint8_t header;          //0x55
    uint8_t cmdType;         //1 byte CmdType enum
    int16_t baseSpeed;       //2 bytes
    int16_t minSpeed;        //2 bytes
    int16_t maxSpeed;        //2 bytes
    int16_t turnSpeed;       //2 bytes
    int16_t threshEnter;     //2 bytes
    float   pidKp;           //4 bytes
    float   pidKd;           //4 bytes
    float   encKp;           //4 bytes
    uint8_t runMode;         //1 byte RunMode enum
    uint8_t enableInverted;  //1 byte (0/1)
    uint8_t skipStartExit;   //1 byte (0/1)
    uint8_t sensorSens[16];  //16 bytes: 0=off, 20..180% sensitivity
    int16_t manualLeft;      //2 bytes (-255..255)
    int16_t manualRight;     //2 bytes (-255..255)
    uint8_t checksum;        //1 byte xor
};
static_assert(sizeof(CommandPacket) == 48, "CommandPacket size mismatch");

#pragma pack(pop)

//inline helper for fast xor checksum
inline uint8_t calcProtocolChecksum(const uint8_t *data, size_t len) {
    uint8_t c = 0;
    for (size_t i = 0; i < len; i++) {
        c ^= data[i];
    }
    return c;
}
