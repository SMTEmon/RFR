#pragma once
#if defined(ARDUINO)
#include <Arduino.h>
#else
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#endif

//shared communication protocol between lfr robot and dongle
//all structs packed 1-byte aligned to match exactly across esp32 targets

enum RunMode : uint8_t {
    MODE_AUTONOMOUS   = 0,
    MODE_BENCH_DIAG   = 1,
    MODE_MOTOR_TEST   = 2,
    MODE_MANUAL_WASD  = 3,
    MODE_MAP_AND_TUNE = 4,
    MODE_MAPPED_RACE  = 5
};

enum CmdType : uint8_t {
    CMD_NOP             = 0,
    CMD_STOP            = 1,
    CMD_START           = 2,
    CMD_SET_PARAMS      = 3,
    CMD_SAVE_NVS        = 4,
    CMD_RESET_ENC       = 5,
    CMD_MANUAL_DRIVE    = 6,
    CMD_SYNC_CFG        = 7,
    CMD_CALIB_START     = 8,
    CMD_CALIB_DEFAULT   = 9,
    CMD_MAP_START_CRAWL = 10,
    CMD_MAP_MARK_LAP    = 11,
    CMD_TUNE_VALIDATE   = 12,
    CMD_TUNE_SAVE_ALL   = 13,
    CMD_MAP_REQUEST     = 14
};

enum AckEvent : uint8_t {
    ACK_NONE           = 0,
    ACK_PARAMS_OK      = 1,
    ACK_NVS_SAVED      = 2,
    ACK_CALIB_DONE     = 3,
    ACK_CALIB_FALLBACK = 4,
    ACK_CALIB_DEFAULT  = 5,
    ACK_ENC_RESET      = 6,
    ACK_MAP_SAVED      = 7,
    ACK_TUNE_COMPLETE  = 8,
    ACK_BOOT_BROWNOUT  = 9,
    ACK_BOOT_PANIC     = 10
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

//state name lookup helper shared between robot and dongle
inline const char* navStateToStr(uint8_t st) {
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

enum TuneState : uint8_t {
    TUNE_IDLE             = 0,
    TUNE_PIVOT_CAL        = 1,
    TUNE_CRAWL_MAP        = 2,
    TUNE_WAIT_VALIDATE    = 3,
    TUNE_SPEED_SWEEP      = 4,
    TUNE_COMPLETE         = 5,
    TUNE_SAFE_ABORT       = 6
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
    uint8_t accelSlew;       //1 byte (1..20 pwm/tick)
    uint8_t junctionMode;    //1 byte (0=loop straight, 1=lhr, 2=rhr)
    uint8_t allowUturn;      //1 byte (0=stop on line loss, 1=u-turn)
    uint8_t ackEvent;        //1 byte AckEvent enum
    uint8_t checksum;        //1 byte xor
};
static_assert(sizeof(ConfigSyncPacket) == 47, "ConfigSyncPacket size mismatch");

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
    uint8_t accelSlew;       //1 byte (1..20 pwm/tick)
    uint8_t junctionMode;    //1 byte (0=loop straight, 1=lhr, 2=rhr)
    uint8_t allowUturn;      //1 byte (0=stop on line loss, 1=u-turn)
    int16_t manualLeft;      //2 bytes (-255..255)
    int16_t manualRight;     //2 bytes (-255..255)
    uint8_t checksum;        //1 byte xor
};
static_assert(sizeof(CommandPacket) == 51, "CommandPacket size mismatch");

//0xB1: downsampled 120-bin closed loop heading profile for 2d canvas rendering (146 bytes)
struct MapPathPacket {
    uint8_t  header;           //0xB1
    uint16_t totalLengthCm;    //loop perimeter in cm
    int8_t   windingSign;      //+1 ccw, -1 cw, 0 figure-8
    uint8_t  hasGaps;          //0 if continuous tape loop
    uint8_t  numPoints;        //up to 120 evenly spaced points around loop
    int8_t   dThetaScaled[120];//heading delta per bin (scaled * 250 rad)
    uint8_t  optSens[16];      //recommended per-sensor sensitivity %
    int16_t  optThresh;        //recommended master black threshold
    uint8_t  optAccelSlew;     //recommended launch slew rate
    uint8_t  checksum;
};
static_assert(sizeof(MapPathPacket) == 146, "MapPathPacket size mismatch");

//0xB2: extracted track segments (147 bytes)
struct SegmentWire {
    uint8_t  type;             //0=straight, 1=gentle_l, 2=gentle_r, 3=sharp_l, 4=sharp_r
    uint16_t startCm;
    uint16_t lengthCm;
    int16_t  radiusCm;         //signed radius (0 = straight)
    uint8_t  maxSafePwm;       //learned max pwm for this segment
};
struct MapSegmentsPacket {
    uint8_t     header;        //0xB2
    uint8_t     numSegments;   //1..18
    SegmentWire segs[18];      //18 * 8 = 144 bytes
    uint8_t     checksum;
};
static_assert(sizeof(MapSegmentsPacket) == 147, "MapSegmentsPacket size mismatch");

//0xB3: live mapping & auto-tune telemetry status (34 bytes)
struct TuneStatusPacket {
    uint8_t  header;           //0xB3
    uint8_t  tuneState;        //TuneState enum
    uint8_t  currentTier;      //0..7
    uint8_t  totalTiers;       //8
    uint8_t  subTrial;         //0=base, 1=kp_probe, 2=kd_probe
    uint8_t  lapNumber;        //current lap count
    uint8_t  currentSegIdx;    //live segment index on 2d map
    uint8_t  overlapMatchPct;  //0..100% loop closure confidence
    uint16_t currentDistCm;    //live position along loop
    uint16_t totalLengthCm;    //total loop length
    int16_t  netHeadingDeg;    //accumulated heading during crawl
    int16_t  straightPwm;      //current straight test pwm
    int16_t  cornerPwm;        //current corner test pwm
    float    activeKp;         //4 bytes active kp
    float    activeKd;         //4 bytes active kd
    uint16_t lastRmsErr01Mm;   //in 0.1mm
    uint16_t lastMaxErr01Mm;   //in 0.1mm
    uint8_t  lastQualityScore; //0..100
    uint8_t  lastLiftPeak;     //peak nose lift detected
    uint8_t  statusFlags;      //bit0: map_valid, bit1: lut_valid, bit2: soft_save_triggered, bit3: enc_fault
    uint8_t  checksum;
};
static_assert(sizeof(TuneStatusPacket) == 34, "TuneStatusPacket size mismatch");

//0xB4: 8-tier learned speed-to-pid lookup table (99 bytes)
struct SpeedTierWire {
    int16_t  straightPwm;
    int16_t  cornerPwm;
    int16_t  kpX10000;         //e.g. 450 -> 0.0450
    int16_t  kdX1000;          //e.g. 280 -> 0.280
    uint16_t rmsErr01Mm;       //rms error in 0.1mm
    uint8_t  qualityScore;     //0..100
    uint8_t  state;            //0=pending, 1=testing, 2=locked, 3=capped
};
struct GainTablePacket {
    uint8_t       header;      //0xB4
    uint8_t       numTiers;    //8
    SpeedTierWire tiers[8];    //8 * 12 = 96 bytes
    uint8_t       checksum;
};
static_assert(sizeof(GainTablePacket) == 99, "GainTablePacket size mismatch");

#pragma pack(pop)

//inline helper for fast xor checksum
inline uint8_t calcProtocolChecksum(const uint8_t *data, size_t len) {
    uint8_t c = 0;
    for (size_t i = 0; i < len; i++) {
        c ^= data[i];
    }
    return c;
}
