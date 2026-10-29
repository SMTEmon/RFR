#pragma once
#include <Arduino.h>

//tb6612fng pins
constexpr uint8_t PIN_PWMA = 13;
constexpr uint8_t PIN_AIN1 = 14;
constexpr uint8_t PIN_AIN2 = 4;
constexpr uint8_t PIN_STBY = 27;
constexpr uint8_t PIN_BIN1 = 26;
constexpr uint8_t PIN_BIN2 = 25;
constexpr uint8_t PIN_PWMB = 33;

//pwm channels and frequency
constexpr uint8_t PWM_CH_LEFT  = 0;
constexpr uint8_t PWM_CH_RIGHT = 1;
constexpr uint32_t PWM_FREQ_HZ = 20000;
constexpr uint8_t PWM_RES_BITS = 8; //8bit 0..255

//encoders (g35 is input only without pullup)
constexpr uint8_t PIN_ENC_LA = 32;
constexpr uint8_t PIN_ENC_LB = 22;
constexpr uint8_t PIN_ENC_RA = 16;
constexpr uint8_t PIN_ENC_RB = 17;

//cd74hc4067 multiplexer pins
constexpr uint8_t PIN_MUX_S0  = 5;
constexpr uint8_t PIN_MUX_S1  = 18;
constexpr uint8_t PIN_MUX_S2  = 19;
constexpr uint8_t PIN_MUX_S3  = 21;
constexpr uint8_t PIN_MUX_SIG = 15; //adc1 channel 6
constexpr uint8_t PIN_LED     = 2;  //devkit onboard led

//sensor array geometry
constexpr uint8_t NUM_SENSORS = 16;

//logical idx 0 (left wing) -> c0 ... idx 15 (right wing) -> c15
inline constexpr uint8_t logicalToMux(uint8_t idx) {
    return idx;
}

//exact cad coordinates from final.dxf in mm
constexpr float SENSOR_X_MM[NUM_SENSORS] = {
    -52.2f, -52.2f, -52.2f, -43.6f,
    -35.0f, -25.0f, -15.0f,  -5.0f,   5.0f,  15.0f,  25.0f,  35.0f,
     43.6f,  52.2f,  52.2f,  52.2f
};

constexpr float SENSOR_Y_MM[NUM_SENSORS] = {
    -54.6f, -39.6f, -24.6f, -12.3f,
      0.0f,   0.0f,   0.0f,   0.0f,   0.0f,   0.0f,   0.0f,   0.0f,
    -12.3f, -24.6f, -39.6f, -54.6f
};

//cad lateral weights in 0.01mm scale for pid error
constexpr int32_t PID_WEIGHTS_12[12] = {
    -5220, -4360, -3500, -2500, -1500, -500, 500, 1500, 2500, 3500, 4360, 5220
};

constexpr int32_t PID_WEIGHTS_8[8] = {
    -3500, -2500, -1500, -500, 500, 1500, 2500, 3500
};

//detection thresholds
constexpr int16_t THRESH_BLACK_ENTER = 500; //schmitt trigger on (500 catches ~2100 adc on uncalibrated sensors)
constexpr int16_t THRESH_BLACK_EXIT  = 350; //schmitt trigger off

//timing
constexpr uint32_t LOOP_INTERVAL_US = 2000; //500hz loop
constexpr uint32_t MUX_SETTLE_US    = 20;   //rfr.ino used 15us, 20us ensures stable adc
constexpr uint32_t CALIB_SWEEP_MS   = 5000; //hand sweep duration
constexpr uint32_t CALIB_PLACE_MS   = 2000; //placement window

//bench test override: skip sweep when bot cannot be lifted
constexpr bool SKIP_CALIBRATION      = true; //set false when on full track
constexpr uint16_t DEFAULT_RAW_MIN   = 200;  //fallback ambient/dark adc (from rfr.ino)
constexpr uint16_t DEFAULT_RAW_MAX   = 4000; //fallback reflection/white adc (from rfr.ino)

//sensor polarity:
//hardware has pull-up to vcc:
//white surface reflects ir -> phototransistor conducts to gnd -> low adc (~200)
//black line absorbs ir -> phototransistor off -> pull-up holds high -> high adc (~4000)
constexpr bool ADC_HIGH_IS_BLACK     = true;

//modes:
//BENCH_DEBUG_MODE: safe sensor diagnostic (motors 0)
//MOTOR_TEST_MODE: open-loop sequential motor test (left -> right -> both -> stop)
//set both false to run full autonomous navigation on track
constexpr bool BENCH_DEBUG_MODE      = false;
constexpr bool MOTOR_TEST_MODE       = false;

//odometry & distance thresholds
//shortcut: default 28 ticks per cm, calibrate with ruler on bench
constexpr float TICKS_PER_CM         = 28.0f;
constexpr int32_t START_EXIT_TICKS   = static_cast<int32_t>(12.0f * TICKS_PER_CM);
constexpr int32_t MAX_GAP_TICKS      = static_cast<int32_t>(14.0f * TICKS_PER_CM);
constexpr int32_t WING_SETBACK_TICKS = static_cast<int32_t>(5.5f  * TICKS_PER_CM);
constexpr int32_t STOP_BOX_TICKS     = static_cast<int32_t>(7.5f  * TICKS_PER_CM);
constexpr int32_t LATCH_WINDOW_TICKS = static_cast<int32_t>(8.0f  * TICKS_PER_CM);
constexpr int32_t UTURN_180_TICKS    = static_cast<int32_t>(20.0f * TICKS_PER_CM);

//open-loop test speed
constexpr int16_t TEST_MOTOR_SPEED   = 130;   //pwm duty cycle for open-loop motor test (0..255)

//motor & encoder polarity
constexpr bool INVERT_LEFT_MOTOR     = true; //flipped to false so left motor rolls forward
constexpr bool INVERT_RIGHT_MOTOR    = false; //inverts right motor forward direction
constexpr bool INVERT_LEFT_ENC       = true; //toggle if left encoder counts down when rolling forward
constexpr bool INVERT_RIGHT_ENC      = true; //toggle if right encoder counts down when rolling forward

//base speed and pid tuning gains
constexpr int16_t BASE_SPEED         = 140;
constexpr int16_t MIN_SPEED          = 80;
constexpr int16_t MAX_SPEED          = 220;
constexpr int16_t TURN_SPEED         = 120;

constexpr float PID_KP               = 0.045f;
constexpr float PID_KD               = 0.280f;
constexpr float ENC_KP               = 2.0f; //encoder straight heading lock gain
