#include "motors.h"
#include "config.h"
#include "telemetry.h"

static int16_t lastLeft = 0;
static int16_t lastRight = 0;
static int16_t motorAccelSlew = DEFAULT_ACCEL_SLEW;
static MotorEnvelopeState g_motorEnvelope;

//centralized pwm duty write with anti-phase interleaving
static void writeMotorPwmDuty(uint8_t leftDuty, uint8_t rightDuty) {
    ledcWrite(PIN_PWMA, leftDuty);
    //pwmb is output inverted so write 255 - duty for 180 deg anti-phase
    ledcWrite(PIN_PWMB, 255 - rightDuty);
}

void motorsInit() {
    //force stby and pwms low first to prevent boot twitch and strap conflicts
    pinMode(PIN_STBY, OUTPUT);
    digitalWrite(PIN_STBY, LOW);
    pinMode(PIN_PWMA, OUTPUT);
    digitalWrite(PIN_PWMA, LOW);
    pinMode(PIN_PWMB, OUTPUT);
    digitalWrite(PIN_PWMB, LOW);

    //setup direction pins
    pinMode(PIN_AIN1, OUTPUT);
    pinMode(PIN_AIN2, OUTPUT);
    pinMode(PIN_BIN1, OUTPUT);
    pinMode(PIN_BIN2, OUTPUT);

    //setup 20khz pwm on dedicated channels
    ledcAttachChannel(PIN_PWMA, PWM_FREQ_HZ, PWM_RES_BITS, PWM_CH_LEFT);
    ledcAttachChannel(PIN_PWMB, PWM_FREQ_HZ, PWM_RES_BITS, PWM_CH_RIGHT);

    //enable 180 deg anti-phase on right motor to halve peak current draw
    ledcWrite(PIN_PWMA, 0);
    ledcWrite(PIN_PWMB, 255);
    ledcOutputInvert(PIN_PWMB, true);

    lastLeft = 0;
    lastRight = 0;
    g_motorEnvelope.reset();
    motorAccelSlew = DEFAULT_ACCEL_SLEW;
}

void motorsArm(bool enable) {
    //enable or disable motor driver h-bridge
    digitalWrite(PIN_STBY, enable ? HIGH : LOW);
    if (!enable) {
        g_motorEnvelope.reset();
        writeMotorPwmDuty(0, 0);
    }
}

void setMotorAccelSlew(int16_t slew) {
    motorAccelSlew = constrain(slew, 1, 25);
}

int16_t getMotorAccelSlew() {
    return motorAccelSlew;
}

static void applyHardwarePwm(int16_t leftSpeed, int16_t rightSpeed) {
    lastLeft = leftSpeed;
    lastRight = rightSpeed;

    //apply polarity flags
    if (INVERT_LEFT_MOTOR)  leftSpeed  = -leftSpeed;
    if (INVERT_RIGHT_MOTOR) rightSpeed = -rightSpeed;

    uint8_t dutyL = 0;
    uint8_t dutyR = 0;

    //left motor control
    if (leftSpeed > 0) {
        digitalWrite(PIN_AIN1, HIGH);
        digitalWrite(PIN_AIN2, LOW);
        dutyL = static_cast<uint8_t>(leftSpeed);
    } else if (leftSpeed < 0) {
        digitalWrite(PIN_AIN1, LOW);
        digitalWrite(PIN_AIN2, HIGH);
        dutyL = static_cast<uint8_t>(-leftSpeed);
    } else {
        //coast when stopped
        digitalWrite(PIN_AIN1, LOW);
        digitalWrite(PIN_AIN2, LOW);
        dutyL = 0;
    }

    //right motor control
    if (rightSpeed > 0) {
        digitalWrite(PIN_BIN1, HIGH);
        digitalWrite(PIN_BIN2, LOW);
        dutyR = static_cast<uint8_t>(rightSpeed);
    } else if (rightSpeed < 0) {
        digitalWrite(PIN_BIN1, LOW);
        digitalWrite(PIN_BIN2, HIGH);
        dutyR = static_cast<uint8_t>(-rightSpeed);
    } else {
        //coast when stopped
        digitalWrite(PIN_BIN1, LOW);
        digitalWrite(PIN_BIN2, LOW);
        dutyR = 0;
    }

    writeMotorPwmDuty(dutyL, dutyR);
}

void driveDirect(int16_t leftSpeed, int16_t rightSpeed) {
    int16_t limit = (g_cfg.maxSpeed > 0 && g_cfg.maxSpeed <= 255) ? g_cfg.maxSpeed : 255;
    leftSpeed = constrain(leftSpeed, -limit, limit);
    rightSpeed = constrain(rightSpeed, -limit, limit);

    int16_t outL = 0;
    int16_t outR = 0;
    //direct drive allows faster slew but still protects power rail and zero crossings
    int16_t directAccel = max<int16_t>(motorAccelSlew, 4);
    stepDualMotorEnvelope(g_motorEnvelope,
                          leftSpeed, rightSpeed,
                          directAccel, DEFAULT_DECEL_SLEW,
                          MAX_WHEEL_STEP_PWM, REVERSAL_COAST_TICKS,
                          outL, outR);

    applyHardwarePwm(outL, outR);
}

void drive(int16_t leftSpeed, int16_t rightSpeed) {
    int16_t limit = (g_cfg.maxSpeed > 0 && g_cfg.maxSpeed <= 255) ? g_cfg.maxSpeed : 255;
    leftSpeed = constrain(leftSpeed, -limit, limit);
    rightSpeed = constrain(rightSpeed, -limit, limit);

    int16_t outL = 0;
    int16_t outR = 0;
    stepDualMotorEnvelope(g_motorEnvelope,
                          leftSpeed, rightSpeed,
                          motorAccelSlew, DEFAULT_DECEL_SLEW,
                          MAX_WHEEL_STEP_PWM, REVERSAL_COAST_TICKS,
                          outL, outR);

    applyHardwarePwm(outL, outR);
}

void shortBrake() {
    //tb6612fng ignores pwm when in1=h & in2=h; coast instead to prevent rail collapse
    coastStop();
}

void coastStop() {
    digitalWrite(PIN_AIN1, LOW);
    digitalWrite(PIN_AIN2, LOW);
    digitalWrite(PIN_BIN1, LOW);
    digitalWrite(PIN_BIN2, LOW);
    writeMotorPwmDuty(0, 0);
    lastLeft = 0;
    lastRight = 0;
    g_motorEnvelope.reset();
}

int16_t getLastLeftPwm() {
    return lastLeft;
}

int16_t getLastRightPwm() {
    return lastRight;
}
