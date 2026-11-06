#include "motors.h"
#include "config.h"
#include "telemetry.h"

static int16_t lastLeft = 0;
static int16_t lastRight = 0;
static int16_t currentLeft = 0;
static int16_t currentRight = 0;
static int16_t motorAccelSlew = DEFAULT_ACCEL_SLEW;

void motorsInit() {
    //force stby low first to prevent boot twitch
    pinMode(PIN_STBY, OUTPUT);
    digitalWrite(PIN_STBY, LOW);

    //setup direction pins
    pinMode(PIN_AIN1, OUTPUT);
    pinMode(PIN_AIN2, OUTPUT);
    pinMode(PIN_BIN1, OUTPUT);
    pinMode(PIN_BIN2, OUTPUT);

    //setup 20khz pwm on pins (esp32 core v3 api)
    ledcAttach(PIN_PWMA, PWM_FREQ_HZ, PWM_RES_BITS);
    ledcAttach(PIN_PWMB, PWM_FREQ_HZ, PWM_RES_BITS);

    //ensure outputs are off
    ledcWrite(PIN_PWMA, 0);
    ledcWrite(PIN_PWMB, 0);

    lastLeft = 0;
    lastRight = 0;
    currentLeft = 0;
    currentRight = 0;
    motorAccelSlew = DEFAULT_ACCEL_SLEW;
}

void motorsArm(bool enable) {
    //enable or disable motor driver h-bridge
    digitalWrite(PIN_STBY, enable ? HIGH : LOW);
    if (!enable) {
        currentLeft = 0;
        currentRight = 0;
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

    //left motor control
    if (leftSpeed > 0) {
        digitalWrite(PIN_AIN1, HIGH);
        digitalWrite(PIN_AIN2, LOW);
        ledcWrite(PIN_PWMA, leftSpeed);
    } else if (leftSpeed < 0) {
        digitalWrite(PIN_AIN1, LOW);
        digitalWrite(PIN_AIN2, HIGH);
        ledcWrite(PIN_PWMA, -leftSpeed);
    } else {
        //short brake when stopped
        digitalWrite(PIN_AIN1, HIGH);
        digitalWrite(PIN_AIN2, HIGH);
        ledcWrite(PIN_PWMA, 0);
    }

    //right motor control
    if (rightSpeed > 0) {
        digitalWrite(PIN_BIN1, HIGH);
        digitalWrite(PIN_BIN2, LOW);
        ledcWrite(PIN_PWMB, rightSpeed);
    } else if (rightSpeed < 0) {
        digitalWrite(PIN_BIN1, LOW);
        digitalWrite(PIN_BIN2, HIGH);
        ledcWrite(PIN_PWMB, -rightSpeed);
    } else {
        //short brake when stopped
        digitalWrite(PIN_BIN1, HIGH);
        digitalWrite(PIN_BIN2, HIGH);
        ledcWrite(PIN_PWMB, 0);
    }
}

void driveDirect(int16_t leftSpeed, int16_t rightSpeed) {
    int16_t limit = (g_cfg.maxSpeed > 0 && g_cfg.maxSpeed <= 255) ? g_cfg.maxSpeed : 255;
    currentLeft = constrain(leftSpeed, -limit, limit);
    currentRight = constrain(rightSpeed, -limit, limit);
    applyHardwarePwm(currentLeft, currentRight);
}

void drive(int16_t leftSpeed, int16_t rightSpeed) {
    int16_t limit = (g_cfg.maxSpeed > 0 && g_cfg.maxSpeed <= 255) ? g_cfg.maxSpeed : 255;
    leftSpeed = constrain(leftSpeed, -limit, limit);
    rightSpeed = constrain(rightSpeed, -limit, limit);

    //slew rate limiting only on positive forward acceleration (to stop pitch-up lift)
    //braking/deceleration is instant for safety
    if (leftSpeed > currentLeft) {
        currentLeft = min<int16_t>(leftSpeed, currentLeft + motorAccelSlew);
    } else {
        currentLeft = leftSpeed; //instant decel
    }

    if (rightSpeed > currentRight) {
        currentRight = min<int16_t>(rightSpeed, currentRight + motorAccelSlew);
    } else {
        currentRight = rightSpeed; //instant decel
    }

    applyHardwarePwm(currentLeft, currentRight);
}

void shortBrake() {
    digitalWrite(PIN_AIN1, HIGH);
    digitalWrite(PIN_AIN2, HIGH);
    digitalWrite(PIN_BIN1, HIGH);
    digitalWrite(PIN_BIN2, HIGH);
    ledcWrite(PIN_PWMA, 255);
    ledcWrite(PIN_PWMB, 255);
    lastLeft = 0;
    lastRight = 0;
    currentLeft = 0;
    currentRight = 0;
}

void coastStop() {
    digitalWrite(PIN_AIN1, LOW);
    digitalWrite(PIN_AIN2, LOW);
    digitalWrite(PIN_BIN1, LOW);
    digitalWrite(PIN_BIN2, LOW);
    ledcWrite(PIN_PWMA, 0);
    ledcWrite(PIN_PWMB, 0);
    lastLeft = 0;
    lastRight = 0;
    currentLeft = 0;
    currentRight = 0;
}

int16_t getLastLeftPwm() {
    return lastLeft;
}

int16_t getLastRightPwm() {
    return lastRight;
}
