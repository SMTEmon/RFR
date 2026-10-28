#include "motors.h"
#include "config.h"

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
}

void motorsArm(bool enable) {
    //enable or disable motor driver h-bridge
    digitalWrite(PIN_STBY, enable ? HIGH : LOW);
}

void drive(int16_t leftSpeed, int16_t rightSpeed) {
    //clamp values to 8bit pwm bounds
    leftSpeed = constrain(leftSpeed, -255, 255);
    rightSpeed = constrain(rightSpeed, -255, 255);

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

void shortBrake() {
    //hard active brake using motor back emf
    digitalWrite(PIN_AIN1, HIGH);
    digitalWrite(PIN_AIN2, HIGH);
    digitalWrite(PIN_BIN1, HIGH);
    digitalWrite(PIN_BIN2, HIGH);
    ledcWrite(PIN_PWMA, 255);
    ledcWrite(PIN_PWMB, 255);
}

void coastStop() {
    //passive float stop
    digitalWrite(PIN_AIN1, LOW);
    digitalWrite(PIN_AIN2, LOW);
    digitalWrite(PIN_BIN1, LOW);
    digitalWrite(PIN_BIN2, LOW);
    ledcWrite(PIN_PWMA, 0);
    ledcWrite(PIN_PWMB, 0);
}
