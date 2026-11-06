#pragma once
#include <Arduino.h>

//init pwm channels and direction pins
void motorsInit();

//toggle stby pin
void motorsArm(bool enable);

//set wheel speeds (-255 to 255) with forward throttle slew rate limiting
void drive(int16_t leftSpeed, int16_t rightSpeed);

//direct drive without slew limiting (used for spins, bench tests, and emergency maneuvers)
void driveDirect(int16_t leftSpeed, int16_t rightSpeed);

//set acceleration slew rate limit (+pwm per 2ms tick, 1..25)
void setMotorAccelSlew(int16_t slew);
int16_t getMotorAccelSlew();

//active electromagnetic brake on both motors
void shortBrake();

//coast stop
void coastStop();

//get last applied pwm outputs for telemetry
int16_t getLastLeftPwm();
int16_t getLastRightPwm();
