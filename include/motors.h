#pragma once
#include <Arduino.h>

//init pwm channels and direction pins
void motorsInit();

//toggle stby pin
void motorsArm(bool enable);

//set wheel speeds (-255 to 255)
void drive(int16_t leftSpeed, int16_t rightSpeed);

//active electromagnetic brake on both motors
void shortBrake();

//coast stop
void coastStop();
