#pragma once
#include <Arduino.h>

//init isr and configure pins
void encodersInit();

//reset tick counters to zero
void resetEncoders();

//get instantaneous counts
int32_t getLeftTicks();
int32_t getRightTicks();
int32_t getAvgTicks();

//convert ticks to physical centimeters
float ticksToCm(int32_t ticks);
int32_t cmToTicks(float cm);
