#pragma once
#include <Arduino.h>
#include "config.h"

//init mux pins and adc
void sensorsInit();

//calibrate min max during boot sweep
void sensorsCalibrate();

//read all 16 channels through mux
void sensorsRead(bool isInverted);

//get normalized line value (0 white, 1000 black)
int16_t getSensorValue(uint8_t idx);

//get raw unnormalized adc reading (0..4095)
uint16_t getSensorRaw(uint8_t idx);

//schmitt trigger boolean states
bool isSensorBlack(uint8_t idx);

//track pattern flags
bool hasLine();
bool isAllWhite();
bool isAllBlack();
bool isFrontAllBlack();
bool hasLeftWing();
bool hasRightWing();
bool hasLeftBevel();
bool hasRightBevel();
bool isInvertedPattern();

//compute weighted lateral error in 0.01mm scale
int32_t computePositionError(bool isInverted);

//debug helpers
void printSensorRaw();
void printSensorCalibrated();
