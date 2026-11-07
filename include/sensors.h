#pragma once
#include <Arduino.h>
#include "config.h"

//init mux pins and adc
void sensorsInit();

//calibrate min max during boot sweep (blocking legacy or fallback)
void sensorsCalibrate();

//runtime dynamic calibration
void sensorsStartCalibration();
bool sensorsUpdateCalibration(); //returns true when 5s sweep finishes
void sensorsResetCalibrationDefault();
void sensorsResetCalibrationMinMax();
void sensorsSampleCalibrationStep();
void sensorsFinalizeCalibration();
bool isSensorCalibrating();

//runtime schmitt threshold & per-sensor sensitivity adjustment
void setSensorThreshold(int16_t enterThresh);
int16_t getSensorThreshold();
void setSensorSensitivity(uint8_t idx, uint8_t sensPct);
void setAllSensorSensitivities(const uint8_t sens[NUM_SENSORS]);
uint8_t getSensorSensitivity(uint8_t idx);

//read all 16 channels through mux
void sensorsRead(bool isInverted);

//get normalized line value (0 white, 1000 black)
int16_t getSensorValue(uint8_t idx);

//get un-equalized normalized dark value (0 white, 1000 black)
int16_t getSensorNormalizedDark(uint8_t idx);

//get raw unnormalized adc reading (0..4095)
uint16_t getSensorRaw(uint8_t idx);

//get common-mode pitch-up nose lift index (0..255)
uint8_t getLastFrameLift();

//returns true if bot is picked up in air
bool isRobotLifted();
void resetAirLiftDetector();

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
