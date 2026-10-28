#include <Arduino.h>
#include "config.h"
#include "motors.h"
#include "encoders.h"
#include "sensors.h"
#include "navigator.h"

static uint32_t lastLoopMicros = 0;
static uint32_t lastPrintMillis = 0;

void setup() {
    Serial.begin(115200);

    //setup motors with stby low during boot
    motorsInit();

    //setup encoders and sensors
    encodersInit();
    sensorsInit();

    Serial.println("--- LFR Booting ---");

    if (SKIP_CALIBRATION) {
        Serial.println("Bench mode: Hand-sweep calibration skipped!");
    } else {
        Serial.println("Sweep sensor bar over black and white for 5s...");
    }

    //calibrate min max or load bench defaults
    sensorsCalibrate();

    if (!SKIP_CALIBRATION) {
        Serial.println("Calibration complete. Placing inside #31...");
    }

    //arm motors and launch navigator
    motorsArm(true);
    navigatorInit();

    lastLoopMicros = micros();
    Serial.println("Starting Round 1 demo run!");
}

void loop() {
    uint32_t currentMicros = micros();

    //deterministic 500hz control loop
    if (currentMicros - lastLoopMicros >= LOOP_INTERVAL_US) {
        lastLoopMicros += LOOP_INTERVAL_US;

        //advance state machine and motor outputs
        navigatorUpdate();
    }

    //debug telemetry every 100ms
    if (millis() - lastPrintMillis >= 100) {
        lastPrintMillis = millis();

        Serial.print("State: ");
        Serial.print(getNavStateName());
        Serial.print("\tL_enc: ");
        Serial.print(getLeftTicks());
        Serial.print("\tR_enc: ");
        Serial.print(getRightTicks());
        Serial.print("\tDist: ");
        Serial.print(ticksToCm(getAvgTicks()), 1);
        Serial.println(" cm");
    }
}
