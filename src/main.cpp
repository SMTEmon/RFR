#include <Arduino.h>
#include "config.h"
#include "motors.h"
#include "encoders.h"
#include "sensors.h"
#include "navigator.h"

static uint32_t lastLoopMicros = 0;
static uint32_t lastPrintMillis = 0;
static uint32_t lastRawPrintMillis = 0;

static void runBenchDiagnostics() {
    //keep motors safely stopped while testing on bench
    drive(0, 0);

    //sample sensors with normal polarity
    sensorsRead(false);

    int32_t err = computePositionError(false);
    int32_t leftEnc = getLeftTicks();
    int32_t rightEnc = getRightTicks();

    //visual 16-sensor map (# = black line, . = white floor)
    Serial.print("Map: [ ");
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        if (isSensorBlack(i)) {
            Serial.print("# ");
        } else {
            Serial.print(". ");
        }
    }
    Serial.print("] ");

    //calculated pid lateral error
    Serial.print("Err: ");
    Serial.print(err);

    //wheel encoder counts (rotate wheels by hand to verify direction)
    Serial.print("\tL_enc: ");
    Serial.print(leftEnc);
    Serial.print("  R_enc: ");
    Serial.print(rightEnc);

    //detected track feature flags
    if (hasLeftWing())        Serial.print(" [L_WING]");
    if (hasRightWing())       Serial.print(" [R_WING]");
    if (hasLeftBevel())       Serial.print(" [L_BEVEL]");
    if (hasRightBevel())      Serial.print(" [R_BEVEL]");
    if (isInvertedPattern())  Serial.print(" [INVERTED]");
    if (isAllBlack())         Serial.print(" [ALL_BLACK]");
    if (isAllWhite())         Serial.print(" [ALL_WHITE]");

    Serial.println();

    //print raw adc values every 1 second
    if (millis() - lastRawPrintMillis >= 1000) {
        lastRawPrintMillis = millis();
        Serial.print("Raw ADC: ");
        for (uint8_t i = 0; i < NUM_SENSORS; i++) {
            Serial.print(getSensorRaw(i));
            Serial.print(" ");
        }
        Serial.println();
    }
}

void setup() {
    Serial.begin(115200);

    //setup motors with stby low during boot
    motorsInit();

    //setup encoders and sensors
    encodersInit();
    sensorsInit();

    Serial.println("--- LFR Booting ---");

    if (BENCH_DEBUG_MODE) {
        Serial.println("==================================================");
        Serial.println("BENCH DIAGNOSTIC MODE ACTIVE (Motors Safe)");
        Serial.println("Slide black tape across sensors to see [ # . ] map");
        Serial.println("Rotate wheels by hand to check L/R encoder counts");
        Serial.println("==================================================");
    } else {
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
        Serial.println("Starting Round 1 demo run!");
    }

    lastLoopMicros = micros();
}

void loop() {
    //bench diagnostic mode branch
    if (BENCH_DEBUG_MODE) {
        if (millis() - lastPrintMillis >= 150) {
            lastPrintMillis = millis();
            runBenchDiagnostics();
        }
        return;
    }

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
