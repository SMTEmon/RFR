#include <Arduino.h>
#include "config.h"
#include "motors.h"
#include "encoders.h"
#include "sensors.h"
#include "navigator.h"

static uint32_t lastLoopMicros = 0;
static uint32_t lastPrintMillis = 0;
static uint32_t lastRawPrintMillis = 0;

static void printSensorMap() {
    Serial.print("Map: [ ");
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        if (isSensorBlack(i)) {
            Serial.print("# ");
        } else {
            Serial.print(". ");
        }
    }
    Serial.print("] ");
}

static void printFeatureTags() {
    if (hasLeftWing())        Serial.print("[L_WING] ");
    if (hasRightWing())       Serial.print("[R_WING] ");
    if (hasLeftBevel())       Serial.print("[L_BEVEL] ");
    if (hasRightBevel())      Serial.print("[R_BEVEL] ");
    if (isInvertedPattern())  Serial.print("[INVERTED] ");
    if (isAllBlack())         Serial.print("[ALL_BLACK] ");
    if (isAllWhite())         Serial.print("[ALL_WHITE] ");
}

static void printRawADC() {
    Serial.print("Raw ADC: ");
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        Serial.print(getSensorRaw(i));
        Serial.print(" ");
    }
    Serial.println();
}

static void runBenchDiagnostics() {
    //keep motors safely stopped while testing on bench
    drive(0, 0);

    //sample sensors with normal polarity
    sensorsRead(false);

    int32_t err = computePositionError(false);
    int32_t leftEnc = getLeftTicks();
    int32_t rightEnc = getRightTicks();

    //visual 16-sensor map (# = black line, . = white floor)
    printSensorMap();
    printFeatureTags();

    //calculated pid lateral error
    Serial.printf("| Err: %5ld  L_enc: %5ld  R_enc: %5ld\n", (long)err, (long)leftEnc, (long)rightEnc);

    //print raw adc values every 1 second
    if (millis() - lastRawPrintMillis >= 1000) {
        lastRawPrintMillis = millis();
        printRawADC();
    }
}

static void runMotorTest() {
    static uint32_t phaseTimer = 0;
    static uint8_t phase = 0;

    //sample sensors so map is always live during motor test
    sensorsRead(false);

    //cycle phases every 2.5 seconds
    if (millis() - phaseTimer >= 2500) {
        phaseTimer = millis();
        phase = (phase + 1) % 4;
    }

    int32_t leftEnc = getLeftTicks();
    int32_t rightEnc = getRightTicks();
    int32_t err = computePositionError(false);

    switch (phase) {
        case 0: drive(TEST_MOTOR_SPEED, 0); break;   //left only
        case 1: drive(0, TEST_MOTOR_SPEED); break;   //right only
        case 2: drive(TEST_MOTOR_SPEED, TEST_MOTOR_SPEED); break; //both
        case 3: drive(0, 0); break;     //pause
    }

    if (millis() - lastPrintMillis >= 250) {
        lastPrintMillis = millis();
        printSensorMap();
        printFeatureTags();
        switch (phase) {
            case 0: Serial.printf("| Err: %5ld  L: %5ld  R: %5ld | [TEST] LEFT MOTOR ON (PWM %d)\n",  (long)err, (long)leftEnc, (long)rightEnc, TEST_MOTOR_SPEED); break;
            case 1: Serial.printf("| Err: %5ld  L: %5ld  R: %5ld | [TEST] RIGHT MOTOR ON (PWM %d)\n", (long)err, (long)leftEnc, (long)rightEnc, TEST_MOTOR_SPEED); break;
            case 2: Serial.printf("| Err: %5ld  L: %5ld  R: %5ld | [TEST] BOTH MOTORS ON (PWM %d)\n",  (long)err, (long)leftEnc, (long)rightEnc, TEST_MOTOR_SPEED); break;
            case 3: Serial.printf("| Err: %5ld  L: %5ld  R: %5ld | [TEST] PAUSE (BOTH OFF)\n", (long)err, (long)leftEnc, (long)rightEnc); break;
        }
    }

    if (millis() - lastRawPrintMillis >= 1000) {
        lastRawPrintMillis = millis();
        printRawADC();
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
        sensorsCalibrate();
    } else if (MOTOR_TEST_MODE) {
        Serial.println("==================================================");
        Serial.println("OPEN-LOOP MOTOR TEST ACTIVE");
        Serial.println("Cycle: Left (2.5s) -> Right (2.5s) -> Both (2.5s) -> Stop (2.5s)");
        Serial.println("==================================================");
        motorsArm(true);
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

    //open-loop motor test branch
    if (MOTOR_TEST_MODE) {
        runMotorTest();
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

        printSensorMap();
        printFeatureTags();
        Serial.printf("| State: %-15s Dist: %5.1f cm  L: %5ld  R: %5ld\n",
                      getNavStateName(),
                      ticksToCm(getAvgTicks()),
                      (long)getLeftTicks(),
                      (long)getRightTicks());
    }

    if (millis() - lastRawPrintMillis >= 1000) {
        lastRawPrintMillis = millis();
        printRawADC();
    }
}
