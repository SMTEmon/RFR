#include <Arduino.h>
#include "config.h"
#include "motors.h"
#include "encoders.h"
#include "sensors.h"
#include "navigator.h"
#include "telemetry.h"

static uint32_t lastLoopMicros = 0;
static uint32_t lastPrintMillis = 0;
static uint32_t lastRawPrintMillis = 0;
static bool wasArmed = false;
static uint32_t stopBrakeTimer = 0;

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

    //cycle phases every 2.5 seconds
    if (millis() - phaseTimer >= 2500) {
        phaseTimer = millis();
        phase = (phase + 1) % 4;
    }

    int16_t spd = g_cfg.baseSpeed;
    switch (phase) {
        case 0: drive(spd, 0); break;   //left only
        case 1: drive(0, spd); break;   //right only
        case 2: drive(spd, spd); break; //both
        case 3: drive(0, 0); break;     //pause
    }

    if (millis() - lastPrintMillis >= 250) {
        lastPrintMillis = millis();
        int32_t leftEnc = getLeftTicks();
        int32_t rightEnc = getRightTicks();
        int32_t err = computePositionError(false);
        printSensorMap();
        printFeatureTags();
        switch (phase) {
            case 0: Serial.printf("| Err: %5ld  L: %5ld  R: %5ld | [TEST] LEFT MOTOR ON (PWM %d)\n",  (long)err, (long)leftEnc, (long)rightEnc, spd); break;
            case 1: Serial.printf("| Err: %5ld  L: %5ld  R: %5ld | [TEST] RIGHT MOTOR ON (PWM %d)\n", (long)err, (long)leftEnc, (long)rightEnc, spd); break;
            case 2: Serial.printf("| Err: %5ld  L: %5ld  R: %5ld | [TEST] BOTH MOTORS ON (PWM %d)\n",  (long)err, (long)leftEnc, (long)rightEnc, spd); break;
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

    //setup esp-now wireless telemetry and nvs config
    telemetryInit();

    Serial.println("--- LFR Booting (Motors Disarmed) ---");

    //calibrate min max or load bench defaults
    sensorsCalibrate();

    //ensure safe disarmed state on boot
    g_armed = false;
    motorsArm(false);
    coastStop();

    navigatorInit(g_cfg.skipStartExit);

    lastLoopMicros = micros();
}

void loop() {
    //process incoming esp-now commands on core 1
    telemetryProcessCommands();

    //broadcast telemetry packet to dongle at 25hz
    telemetryUpdate();

    //sample all 16 sensors at 500hz so dashboard adc bars and telemetry never freeze
    sensorsRead(isNavigatorInverted());

    //handle calibration sweep if triggered
    if (isSensorCalibrating()) {
        motorsArm(false);
        coastStop();
        if (sensorsUpdateCalibration()) {
            telemetrySendConfig(ACK_CALIB_DONE);
            Serial.println("//calibration sweep complete");
        }
        return;
    }

    //handle disarmed and active stop braking transition
    if (!g_armed) {
        if (wasArmed) {
            //robot was just running, apply 150ms active electromagnetic short brake
            wasArmed = false;
            stopBrakeTimer = millis();
            motorsArm(true);
            shortBrake();
        } else if (stopBrakeTimer > 0 && millis() - stopBrakeTimer < STOP_BRAKE_MS) {
            //holding active brake
            motorsArm(true);
            shortBrake();
        } else {
            //transition to passive coast and disable h-bridge
            stopBrakeTimer = 0;
            coastStop();
            motorsArm(false);
        }

        //keep lastLoopMicros updated so it never accumulates catch-up bursts when re-armed
        lastLoopMicros = micros();

        if (g_cfg.runMode == MODE_BENCH_DIAG) {
            if (millis() - lastPrintMillis >= 150) {
                lastPrintMillis = millis();
                runBenchDiagnostics();
            }
        }
        return;
    }

    //robot is armed!
    wasArmed = true;

    //bench diagnostic mode branch (motors held at 0)
    if (g_cfg.runMode == MODE_BENCH_DIAG) {
        motorsArm(false);
        coastStop();
        if (millis() - lastPrintMillis >= 150) {
            lastPrintMillis = millis();
            runBenchDiagnostics();
        }
        return;
    }

    //open-loop motor test branch
    if (g_cfg.runMode == MODE_MOTOR_TEST) {
        motorsArm(true);
        runMotorTest();
        return;
    }

    //manual wasd rc mode branch
    if (g_cfg.runMode == MODE_MANUAL_WASD) {
        //deadman watchdog: if no manual packet received for 250ms, stop motors
        if (millis() - g_lastManualCmdMs > MANUAL_DEADMAN_MS) {
            coastStop();
        } else {
            motorsArm(true);
            drive(g_manualLeft, g_manualRight);
        }

        if (millis() - lastPrintMillis >= 100) {
            lastPrintMillis = millis();
            printSensorMap();
            Serial.printf("| [WASD] L_PWM: %4d  R_PWM: %4d  L_enc: %5ld  R_enc: %5ld\n",
                          getLastLeftPwm(), getLastRightPwm(),
                          (long)getLeftTicks(), (long)getRightTicks());
        }
        return;
    }

    //autonomous track mode branch
    motorsArm(true);
    uint32_t currentMicros = micros();

    //deterministic 500hz control loop
    if (currentMicros - lastLoopMicros >= LOOP_INTERVAL_US) {
        //guard against catch-up burst if stalled
        if (currentMicros - lastLoopMicros > 50000) {
            lastLoopMicros = currentMicros;
        } else {
            lastLoopMicros += LOOP_INTERVAL_US;
        }

        //advance state machine and motor outputs
        navigatorUpdate();

        //auto-disarm when finish box reached
        if (getNavState() == STATE_STOPPED) {
            g_armed = false;
        }
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
