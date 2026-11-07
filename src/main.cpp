#include <Arduino.h>
#include "config.h"
#include "motors.h"
#include "encoders.h"
#include "sensors.h"
#include "navigator.h"
#include "telemetry.h"
#include "autotune.h"

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
    Serial.setTxBufferSize(1024);
    Serial.begin(115200);

    //setup motors with stby low during boot
    motorsInit();

    //setup encoders and sensors
    encodersInit();
    sensorsInit();

    //setup esp-now wireless telemetry and nvs config
    telemetryInit();

    //initialize autotune & mapping engine
    autotuneInit();

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

    //handle disarmed and active stop transition
    if (!g_armed) {
        if (wasArmed) {
            wasArmed = false;
            stopBrakeTimer = millis();
            motorsArm(true);
            drive(0, 0); //gentle envelope ramp down
        } else if (stopBrakeTimer > 0 && millis() - stopBrakeTimer < STOP_RAMP_MS) {
            motorsArm(true);
            drive(0, 0);
        } else {
            stopBrakeTimer = 0;
            coastStop();
            motorsArm(false);
        }

        //read sensors at 500hz while disarmed so dashboard adc bars never freeze
        uint32_t currentMicros = micros();
        if (currentMicros - lastLoopMicros >= LOOP_INTERVAL_US) {
            if (currentMicros - lastLoopMicros > 50000) {
                lastLoopMicros = currentMicros;
            } else {
                lastLoopMicros += LOOP_INTERVAL_US;
            }
            sensorsRead(false);
        }

        if (g_cfg.runMode == MODE_BENCH_DIAG) {
            if (millis() - lastPrintMillis >= 150) {
                lastPrintMillis = millis();
                runBenchDiagnostics();
            }
        }
        return;
    }

    //robot is armed!
    if (!wasArmed) {
        wasArmed = true;
        resetAirLiftDetector();
    }

    //bench diagnostic mode branch (motors held at 0)
    if (g_cfg.runMode == MODE_BENCH_DIAG) {
        motorsArm(false);
        coastStop();
        sensorsRead(false);
        if (millis() - lastPrintMillis >= 150) {
            lastPrintMillis = millis();
            runBenchDiagnostics();
        }
        return;
    }

    //open-loop motor test branch
    if (g_cfg.runMode == MODE_MOTOR_TEST) {
        motorsArm(true);
        sensorsRead(false);
        runMotorTest();
        return;
    }

    //unified 500hz (2ms) control loop for all driving modes
    uint32_t currentMicros = micros();
    if (currentMicros - lastLoopMicros < LOOP_INTERVAL_US) {
        return;
    }
    if (currentMicros - lastLoopMicros > 50000) {
        lastLoopMicros = currentMicros;
    } else {
        lastLoopMicros += LOOP_INTERVAL_US;
    }

    //read sensors exactly once per 2ms tick
    sensorsRead(isNavigatorInverted());

    //manual wasd rc mode branch
    if (g_cfg.runMode == MODE_MANUAL_WASD) {
        if (millis() - g_lastManualCmdMs > MANUAL_DEADMAN_MS) {
            coastStop();
        } else {
            motorsArm(true);
            drive(g_manualLeft, g_manualRight);
        }
        return;
    }

    //track mapping & progressive auto-tune mode branch
    if (g_cfg.runMode == MODE_MAP_AND_TUNE) {
        autotuneUpdate();
        if (getTuneState() == TUNE_WAIT_VALIDATE || getTuneState() == TUNE_COMPLETE || getTuneState() == TUNE_SAFE_ABORT) {
            g_armed = false;
        }
        return;
    }

    //mapped race mode branch (segment speed limits + gain scheduling)
    if (g_cfg.runMode == MODE_MAPPED_RACE) {
        motorsArm(true);
        mappedRaceUpdate();
        return;
    }

    //standard autonomous track mode branch
    motorsArm(true);
    navigatorUpdate();

    if (getNavState() == STATE_STOPPED) {
        g_armed = false;
    }
}
