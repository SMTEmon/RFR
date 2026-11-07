#include "sensors.h"

//pack select pins into array
static const int selectPins[4] = {PIN_MUX_S0, PIN_MUX_S1, PIN_MUX_S2, PIN_MUX_S3};

static uint16_t rawMin[NUM_SENSORS];
static uint16_t rawMax[NUM_SENSORS];
static uint16_t sensorRaw[NUM_SENSORS];
static int16_t sensorNormDark[NUM_SENSORS];
static int16_t sensorVal[NUM_SENSORS];
static bool sensorBlack[NUM_SENSORS];
static uint8_t sensorSens[NUM_SENSORS];
static int32_t lastError = 0;

static int16_t threshEnter = THRESH_BLACK_ENTER;
static int16_t threshExit  = THRESH_BLACK_EXIT;

static bool calibrating = false;
static uint32_t calibStartTime = 0;
static uint8_t lastFrameLift = 0;
static uint32_t airLiftStartTime = 0;
static bool airLiftActive = false;

//mux helper functions
static void initMux(const int pins[4]) {
    for (int i = 0; i < 4; i++) {
        pinMode(pins[i], OUTPUT);
        digitalWrite(pins[i], LOW);
    }
    //note: do not call pinMode on analog pin in core 3.x (blinds adc)
}

static void setMuxAddress(const int pins[4], uint8_t channel) {
    for (int bit = 0; bit < 4; bit++) {
        digitalWrite(pins[bit], bitRead(channel, bit));
    }
}

static uint16_t readSensorChannel(const int pins[4], int signalPin, uint8_t channel) {
    setMuxAddress(pins, channel);

    //let mux and phototransistor rc line stabilize
    delayMicroseconds(MUX_SETTLE_US);

    //dummy read to discharge/precharge esp32 adc internal capacitor
    analogRead(signalPin);
    delayMicroseconds(5);

    //actual stable reading
    return static_cast<uint16_t>(analogRead(signalPin));
}

//core sensor api
void sensorsInit() {
    initMux(selectPins);
    analogReadResolution(12);
    analogSetPinAttenuation(PIN_MUX_SIG, ADC_11db);

    //initialize default calibration range to bench defaults
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        rawMin[i] = DEFAULT_RAW_MIN;
        rawMax[i] = DEFAULT_RAW_MAX;
        sensorRaw[i] = 0;
        sensorNormDark[i] = 0;
        sensorVal[i] = 0;
        sensorBlack[i] = false;
        sensorSens[i] = DEFAULT_SENSOR_SENS[i];
    }
}

void setSensorThreshold(int16_t enterThresh) {
    threshEnter = constrain(enterThresh, 150, 950);
    threshExit  = constrain(enterThresh - 150, 50, 850); //150 units schmitt hysteresis
}

int16_t getSensorThreshold() {
    return threshEnter;
}

void setSensorSensitivity(uint8_t idx, uint8_t sensPct) {
    if (idx < NUM_SENSORS) {
        sensorSens[idx] = constrain(sensPct, 0, 180);
    }
}

void setAllSensorSensitivities(const uint8_t sens[NUM_SENSORS]) {
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        sensorSens[i] = constrain(sens[i], 0, 180);
    }
}

uint8_t getSensorSensitivity(uint8_t idx) {
    if (idx >= NUM_SENSORS) return 100;
    return sensorSens[idx];
}

uint8_t getLastFrameLift() {
    return lastFrameLift;
}

bool isRobotLifted() {
    return airLiftActive;
}

void resetAirLiftDetector() {
    airLiftStartTime = 0;
    airLiftActive = false;
}

void sensorsResetCalibrationMinMax() {
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        rawMin[i] = 4095;
        rawMax[i] = 0;
    }
}

void sensorsSampleCalibrationStep() {
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        uint16_t raw = sensorRaw[i];
        if (raw < rawMin[i]) rawMin[i] = raw;
        if (raw > rawMax[i]) rawMax[i] = raw;
    }
}

void sensorsStartCalibration() {
    calibrating = true;
    calibStartTime = millis();
    pinMode(PIN_LED, OUTPUT);
    sensorsResetCalibrationMinMax();
}

bool sensorsUpdateCalibration() {
    if (!calibrating) return false;

    //blink led at 10hz during sweep
    digitalWrite(PIN_LED, ((millis() / 50) % 2) == 0 ? HIGH : LOW);

    sensorsSampleCalibrationStep();

    //check if 5s sweep completed
    if (millis() - calibStartTime >= CALIB_SWEEP_MS) {
        calibrating = false;
        digitalWrite(PIN_LED, LOW);
        sensorsFinalizeCalibration();
        return true;
    }
    return false;
}

void sensorsFinalizeCalibration() {
    //guard against channels that were not swept over black and white
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        if (rawMax[i] <= rawMin[i] + 600) {
            rawMin[i] = DEFAULT_RAW_MIN;
            rawMax[i] = DEFAULT_RAW_MAX;
        }
    }
}

void sensorsResetCalibrationDefault() {
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        rawMin[i] = DEFAULT_RAW_MIN;
        rawMax[i] = DEFAULT_RAW_MAX;
    }
}

bool isSensorCalibrating() {
    return calibrating;
}

void sensorsCalibrate() {
    //skip sweep during bench wiring tests
    if (SKIP_CALIBRATION) {
        sensorsResetCalibrationDefault();
        return;
    }

    pinMode(PIN_LED, OUTPUT);
    uint32_t startTime = millis();

    //phase 1: 5 second hand sweep window
    while (millis() - startTime < CALIB_SWEEP_MS) {
        //blink led at 10hz to give visual pace
        digitalWrite(PIN_LED, ((millis() / 50) % 2) == 0 ? HIGH : LOW);

        for (uint8_t i = 0; i < NUM_SENSORS; i++) {
            uint8_t ch = logicalToMux(i);
            uint16_t raw = readSensorChannel(selectPins, PIN_MUX_SIG, ch);

            if (raw < rawMin[i]) rawMin[i] = raw;
            if (raw > rawMax[i]) rawMax[i] = raw;
        }
        delay(2);
    }

    //phase 2: 2 second steady pause to set bot inside start box
    digitalWrite(PIN_LED, HIGH);
    delay(CALIB_PLACE_MS);
    digitalWrite(PIN_LED, LOW);
}

void sensorsRead(bool isInverted) {
    uint8_t darkCount = 0;

    //first pass: read raw adc and compute un-equalized norm
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        uint8_t ch = logicalToMux(i);
        uint16_t raw = readSensorChannel(selectPins, PIN_MUX_SIG, ch);
        sensorRaw[i] = raw;

        if (sensorSens[i] == 0) {
            sensorNormDark[i] = 0;
            sensorVal[i] = 0;
            sensorBlack[i] = false;
            continue;
        }

        uint16_t rmin = rawMin[i];
        uint16_t rmax = rawMax[i];
        if (rmax <= rmin + 50) {
            rmax = rmin + 50;
        }

        int32_t clamped = constrain(raw, rmin, rmax);
        int16_t norm;
        if (ADC_HIGH_IS_BLACK) {
            norm = (clamped - rmin) * 1000 / (rmax - rmin);
        } else {
            norm = (rmax - clamped) * 1000 / (rmax - rmin);
        }
        sensorNormDark[i] = norm;
        if (norm > 750) darkCount++;
    }

    uint8_t rawAirCount = 0;
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        if (sensorRaw[i] > 3600) rawAirCount++;
    }

    //airborne detector: if 15+ sensors see open-air dark (>3600 raw) continuously for >800ms
    if (rawAirCount >= 15 && darkCount >= 15) {
        if (airLiftStartTime == 0) airLiftStartTime = millis();
        if (millis() - airLiftStartTime > 800) {
            airLiftActive = true;
        }
    } else {
        airLiftStartTime = 0;
        airLiftActive = false;
    }

    //common-mode floor baseline rejection:
    //find lowest and 2nd lowest values among front row/bevels (S02..S13)
    int16_t min1 = 1000;
    int16_t min2 = 1000;
    for (uint8_t i = 2; i <= 13; i++) {
        if (sensorSens[i] == 0) continue;
        int16_t val = sensorNormDark[i];
        if (val < min1) {
            min2 = min1;
            min1 = val;
        } else if (val < min2) {
            min2 = val;
        }
    }

    //min2 represents the ambient floor reflection level across the sensor bar
    //if chassis pitches up during acceleration, min2 jumps up
    int16_t floorLift = 0;
    if (min2 < 420 && min2 > 40) {
        //floor has lifted due to acceleration pitch-up or room glare
        floorLift = min2 - 40;
    }
    lastFrameLift = static_cast<uint8_t>(constrain(floorLift, 0, 255));

    //second pass: equalize analog curve and evaluate per-sensor schmitt trigger
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        if (sensorSens[i] == 0) continue;

        int16_t norm = sensorNormDark[i];

        //subtract common-mode pitch-up lift for trigger evaluation
        int16_t trigNorm = max<int16_t>(0, norm - floorLift);

        //compute per-channel effective threshold
        int16_t enterI = constrain(threshEnter - (static_cast<int16_t>(sensorSens[i]) - 100) * 5, 80, 940);
        int16_t hystI  = min<int16_t>(120, enterI / 2);
        int16_t exitI  = max<int16_t>(40, enterI - hystI);

        //equalize analog curve around enterI -> 500 for balanced pid centroid
        int16_t eqDark;
        if (norm <= enterI) {
            eqDark = static_cast<int32_t>(norm) * 500 / enterI;
        } else {
            eqDark = 500 + static_cast<int32_t>(norm - enterI) * 500 / (1000 - enterI);
        }

        sensorVal[i] = isInverted ? (1000 - eqDark) : eqDark;

        //per-channel schmitt trigger update using floor-lift-compensated reading
        if (!isInverted) {
            if (trigNorm >= enterI) {
                sensorBlack[i] = true;
            } else if (trigNorm <= exitI) {
                sensorBlack[i] = false;
            }
        } else {
            if (trigNorm <= exitI) {
                sensorBlack[i] = true;
            } else if (trigNorm >= enterI) {
                sensorBlack[i] = false;
            }
        }
    }
}

int16_t getSensorValue(uint8_t idx) {
    if (idx >= NUM_SENSORS) return 0;
    return sensorVal[idx];
}

int16_t getSensorNormalizedDark(uint8_t idx) {
    if (idx >= NUM_SENSORS) return 0;
    return sensorNormDark[idx];
}

uint16_t getSensorRaw(uint8_t idx) {
    if (idx >= NUM_SENSORS) return 0;
    return sensorRaw[idx];
}

bool isSensorBlack(uint8_t idx) {
    if (idx >= NUM_SENSORS) return false;
    return sensorBlack[idx];
}

bool hasLine() {
    //check if line is detected anywhere in front row or bevels
    for (uint8_t i = 2; i <= 13; i++) {
        if (sensorBlack[i]) return true;
    }
    return false;
}

bool isAllWhite() {
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        if (sensorBlack[i]) return false;
    }
    return true;
}

bool isAllBlack() {
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        if (!sensorBlack[i]) return false;
    }
    return true;
}

bool isFrontAllBlack() {
    //front 8 row 4..11
    for (uint8_t i = 4; i <= 11; i++) {
        if (!sensorBlack[i]) return false;
    }
    return true;
}

bool hasLeftWing() {
    return sensorBlack[0] || sensorBlack[1];
}

bool hasRightWing() {
    return sensorBlack[14] || sensorBlack[15];
}

bool hasLeftBevel() {
    return sensorBlack[2] || sensorBlack[3];
}

bool hasRightBevel() {
    return sensorBlack[12] || sensorBlack[13];
}

bool isInvertedPattern() {
    //outer front black while center white
    bool outerLeftBlack  = sensorBlack[4] || sensorBlack[5];
    bool outerRightBlack = sensorBlack[10] || sensorBlack[11];
    bool centerWhite     = (!sensorBlack[7]) && (!sensorBlack[8]);
    return outerLeftBlack && outerRightBlack && centerWhite;
}

int32_t computePositionError(bool isInverted) {
    int64_t weightedSum = 0;
    int32_t totalWeight = 0;

    if (!isInverted) {
        //normal mode: weighted centroid across 12 sensors (idx 2..13)
        //subtract white-floor baseline noise (CENTROID_NOISE_FLOOR)
        for (uint8_t i = 2; i <= 13; i++) {
            int16_t val = sensorVal[i];
            int16_t cleanVal = val > CENTROID_NOISE_FLOOR ? (val - CENTROID_NOISE_FLOOR) : 0;
            weightedSum += static_cast<int64_t>(cleanVal) * PID_WEIGHTS_12[i - 2];
            totalWeight += cleanVal;
        }
    } else {
        //inverted mode: strictly across 8 front sensors (idx 4..11)
        for (uint8_t i = 4; i <= 11; i++) {
            int16_t val = sensorVal[i];
            int16_t cleanVal = val > CENTROID_NOISE_FLOOR ? (val - CENTROID_NOISE_FLOOR) : 0;
            weightedSum += static_cast<int64_t>(cleanVal) * PID_WEIGHTS_8[i - 4];
            totalWeight += cleanVal;
        }
    }

    if (totalWeight < 60) {
        //loss of line: keep last known error sign to guide recovery
        return lastError > 0 ? 5500 : -5500;
    }

    lastError = static_cast<int32_t>(weightedSum / totalWeight);
    return lastError;
}
