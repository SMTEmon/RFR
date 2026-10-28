#include "sensors.h"

static uint16_t rawMin[NUM_SENSORS];
static uint16_t rawMax[NUM_SENSORS];
static int16_t sensorVal[NUM_SENSORS];
static bool sensorBlack[NUM_SENSORS];
static int32_t lastError = 0;

static void setMuxChannel(uint8_t ch) {
    digitalWrite(PIN_MUX_S0, (ch & 0x01) ? HIGH : LOW);
    digitalWrite(PIN_MUX_S1, (ch & 0x02) ? HIGH : LOW);
    digitalWrite(PIN_MUX_S2, (ch & 0x04) ? HIGH : LOW);
    digitalWrite(PIN_MUX_S3, (ch & 0x08) ? HIGH : LOW);
}

void sensorsInit() {
    //setup mux address lines
    pinMode(PIN_MUX_S0, OUTPUT);
    pinMode(PIN_MUX_S1, OUTPUT);
    pinMode(PIN_MUX_S2, OUTPUT);
    pinMode(PIN_MUX_S3, OUTPUT);

    //setup analog input pin
    pinMode(PIN_MUX_SIG, INPUT);
    analogReadResolution(12);

    //initialize default calibration range
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        rawMin[i] = 4095;
        rawMax[i] = 0;
        sensorVal[i] = 0;
        sensorBlack[i] = false;
    }
}

void sensorsCalibrate() {
    //skip sweep during bench wiring tests
    if (SKIP_CALIBRATION) {
        for (uint8_t i = 0; i < NUM_SENSORS; i++) {
            rawMin[i] = DEFAULT_RAW_MIN;
            rawMax[i] = DEFAULT_RAW_MAX;
        }
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
            setMuxChannel(ch);
            delayMicroseconds(MUX_SETTLE_US);
            uint16_t raw = analogRead(PIN_MUX_SIG);

            if (raw < rawMin[i]) rawMin[i] = raw;
            if (raw > rawMax[i]) rawMax[i] = raw;
        }
        delay(2);
    }

    //phase 2: 2 second steady pause to set bot inside #31
    digitalWrite(PIN_LED, HIGH);
    delay(CALIB_PLACE_MS);
    digitalWrite(PIN_LED, LOW);
}

void sensorsRead(bool isInverted) {
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        uint8_t ch = logicalToMux(i);
        setMuxChannel(ch);
        delayMicroseconds(MUX_SETTLE_US);
        uint16_t raw = analogRead(PIN_MUX_SIG);

        //guard against zero span
        uint16_t rmin = rawMin[i];
        uint16_t rmax = rawMax[i];
        if (rmax <= rmin + 50) {
            rmax = rmin + 50;
        }

        //normalize 0 (white) to 1000 (black)
        int32_t clamped = constrain(raw, rmin, rmax);
        int16_t norm = (clamped - rmin) * 1000 / (rmax - rmin);

        //invert polarity if currently inside inverted zone
        if (isInverted) {
            sensorVal[i] = 1000 - norm;
        } else {
            sensorVal[i] = norm;
        }

        //schmitt trigger update
        if (sensorVal[i] >= THRESH_BLACK_ENTER) {
            sensorBlack[i] = true;
        } else if (sensorVal[i] <= THRESH_BLACK_EXIT) {
            sensorBlack[i] = false;
        }
    }
}

int16_t getSensorValue(uint8_t idx) {
    if (idx >= NUM_SENSORS) return 0;
    return sensorVal[idx];
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
        for (uint8_t i = 2; i <= 13; i++) {
            int16_t val = sensorVal[i];
            weightedSum += static_cast<int64_t>(val) * PID_WEIGHTS_12[i - 2];
            totalWeight += val;
        }
    } else {
        //inverted mode: strictly across 8 front sensors (idx 4..11)
        for (uint8_t i = 4; i <= 11; i++) {
            int16_t val = sensorVal[i];
            weightedSum += static_cast<int64_t>(val) * PID_WEIGHTS_8[i - 4];
            totalWeight += val;
        }
    }

    if (totalWeight < 100) {
        //loss of line: keep last known error sign to guide recovery
        return lastError > 0 ? 5500 : -5500;
    }

    lastError = static_cast<int32_t>(weightedSum / totalWeight);
    return lastError;
}

void printSensorRaw() {
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        uint8_t ch = logicalToMux(i);
        setMuxChannel(ch);
        delayMicroseconds(MUX_SETTLE_US);
        Serial.print(analogRead(PIN_MUX_SIG));
        Serial.print(i == NUM_SENSORS - 1 ? "\n" : "\t");
    }
}

void printSensorCalibrated() {
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        Serial.print(sensorVal[i]);
        Serial.print(i == NUM_SENSORS - 1 ? "\n" : "\t");
    }
}
