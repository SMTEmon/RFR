#include "encoders.h"
#include "config.h"

//atomic access lock
static portMUX_TYPE encMux = portMUX_INITIALIZER_UNLOCKED;

static volatile int32_t leftTicks = 0;
static volatile int32_t rightTicks = 0;

//left motor encoder isr
static void IRAM_ATTR leftEncoderISR() {
    //read phase b to determine direction
    int a = digitalRead(PIN_ENC_LA);
    int b = digitalRead(PIN_ENC_LB);
    if (a == b) {
        leftTicks = leftTicks + 1;
    } else {
        leftTicks = leftTicks - 1;
    }
}

//right motor encoder isr
static void IRAM_ATTR rightEncoderISR() {
    int a = digitalRead(PIN_ENC_RA);
    int b = digitalRead(PIN_ENC_RB);
    //phase polarity for forward roll
    if (a != b) {
        rightTicks = rightTicks + 1;
    } else {
        rightTicks = rightTicks - 1;
    }
}

void encodersInit() {
    //left encoder inputs (pin 35 has no internal pullup)
    pinMode(PIN_ENC_LA, INPUT_PULLUP);
    pinMode(PIN_ENC_LB, INPUT);

    //right encoder inputs
    pinMode(PIN_ENC_RA, INPUT_PULLUP);
    pinMode(PIN_ENC_RB, INPUT_PULLUP);

    //attach interrupts on phase a change
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_LA), leftEncoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_RA), rightEncoderISR, CHANGE);

    resetEncoders();
}

void resetEncoders() {
    portENTER_CRITICAL(&encMux);
    leftTicks = 0;
    rightTicks = 0;
    portEXIT_CRITICAL(&encMux);
}

int32_t getLeftTicks() {
    portENTER_CRITICAL(&encMux);
    int32_t val = leftTicks;
    portEXIT_CRITICAL(&encMux);
    return val;
}

int32_t getRightTicks() {
    portENTER_CRITICAL(&encMux);
    int32_t val = rightTicks;
    portEXIT_CRITICAL(&encMux);
    return val;
}

int32_t getAvgTicks() {
    portENTER_CRITICAL(&encMux);
    int32_t val = (leftTicks + rightTicks) / 2;
    portEXIT_CRITICAL(&encMux);
    return val;
}

float ticksToCm(int32_t ticks) {
    return static_cast<float>(ticks) / TICKS_PER_CM;
}

int32_t cmToTicks(float cm) {
    return static_cast<int32_t>(cm * TICKS_PER_CM);
}
