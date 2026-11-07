#include "encoders.h"
#include "config.h"
#include <hal/gpio_ll.h>
#include <esp_cpu.h>

//atomic access lock
static portMUX_TYPE encMux = portMUX_INITIALIZER_UNLOCKED;

static volatile int32_t leftTicks = 0;
static volatile int32_t rightTicks = 0;
static volatile uint32_t lastLeftCycles = 0;
static volatile uint32_t lastRightCycles = 0;
static constexpr uint32_t MIN_EDGE_CYCLES = 2400; //10us at 240mhz rejects brush emi

//left motor encoder isr
static void IRAM_ATTR leftEncoderISR() {
    uint32_t now = esp_cpu_get_cycle_count();
    if (now - lastLeftCycles < MIN_EDGE_CYCLES) return;
    lastLeftCycles = now;

    //direct register read from iram avoiding flash cache calls
    int a = gpio_ll_get_level(&GPIO, PIN_ENC_LA);
    int b = gpio_ll_get_level(&GPIO, PIN_ENC_LB);
    if (a == b) {
        leftTicks = leftTicks + 1;
    } else {
        leftTicks = leftTicks - 1;
    }
}

//right motor encoder isr
static void IRAM_ATTR rightEncoderISR() {
    uint32_t now = esp_cpu_get_cycle_count();
    if (now - lastRightCycles < MIN_EDGE_CYCLES) return;
    lastRightCycles = now;

    //direct register read from iram avoiding flash cache calls
    int a = gpio_ll_get_level(&GPIO, PIN_ENC_RA);
    int b = gpio_ll_get_level(&GPIO, PIN_ENC_RB);
    if (a != b) {
        rightTicks = rightTicks + 1;
    } else {
        rightTicks = rightTicks - 1;
    }
}

void encodersInit() {
    //left encoder inputs
    pinMode(PIN_ENC_LA, INPUT_PULLUP);
    pinMode(PIN_ENC_LB, INPUT_PULLUP);

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
    return INVERT_LEFT_ENC ? -val : val;
}

int32_t getRightTicks() {
    portENTER_CRITICAL(&encMux);
    int32_t val = rightTicks;
    portEXIT_CRITICAL(&encMux);
    return INVERT_RIGHT_ENC ? -val : val;
}

int32_t getAvgTicks() {
    return (getLeftTicks() + getRightTicks()) / 2;
}

float ticksToCm(int32_t ticks) {
    return static_cast<float>(ticks) / TICKS_PER_CM;
}

int32_t cmToTicks(float cm) {
    return static_cast<int32_t>(cm * TICKS_PER_CM);
}
