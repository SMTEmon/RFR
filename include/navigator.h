#pragma once
#include <Arduino.h>
#include "protocol.h"

enum TurnDir {
    DIR_NONE,
    DIR_LEFT,
    DIR_RIGHT
};

enum JunctionPolicy : uint8_t {
    JUNCTION_LOOP_STRAIGHT = 0,
    JUNCTION_LHR           = 1,
    JUNCTION_RHR           = 2
};

void navigatorInit(bool skipStartExit = false);
void navigatorUpdate();
NavState getNavState();
const char* getNavStateName();
bool isNavigatorInverted();
