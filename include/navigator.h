#pragma once
#include <Arduino.h>
#include "protocol.h"

enum TurnDir {
    DIR_NONE,
    DIR_LEFT,
    DIR_RIGHT
};

void navigatorInit(bool skipStartExit = false);
void navigatorUpdate();
NavState getNavState();
const char* getNavStateName();
bool isNavigatorInverted();
