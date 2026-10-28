#pragma once
#include <Arduino.h>

enum NavState {
    STATE_START_EXIT,
    STATE_NORMAL_PID,
    STATE_INVERTED_PID,
    STATE_SHARP_TURN,
    STATE_GAP_OR_DEADEND,
    STATE_UTURN_180,
    STATE_ALL_BLACK_CHECK,
    STATE_STOPPED
};

enum TurnDir {
    DIR_NONE,
    DIR_LEFT,
    DIR_RIGHT
};

void navigatorInit();
void navigatorUpdate();
NavState getNavState();
const char* getNavStateName();
