#include "navigator.h"
#include "config.h"
#include "sensors.h"
#include "motors.h"
#include "encoders.h"
#include "telemetry.h"

static NavState currentState = STATE_START_EXIT;
static TurnDir latchedDir = DIR_NONE;
static int32_t latchStartTicks = 0;
static int32_t gapStartTicks = 0;
static int32_t checkStartTicks = 0;
static int32_t prevError = 0;
static float filteredDError = 0.0f;
static bool invertedActive = false;

//differential spin tracking variables (solves infinite spinning)
static int32_t spinStartLeft = 0;
static int32_t spinStartRight = 0;
static uint32_t spinStartTimeMs = 0;

//wing latch debouncers
static uint8_t leftWingCount = 0;
static uint8_t rightWingCount = 0;

void navigatorInit(bool skipStartExit) {
    currentState = skipStartExit ? STATE_NORMAL_PID : STATE_START_EXIT;
    latchedDir = DIR_NONE;
    prevError = 0;
    filteredDError = 0.0f;
    invertedActive = false;
    spinStartLeft = 0;
    spinStartRight = 0;
    spinStartTimeMs = 0;
    leftWingCount = 0;
    rightWingCount = 0;
    resetEncoders();
}

NavState getNavState() {
    return currentState;
}

bool isNavigatorInverted() {
    return invertedActive;
}

const char* getNavStateName() {
    switch (currentState) {
        case STATE_START_EXIT:      return "START_EXIT";
        case STATE_NORMAL_PID:     return "NORMAL_PID";
        case STATE_INVERTED_PID:   return "INVERTED_PID";
        case STATE_SHARP_TURN:     return "SHARP_TURN";
        case STATE_GAP_OR_DEADEND: return "GAP_OR_DEADEND";
        case STATE_UTURN_180:      return "UTURN_180";
        case STATE_ALL_BLACK_CHECK:return "ALL_BLACK_CHECK";
        case STATE_STOPPED:        return "STOPPED";
        case STATE_CALIBRATING:    return "CALIBRATING";
        default:                   return "UNKNOWN";
    }
}

void navigatorUpdate() {
    int32_t currentTicks = getAvgTicks();
    int32_t leftEnc = getLeftTicks();
    int32_t rightEnc = getRightTicks();
    int16_t maxSpd = (g_cfg.maxSpeed > 0 && g_cfg.maxSpeed <= 255) ? g_cfg.maxSpeed : 255;

    //failsafe: if robot was picked up off ground, stop immediately
    if (isRobotLifted()) {
        drive(0, 0);
        currentState = STATE_STOPPED;
        return;
    }

    switch (currentState) {
        case STATE_START_EXIT: {
            //blind straight exit out of #31 start box
            int16_t diff = static_cast<int16_t>(g_cfg.encKp * (leftEnc - rightEnc));
            int16_t spd = constrain(g_cfg.baseSpeed, 0, maxSpd);
            drive(spd - diff, spd + diff);

            //transition once out of solid black box and line is centered
            if (currentTicks >= START_EXIT_TICKS && hasLine() && !isAllBlack()) {
                currentState = STATE_NORMAL_PID;
            }
            break;
        }

        case STATE_NORMAL_PID: {
            //3-frame debounced wing check (prevents single-frame noise spikes)
            if (hasLeftWing() || hasLeftBevel()) {
                if (leftWingCount < 255) leftWingCount++;
            } else {
                leftWingCount = 0;
            }

            if (hasRightWing() || hasRightBevel()) {
                if (rightWingCount < 255) rightWingCount++;
            } else {
                rightWingCount = 0;
            }

            if (leftWingCount >= 3) {
                latchedDir = DIR_LEFT;
                latchStartTicks = currentTicks;
            } else if (rightWingCount >= 3) {
                if (g_cfg.junctionMode == JUNCTION_RHR || latchedDir != DIR_LEFT) {
                    latchedDir = DIR_RIGHT;
                    latchStartTicks = currentTicks;
                }
            }

            //expire latch after window distance
            if (latchedDir != DIR_NONE && (currentTicks - latchStartTicks > LATCH_WINDOW_TICKS)) {
                latchedDir = DIR_NONE;
            }

            //check for entry into #29 inverted zone
            if (g_cfg.enableInverted && isInvertedPattern()) {
                invertedActive = true;
                currentState = STATE_INVERTED_PID;
                break;
            }

            //check all black trigger (t-line or #36 stop box)
            if (isFrontAllBlack()) {
                //in loop straight-priority mode, check if center line continues straight
                if (g_cfg.junctionMode == JUNCTION_LOOP_STRAIGHT && (isSensorBlack(7) || isSensorBlack(8))) {
                    //crossing intersection: continue tracking straight without stopping
                } else {
                    checkStartTicks = currentTicks;
                    currentState = STATE_ALL_BLACK_CHECK;
                    break;
                }
            }

            //check sharp turn when line ends forward but wing was latched
            if (!hasLine() && latchedDir != DIR_NONE) {
                spinStartLeft = leftEnc;
                spinStartRight = rightEnc;
                spinStartTimeMs = millis();
                currentState = STATE_SHARP_TURN;
                break;
            }

            //check gap or off-line
            if (isAllWhite() && latchedDir == DIR_NONE) {
                gapStartTicks = currentTicks;
                currentState = STATE_GAP_OR_DEADEND;
                break;
            }

            //normal pid tracking
            int32_t error = computePositionError(false);
            float rawDError = static_cast<float>(error - prevError) / (LOOP_INTERVAL_US * 1e-6f);
            prevError = error;

            //exponential low pass filter on dError to stop motor chatter
            filteredDError = 0.6f * filteredDError + 0.4f * rawDError;

            int16_t correction = static_cast<int16_t>(g_cfg.pidKp * error + g_cfg.pidKd * filteredDError * 0.001f);

            //slow down on sharp curves
            int16_t speed = g_cfg.baseSpeed;
            if (abs(error) > 2000) {
                speed = g_cfg.minSpeed;
            }

            int16_t leftOut  = speed + correction;
            int16_t rightOut = speed - correction;
            drive(leftOut, rightOut);
            break;
        }

        case STATE_INVERTED_PID: {
            //check exit condition: outer front sensors see normal white floor while center sees black line
            if (!isSensorBlack(4) && !isSensorBlack(5) && !isSensorBlack(10) && !isSensorBlack(11) && hasLine()) {
                invertedActive = false;
                currentState = STATE_NORMAL_PID;
                break;
            }

            int32_t error = computePositionError(true);
            float rawDError = static_cast<float>(error - prevError) / (LOOP_INTERVAL_US * 1e-6f);
            prevError = error;
            filteredDError = 0.6f * filteredDError + 0.4f * rawDError;

            int16_t correction = static_cast<int16_t>(g_cfg.pidKp * error + g_cfg.pidKd * filteredDError * 0.001f);
            int16_t leftOut  = g_cfg.baseSpeed + correction;
            int16_t rightOut = g_cfg.baseSpeed - correction;
            drive(leftOut, rightOut);
            break;
        }

        case STATE_SHARP_TURN: {
            //calculate actual chassis rotation degrees from wheel differential
            int32_t diffTicks = abs((rightEnc - spinStartRight) - (leftEnc - spinStartLeft));
            float spinDeg = (diffTicks / (TICKS_PER_CM * WHEEL_BASE_CM)) * (180.0f / PI);
            uint32_t elapsedMs = millis() - spinStartTimeMs;

            //execute spin in latched direction using direct drive
            if (latchedDir == DIR_LEFT) {
                driveDirect(-g_cfg.turnSpeed, g_cfg.turnSpeed);
            } else {
                driveDirect(g_cfg.turnSpeed, -g_cfg.turnSpeed);
            }

            //re-lock line on center sensors
            if (hasLine() && (isSensorBlack(6) || isSensorBlack(7) || isSensorBlack(8) || isSensorBlack(9))) {
                latchedDir = DIR_NONE;
                leftWingCount = 0;
                rightWingCount = 0;
                currentState = STATE_NORMAL_PID;
                break;
            }

            //anti-infinite-spin failsafe: if turned >125 deg or took >1400ms without finding line, STOP
            if (spinDeg >= MAX_SHARP_SPIN_DEG || elapsedMs > MAX_SPIN_TIMEOUT_MS) {
                drive(0, 0);
                latchedDir = DIR_NONE;
                currentState = STATE_STOPPED;
            }
            break;
        }

        case STATE_GAP_OR_DEADEND: {
            //hold straight heading using encoders
            int16_t diff = static_cast<int16_t>(g_cfg.encKp * (leftEnc - rightEnc));
            int16_t spd = constrain(g_cfg.minSpeed, 0, maxSpd);
            drive(spd - diff, spd + diff);

            //line found within gap limit
            if (hasLine()) {
                currentState = STATE_NORMAL_PID;
                break;
            }

            //exceeded max gap distance
            if (currentTicks - gapStartTicks >= MAX_GAP_TICKS) {
                if (g_cfg.allowUturn) {
                    spinStartLeft = leftEnc;
                    spinStartRight = rightEnc;
                    spinStartTimeMs = millis();
                    currentState = STATE_UTURN_180;
                } else {
                    //clean safe stop when off the line instead of spinning
                    drive(0, 0);
                    currentState = STATE_STOPPED;
                }
            }
            break;
        }

        case STATE_UTURN_180: {
            //calculate actual chassis rotation degrees from wheel differential
            int32_t diffTicks = abs((rightEnc - spinStartRight) - (leftEnc - spinStartLeft));
            float spinDeg = (diffTicks / (TICKS_PER_CM * WHEEL_BASE_CM)) * (180.0f / PI);
            uint32_t elapsedMs = millis() - spinStartTimeMs;

            //spin left using direct drive
            driveDirect(-g_cfg.turnSpeed, g_cfg.turnSpeed);

            //re-lock line once spun past 165 degrees
            if (hasLine() && spinDeg >= UTURN_TARGET_DEG) {
                latchedDir = DIR_NONE;
                leftWingCount = 0;
                rightWingCount = 0;
                currentState = STATE_NORMAL_PID;
                break;
            }

            //anti-infinite-spin failsafe: if rotated past 230 deg or timed out, stop
            if (spinDeg >= UTURN_MAX_SPIN_DEG || elapsedMs > MAX_SPIN_TIMEOUT_MS) {
                drive(0, 0);
                latchedDir = DIR_NONE;
                currentState = STATE_STOPPED;
            }
            break;
        }

        case STATE_ALL_BLACK_CHECK: {
            int16_t diff = static_cast<int16_t>(g_cfg.encKp * (leftEnc - rightEnc));
            int16_t spd = constrain(g_cfg.minSpeed, 0, maxSpd);
            drive(spd - diff, spd + diff);

            int32_t distance = currentTicks - checkStartTicks;

            if (!isFrontAllBlack() && hasLine()) {
                currentState = STATE_NORMAL_PID;
                break;
            }

            if (distance >= STOP_BOX_TICKS) {
                if (isAllBlack()) {
                    currentState = STATE_STOPPED;
                } else if (hasLine()) {
                    currentState = STATE_NORMAL_PID;
                } else {
                    currentState = STATE_GAP_OR_DEADEND;
                }
                break;
            }
            break;
        }

        case STATE_STOPPED: {
            drive(0, 0);
            break;
        }

        default:
            break;
    }
}
