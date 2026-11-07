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

//differential spin tracking variables
static int32_t spinStartLeft = 0;
static int32_t spinStartRight = 0;
static uint32_t spinStartTimeMs = 0;

//state baseline encoder values for straight heading hold
static int32_t straightStartLeft = 0;
static int32_t straightStartRight = 0;

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
    straightStartLeft = 0;
    straightStartRight = 0;
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
    return navStateToStr(currentState);
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
            //blind straight exit out of start box using delta encoders
            int16_t diff = calcStraightHoldDiff(leftEnc, rightEnc, straightStartLeft, straightStartRight, g_cfg.encKp, maxSpd);
            int16_t spd = constrain(g_cfg.baseSpeed, 0, maxSpd);
            drive(spd - diff, spd + diff);

            //transition once out of solid black box and line is centered
            if (currentTicks >= START_EXIT_TICKS && hasLine() && !isAllBlack()) {
                currentState = STATE_NORMAL_PID;
            }
            break;
        }

        case STATE_NORMAL_PID: {
            //3-frame debounced wing check
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

            //check for entry into inverted zone
            if (g_cfg.enableInverted && isInvertedPattern()) {
                invertedActive = true;
                currentState = STATE_INVERTED_PID;
                break;
            }

            //check all black trigger
            if (isFrontAllBlack()) {
                //in loop straight-priority mode, check if center line continues straight
                if (g_cfg.junctionMode == JUNCTION_LOOP_STRAIGHT && (isSensorBlack(7) || isSensorBlack(8))) {
                    //crossing intersection: continue tracking straight without stopping
                } else {
                    checkStartTicks = currentTicks;
                    straightStartLeft = leftEnc;
                    straightStartRight = rightEnc;
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
                straightStartLeft = leftEnc;
                straightStartRight = rightEnc;
                currentState = STATE_GAP_OR_DEADEND;
                break;
            }

            //normal pid tracking
            int32_t error = computePositionError(false);
            int32_t dErrRaw = constrain(error - prevError, -1500, 1500);
            float rawDError = static_cast<float>(dErrRaw) / (LOOP_INTERVAL_US * 1e-6f);
            prevError = error;

            //exponential low pass filter on dError to stop motor chatter
            filteredDError = 0.6f * filteredDError + 0.4f * rawDError;

            int16_t correction = static_cast<int16_t>(g_cfg.pidKp * error + g_cfg.pidKd * filteredDError * 0.001f);

            //slow down on sharp curves
            int16_t speed = g_cfg.baseSpeed;
            if (abs(error) > 2000) {
                speed = g_cfg.minSpeed;
            }

            //clamp forward line tracking outputs to [0, maxSpd] to prevent inner wheel reverse plug braking
            int16_t leftOut  = constrain(speed + correction, 0, maxSpd);
            int16_t rightOut = constrain(speed - correction, 0, maxSpd);
            drive(leftOut, rightOut);
            break;
        }

        case STATE_INVERTED_PID: {
            //check exit condition
            if (!isSensorBlack(4) && !isSensorBlack(5) && !isSensorBlack(10) && !isSensorBlack(11) && hasLine()) {
                invertedActive = false;
                currentState = STATE_NORMAL_PID;
                break;
            }

            int32_t error = computePositionError(true);
            int32_t dErrRaw = constrain(error - prevError, -1500, 1500);
            float rawDError = static_cast<float>(dErrRaw) / (LOOP_INTERVAL_US * 1e-6f);
            prevError = error;
            filteredDError = 0.6f * filteredDError + 0.4f * rawDError;

            int16_t correction = static_cast<int16_t>(g_cfg.pidKp * error + g_cfg.pidKd * filteredDError * 0.001f);
            int16_t leftOut  = constrain(g_cfg.baseSpeed + correction, 0, maxSpd);
            int16_t rightOut = constrain(g_cfg.baseSpeed - correction, 0, maxSpd);
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

            //anti-infinite-spin failsafe
            if (spinDeg >= MAX_SHARP_SPIN_DEG || elapsedMs > MAX_SPIN_TIMEOUT_MS) {
                drive(0, 0);
                latchedDir = DIR_NONE;
                currentState = STATE_STOPPED;
            }
            break;
        }

        case STATE_GAP_OR_DEADEND: {
            //hold straight heading using delta encoders from state entry
            int16_t diff = calcStraightHoldDiff(leftEnc, rightEnc, straightStartLeft, straightStartRight, g_cfg.encKp, maxSpd);
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

            //anti-infinite-spin failsafe
            if (spinDeg >= UTURN_MAX_SPIN_DEG || elapsedMs > MAX_SPIN_TIMEOUT_MS) {
                drive(0, 0);
                latchedDir = DIR_NONE;
                currentState = STATE_STOPPED;
            }
            break;
        }

        case STATE_ALL_BLACK_CHECK: {
            int16_t diff = calcStraightHoldDiff(leftEnc, rightEnc, straightStartLeft, straightStartRight, g_cfg.encKp, maxSpd);
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
                    straightStartLeft = leftEnc;
                    straightStartRight = rightEnc;
                    gapStartTicks = currentTicks;
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
