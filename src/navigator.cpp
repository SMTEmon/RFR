#include "navigator.h"
#include "config.h"
#include "sensors.h"
#include "motors.h"
#include "encoders.h"

static NavState currentState = STATE_START_EXIT;
static TurnDir latchedDir = DIR_NONE;
static int32_t latchStartTicks = 0;
static int32_t gapStartTicks = 0;
static int32_t checkStartTicks = 0;
static int32_t prevError = 0;
static bool invertedActive = false;

void navigatorInit() {
    currentState = STATE_START_EXIT;
    latchedDir = DIR_NONE;
    prevError = 0;
    invertedActive = false;
    resetEncoders();
}

NavState getNavState() {
    return currentState;
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
        default:                   return "UNKNOWN";
    }
}

void navigatorUpdate() {
    //read sensors with current polarity mode
    sensorsRead(invertedActive);

    int32_t currentTicks = getAvgTicks();
    int32_t leftEnc = getLeftTicks();
    int32_t rightEnc = getRightTicks();

    switch (currentState) {
        case STATE_START_EXIT: {
            //blind straight exit out of #31 start box
            int16_t diff = static_cast<int16_t>(ENC_KP * (leftEnc - rightEnc));
            drive(BASE_SPEED - diff, BASE_SPEED + diff);

            //transition once out of solid black box and line is centered
            if (currentTicks >= START_EXIT_TICKS && hasLine() && !isAllBlack()) {
                currentState = STATE_NORMAL_PID;
            }
            break;
        }

        case STATE_NORMAL_PID: {
            //update rolling lhr turn latch
            if (hasLeftWing() || hasLeftBevel()) {
                latchedDir = DIR_LEFT;
                latchStartTicks = currentTicks;
            } else if (hasRightWing() || hasRightBevel()) {
                //left hand rule priority wins ties
                if (latchedDir != DIR_LEFT) {
                    latchedDir = DIR_RIGHT;
                    latchStartTicks = currentTicks;
                }
            }

            //expire latch after window distance
            if (latchedDir != DIR_NONE && (currentTicks - latchStartTicks > LATCH_WINDOW_TICKS)) {
                latchedDir = DIR_NONE;
            }

            //check for entry into #29 inverted zone
            if (isInvertedPattern()) {
                invertedActive = true;
                currentState = STATE_INVERTED_PID;
                break;
            }

            //check all black trigger (t-line or #36 stop box)
            if (isFrontAllBlack()) {
                checkStartTicks = currentTicks;
                currentState = STATE_ALL_BLACK_CHECK;
                break;
            }

            //check sharp turn at apex loss
            if (!hasLine() && latchedDir != DIR_NONE) {
                currentState = STATE_SHARP_TURN;
                break;
            }

            //check gap or dead end
            if (isAllWhite() && latchedDir == DIR_NONE) {
                gapStartTicks = currentTicks;
                currentState = STATE_GAP_OR_DEADEND;
                break;
            }

            //normal pid tracking
            int32_t error = computePositionError(false);
            float dError = static_cast<float>(error - prevError) / (LOOP_INTERVAL_US * 1e-6f);
            prevError = error;

            int16_t correction = static_cast<int16_t>(PID_KP * error + PID_KD * dError * 0.001f);

            //slow down on sharp curves
            int16_t speed = BASE_SPEED;
            if (abs(error) > 2000) {
                speed = MIN_SPEED;
            }

            drive(speed + correction, speed - correction);
            break;
        }

        case STATE_INVERTED_PID: {
            //check exit condition: outer front sensors see normal white floor while center sees black line
            //remember: in inverted mode, sensorVal is 1000 - rawNorm
            if (!isSensorBlack(4) && !isSensorBlack(5) && !isSensorBlack(10) && !isSensorBlack(11) && hasLine()) {
                invertedActive = false;
                currentState = STATE_NORMAL_PID;
                break;
            }

            //pid line following strictly on front 8 sensors
            int32_t error = computePositionError(true);
            float dError = static_cast<float>(error - prevError) / (LOOP_INTERVAL_US * 1e-6f);
            prevError = error;

            int16_t correction = static_cast<int16_t>(PID_KP * error + PID_KD * dError * 0.001f);
            drive(BASE_SPEED + correction, BASE_SPEED - correction);
            break;
        }

        case STATE_SHARP_TURN: {
            //execute zero radius spin in latched direction
            if (latchedDir == DIR_LEFT) {
                drive(-TURN_SPEED, TURN_SPEED);
            } else {
                drive(TURN_SPEED, -TURN_SPEED);
            }

            //re-lock line on center sensors
            if (isSensorBlack(6) || isSensorBlack(7) || isSensorBlack(8) || isSensorBlack(9)) {
                latchedDir = DIR_NONE;
                currentState = STATE_NORMAL_PID;
            }
            break;
        }

        case STATE_GAP_OR_DEADEND: {
            //hold straight heading using encoders
            int16_t diff = static_cast<int16_t>(ENC_KP * (leftEnc - rightEnc));
            drive(MIN_SPEED - diff, MIN_SPEED + diff);

            //line found within gap limit
            if (hasLine()) {
                currentState = STATE_NORMAL_PID;
                break;
            }

            //exceeded max gap distance: #2 dead end reached
            if (currentTicks - gapStartTicks >= MAX_GAP_TICKS) {
                currentState = STATE_UTURN_180;
            }
            break;
        }

        case STATE_UTURN_180: {
            //spin 180 degrees left
            drive(-TURN_SPEED, TURN_SPEED);

            //re-lock line once spun around
            if (hasLine() && (currentTicks - gapStartTicks > UTURN_180_TICKS)) {
                latchedDir = DIR_NONE;
                currentState = STATE_NORMAL_PID;
            }
            break;
        }

        case STATE_ALL_BLACK_CHECK: {
            //creep forward to confirm feature type
            int16_t diff = static_cast<int16_t>(ENC_KP * (leftEnc - rightEnc));
            drive(MIN_SPEED - diff, MIN_SPEED + diff);

            int32_t distance = currentTicks - checkStartTicks;

            //if line re-emerges or front is no longer all black, return to normal tracking
            if (!isFrontAllBlack() && hasLine()) {
                currentState = STATE_NORMAL_PID;
                break;
            }

            //check finish box confirmation after driving stop box distance
            if (distance >= STOP_BOX_TICKS) {
                if (isAllBlack()) {
                    shortBrake();
                    motorsArm(false);
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
            //remain locked in full stop
            shortBrake();
            motorsArm(false);
            break;
        }
    }
}
