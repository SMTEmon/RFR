#pragma once
#include <stdint.h>
#include <stdlib.h>

//envelope state for dual motor acceleration, decel and anti-plug braking
struct MotorEnvelopeState {
    int16_t curL = 0;
    int16_t curR = 0;
    int16_t currentBudget = 0;
    int8_t lastSignL = 0; //1 fwd, -1 rev
    int8_t lastSignR = 0;
    uint8_t coastTicksL = 0;
    uint8_t coastTicksR = 0;

    void reset() {
        curL = 0;
        curR = 0;
        currentBudget = 0;
        lastSignL = 0;
        lastSignR = 0;
        coastTicksL = 0;
        coastTicksR = 0;
    }
};

//step dual motor speeds with total current-budget slew and zero-crossing coast dwell
inline void stepDualMotorEnvelope(MotorEnvelopeState &st,
                                  int16_t rawTargetL, int16_t rawTargetR,
                                  int16_t accelStep, int16_t decelStep,
                                  int16_t maxWheelStep, uint8_t reversalCoastTicks,
                                  int16_t &outL, int16_t &outR) {
    //track kinetic momentum signs
    if (abs(st.curL) >= 25) {
        st.lastSignL = (st.curL > 0) ? 1 : -1;
    }
    if (abs(st.curR) >= 25) {
        st.lastSignR = (st.curR > 0) ? 1 : -1;
    }

    int16_t targetL = rawTargetL;
    int16_t targetR = rawTargetR;

    //check reversal for left wheel
    if (targetL > 0 && st.lastSignL == -1) {
        if (st.coastTicksL < reversalCoastTicks) {
            st.coastTicksL++;
            targetL = 0; //hold 0 during dwell
        } else {
            st.lastSignL = 0;
            st.coastTicksL = 0;
        }
    } else if (targetL < 0 && st.lastSignL == 1) {
        if (st.coastTicksL < reversalCoastTicks) {
            st.coastTicksL++;
            targetL = 0; //hold 0 during dwell
        } else {
            st.lastSignL = 0;
            st.coastTicksL = 0;
        }
    } else {
        st.coastTicksL = 0;
        if (targetL == 0 && st.curL == 0) {
            st.lastSignL = 0;
        }
    }

    //check reversal for right wheel
    if (targetR > 0 && st.lastSignR == -1) {
        if (st.coastTicksR < reversalCoastTicks) {
            st.coastTicksR++;
            targetR = 0; //hold 0 during dwell
        } else {
            st.lastSignR = 0;
            st.coastTicksR = 0;
        }
    } else if (targetR < 0 && st.lastSignR == 1) {
        if (st.coastTicksR < reversalCoastTicks) {
            st.coastTicksR++;
            targetR = 0; //hold 0 during dwell
        } else {
            st.lastSignR = 0;
            st.coastTicksR = 0;
        }
    } else {
        st.coastTicksR = 0;
        if (targetR == 0 && st.curR == 0) {
            st.lastSignR = 0;
        }
    }

    //total supply current budget regulation
    int32_t targetMag = abs(targetL) + abs(targetR);
    int16_t budgetStep = 2 * accelStep;
    if (targetMag > st.currentBudget) {
        int32_t nextBudget = st.currentBudget + budgetStep;
        st.currentBudget = static_cast<int16_t>((nextBudget < targetMag) ? nextBudget : targetMag);
    } else if (targetMag < st.currentBudget) {
        int32_t nextBudget = st.currentBudget - 2 * decelStep;
        st.currentBudget = static_cast<int16_t>((nextBudget > targetMag) ? nextBudget : targetMag);
    }

    if (targetMag > st.currentBudget && targetMag > 0) {
        targetL = static_cast<int16_t>((static_cast<int32_t>(targetL) * st.currentBudget) / targetMag);
        targetR = static_cast<int16_t>((static_cast<int32_t>(targetR) * st.currentBudget) / targetMag);
    }

    //per-wheel step clamping with asymmetric accel vs decel
    auto stepWheel = [](int16_t cur, int16_t tgt, int16_t maxStep, int16_t decStep) -> int16_t {
        bool sameSign = (cur >= 0 && tgt >= 0) || (cur <= 0 && tgt <= 0);
        bool increasingMag = abs(tgt) > abs(cur);
        int16_t limit = (sameSign && !increasingMag) ? decStep : maxStep;
        int16_t diff = tgt - cur;
        if (diff > limit) return cur + limit;
        if (diff < -limit) return cur - limit;
        return tgt;
    };

    st.curL = stepWheel(st.curL, targetL, maxWheelStep, decelStep);
    st.curR = stepWheel(st.curR, targetR, maxWheelStep, decelStep);

    outL = st.curL;
    outR = st.curR;
}

//step throttle toward target with asymmetric accel vs decel limits
inline int16_t stepThrottle(int16_t current, int16_t target, int16_t accelStep, int16_t decelStep) {
    bool sameSign = (current >= 0 && target >= 0) || (current <= 0 && target <= 0);
    bool increasingMag = abs(target) > abs(current);
    int16_t maxStep = (sameSign && !increasingMag) ? decelStep : accelStep;

    int16_t diff = target - current;
    if (diff > maxStep) return current + maxStep;
    if (diff < -maxStep) return current - maxStep;
    return target;
}

//clamp wheel pwm step size to prevent instant plug braking current spikes
inline int16_t clampWheelStep(int16_t current, int16_t target, int16_t maxStep) {
    int16_t diff = target - current;
    if (diff > maxStep) return current + maxStep;
    if (diff < -maxStep) return current - maxStep;
    return target;
}

//safe positive modulo for lap tick wrapping
inline int32_t wrapLapTicks(int32_t ticks, int32_t lapTicks) {
    if (lapTicks <= 0) return 0;
    int32_t mod = ticks % lapTicks;
    return (mod < 0) ? (mod + lapTicks) : mod;
}

//differential straight heading hold from state entry baseline
inline int16_t calcStraightHoldDiff(int32_t leftEnc, int32_t rightEnc,
                                    int32_t startLeft, int32_t startRight,
                                    float encKp, int16_t maxDiff) {
    int32_t delta = (leftEnc - startLeft) - (rightEnc - startRight);
    int32_t raw = static_cast<int32_t>(encKp * static_cast<float>(delta));
    if (raw > maxDiff) return maxDiff;
    if (raw < -maxDiff) return -maxDiff;
    return static_cast<int16_t>(raw);
}

//safe signed index calculation for loop closure correlation
inline int32_t calcCorrelationPairIdx(uint16_t totalBins, uint16_t k, int16_t testL) {
    int32_t idxA = static_cast<int32_t>(totalBins) - 1 - static_cast<int32_t>(k);
    int32_t idxB = idxA - static_cast<int32_t>(testL);
    return (idxA >= 0 && idxB >= 0) ? idxB : -1;
}
