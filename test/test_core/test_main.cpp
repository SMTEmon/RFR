#include <unity.h>
#include "lfr_math.h"
#include "protocol.h"

void setUp(void) {}
void tearDown(void) {}

void test_step_throttle_accel_decel(void) {
    //0 -> +140 should step by accelStep (3)
    int16_t v = stepThrottle(0, 140, 3, 14);
    TEST_ASSERT_EQUAL_INT16(3, v);

    //0 -> -140 should also step by accelStep (3)
    v = stepThrottle(0, -140, 3, 14);
    TEST_ASSERT_EQUAL_INT16(-3, v);

    //+140 -> 0 should step by decelStep (14)
    v = stepThrottle(140, 0, 3, 14);
    TEST_ASSERT_EQUAL_INT16(126, v);

    //-140 -> 0 should step by decelStep (14)
    v = stepThrottle(-140, 0, 3, 14);
    TEST_ASSERT_EQUAL_INT16(-126, v);
}

void test_clamp_wheel_step(void) {
    //+180 -> -180 plug brake should be clamped to +180 - 15 = 165
    int16_t clamped = clampWheelStep(180, -180, 15);
    TEST_ASSERT_EQUAL_INT16(165, clamped);

    //-180 -> +180 plug brake should be clamped to -180 + 15 = -165
    clamped = clampWheelStep(-180, 180, 15);
    TEST_ASSERT_EQUAL_INT16(-165, clamped);

    //small change within step limit should reach target directly
    clamped = clampWheelStep(100, 108, 15);
    TEST_ASSERT_EQUAL_INT16(108, clamped);
}

void test_wrap_lap_ticks(void) {
    //zero lapTicks should not divide by zero
    TEST_ASSERT_EQUAL_INT32(0, wrapLapTicks(1000, 0));

    //normal positive modulo
    TEST_ASSERT_EQUAL_INT32(50, wrapLapTicks(250, 100));

    //negative ticks should wrap into positive range [0, 100)
    TEST_ASSERT_EQUAL_INT32(75, wrapLapTicks(-25, 100));
}

void test_calc_straight_hold_diff(void) {
    //if both wheels ran 2000 ticks during previous turns, but run equally in gap
    int32_t leftEnc = 2500;
    int32_t rightEnc = 500;
    int32_t startLeft = 2400;  //entered gap at 2400
    int32_t startRight = 400;  //entered gap at 400
    //delta in gap: (2500 - 2400) - (500 - 400) = 100 - 100 = 0
    int16_t diff = calcStraightHoldDiff(leftEnc, rightEnc, startLeft, startRight, 2.0f, 220);
    TEST_ASSERT_EQUAL_INT16(0, diff);

    //if robot veered left in gap: leftDelta = 110, rightDelta = 100 -> delta = +10 -> diff = 20
    leftEnc = 2510;
    diff = calcStraightHoldDiff(leftEnc, rightEnc, startLeft, startRight, 2.0f, 220);
    TEST_ASSERT_EQUAL_INT16(20, diff);
}

void test_correlation_pair_idx_underflow(void) {
    //short loop underflow case: candLapBins = 50, totalBins = 62, testL = 58, k = 11
    //idxA = 62 - 1 - 11 = 50. idxB = 50 - 58 = -8. Should return -1 (not unsigned wrap to 65528)
    int32_t idxB = calcCorrelationPairIdx(62, 11, 58);
    TEST_ASSERT_EQUAL_INT32(-1, idxB);

    //valid index case: totalBins = 100, k = 0, testL = 50 -> idxA = 99, idxB = 49
    idxB = calcCorrelationPairIdx(100, 0, 50);
    TEST_ASSERT_EQUAL_INT32(49, idxB);
}

void test_nav_state_to_str(void) {
    TEST_ASSERT_EQUAL_STRING("NORMAL_PID", navStateToStr(STATE_NORMAL_PID));
    TEST_ASSERT_EQUAL_STRING("STOPPED", navStateToStr(STATE_STOPPED));
    TEST_ASSERT_EQUAL_STRING("UNKNOWN", navStateToStr(99));
}

void test_dual_motor_envelope_straight_launch(void) {
    MotorEnvelopeState st;
    int16_t outL = 0, outR = 0;

    //tick 1: budget ramps from 0 to 4 (2*accelStep with accelStep=2)
    stepDualMotorEnvelope(st, 140, 140, 2, 10, 10, 20, outL, outR);
    TEST_ASSERT_EQUAL_INT16(2, outL);
    TEST_ASSERT_EQUAL_INT16(2, outR);
    TEST_ASSERT_EQUAL_INT16(4, st.currentBudget);

    //tick 2: budget ramps to 8
    stepDualMotorEnvelope(st, 140, 140, 2, 10, 10, 20, outL, outR);
    TEST_ASSERT_EQUAL_INT16(4, outL);
    TEST_ASSERT_EQUAL_INT16(4, outR);
    TEST_ASSERT_EQUAL_INT16(8, st.currentBudget);
}

void test_dual_motor_envelope_spin_launch(void) {
    MotorEnvelopeState st;
    int16_t outL = 0, outR = 0;

    //zero-center spin (-120, +120) total mag 240, budget ramps by 4
    stepDualMotorEnvelope(st, -120, 120, 2, 10, 10, 20, outL, outR);
    TEST_ASSERT_EQUAL_INT16(-2, outL);
    TEST_ASSERT_EQUAL_INT16(2, outR);
    TEST_ASSERT_EQUAL_INT16(4, st.currentBudget);
}

void test_dual_motor_envelope_cruising_steering(void) {
    MotorEnvelopeState st;
    int16_t outL = 0, outR = 0;

    //simulate reaching cruising speed 140, 140 (budget 280)
    st.curL = 140;
    st.curR = 140;
    st.currentBudget = 280;
    st.lastSignL = 1;
    st.lastSignR = 1;

    //pid steering targets 160, 120 (sum 280, no budget expansion needed)
    stepDualMotorEnvelope(st, 160, 120, 2, 10, 10, 20, outL, outR);
    TEST_ASSERT_EQUAL_INT16(150, outL); //stepped by maxWheelStep (10)
    TEST_ASSERT_EQUAL_INT16(130, outR); //stepped down by maxWheelStep (10)

    //tick 2 reaches target cleanly
    stepDualMotorEnvelope(st, 160, 120, 2, 10, 10, 20, outL, outR);
    TEST_ASSERT_EQUAL_INT16(160, outL);
    TEST_ASSERT_EQUAL_INT16(120, outR);
}

void test_dual_motor_envelope_reversal_coast_dwell(void) {
    MotorEnvelopeState st;
    int16_t outL = 0, outR = 0;

    //init moving forward at +100
    st.curL = 100;
    st.curR = 100;
    st.currentBudget = 200;
    st.lastSignL = 1;
    st.lastSignR = 1;

    //request sudden reverse -100 on left wheel
    //wheel must dwell at 0 for 20 ticks
    for (int i = 0; i < 20; i++) {
        stepDualMotorEnvelope(st, -100, 100, 2, 10, 10, 20, outL, outR);
        //left wheel must not become negative during the 20-tick coast dwell
        TEST_ASSERT_GREATER_OR_EQUAL_INT16(0, outL);
    }
    TEST_ASSERT_EQUAL_INT16(0, outL);

    //tick 21: dwell expired, left wheel now ramps into negative safely
    stepDualMotorEnvelope(st, -100, 100, 2, 10, 10, 20, outL, outR);
    TEST_ASSERT_LESS_THAN_INT16(0, outL);
}

void test_dual_motor_envelope_reversal_cancel(void) {
    MotorEnvelopeState st;
    int16_t outL = 0, outR = 0;

    st.curL = 80;
    st.curR = 80;
    st.currentBudget = 160;
    st.lastSignL = 1;
    st.lastSignR = 1;

    //reversal requested
    stepDualMotorEnvelope(st, -80, 80, 2, 10, 10, 20, outL, outR);
    TEST_ASSERT_EQUAL_UINT8(1, st.coastTicksL);

    //user cancels reversal and targets forward again
    stepDualMotorEnvelope(st, 80, 80, 2, 10, 10, 20, outL, outR);
    TEST_ASSERT_EQUAL_UINT8(0, st.coastTicksL);
    TEST_ASSERT_GREATER_THAN_INT16(0, outL);
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_step_throttle_accel_decel);
    RUN_TEST(test_clamp_wheel_step);
    RUN_TEST(test_wrap_lap_ticks);
    RUN_TEST(test_calc_straight_hold_diff);
    RUN_TEST(test_correlation_pair_idx_underflow);
    RUN_TEST(test_nav_state_to_str);
    RUN_TEST(test_dual_motor_envelope_straight_launch);
    RUN_TEST(test_dual_motor_envelope_spin_launch);
    RUN_TEST(test_dual_motor_envelope_cruising_steering);
    RUN_TEST(test_dual_motor_envelope_reversal_coast_dwell);
    RUN_TEST(test_dual_motor_envelope_reversal_cancel);
    return UNITY_END();
}
