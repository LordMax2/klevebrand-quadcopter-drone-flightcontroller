#include <unity.h>
#include "../lib/klevebrand_maxfly_drone/components/quadcopter_pid/quadcopter_pid.h"

void setUp(void) {
    // set stuff up here
}

void tearDown(void) {
    // clean stuff up here
}

/// @brief 
/// Test the PID throttle output when PID constants are zero. 
/// The throttle output from the PID should be the same as the throttle input when the constants are zero.
void test_zero_pid_throttle() {
    Pid pid(0, 0, 0);

    float throttle = 1500;

    float gyroRoll = 0;
    float gyroPitch = 0;
    float gyroYaw = 0;

    float desiredRoll = 0;
    float desiredPitch = 0;
    float desiredYaw = 0;

    float throttleLf = pid.pidThrottleLF(throttle, gyroRoll, desiredRoll, gyroPitch, desiredPitch, gyroYaw, desiredYaw);
    float throttleRf = pid.pidThrottleRF(throttle, gyroRoll, desiredRoll, gyroPitch, desiredPitch, gyroYaw, desiredYaw);
    float throttleLb = pid.pidThrottleLB(throttle, gyroRoll, desiredRoll, gyroPitch, desiredPitch, gyroYaw, desiredYaw);
    float throttleRb = pid.pidThrottleRB(throttle, gyroRoll, desiredRoll, gyroPitch, desiredPitch, gyroYaw, desiredYaw);

    TEST_ASSERT_EQUAL(throttle, throttleLf);
    TEST_ASSERT_EQUAL(throttle, throttleRf);
    TEST_ASSERT_EQUAL(throttle, throttleLb);
    TEST_ASSERT_EQUAL(throttle, throttleRb);
}

int main( int argc, char **argv) {
    UNITY_BEGIN();

    RUN_TEST(test_zero_pid_throttle);

    UNITY_END();
}