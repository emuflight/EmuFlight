/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdint.h>
#include <stdbool.h>
#include <limits.h>
#include <cmath>

#include "unittest_macros.h"
#include "gtest/gtest.h"
#include "build/debug.h"

bool simulateMixerSaturated = false;
float simulatedSetpointRate[3] = { 0,0,0 };
float simulatedRcDeflection[3] = { 0,0,0 };
float simulatedThrottlePIDAttenuation = 1.0f;
float simulatedControllerMixRange = 0.0f;

int16_t debug[DEBUG16_VALUE_COUNT];
uint8_t debugMode;

extern "C" {
    #include "build/debug.h"
    #include "common/axis.h"
    #include "common/maths.h"
    #include "common/filter.h"

    #include "config/config_reset.h"
    #include "pg/pg.h"
    #include "pg/pg_ids.h"

    #include "drivers/sound_beeper.h"
    #include "drivers/time.h"

    #include "fc/fc_core.h"
    #include "fc/fc_rc.h"

    #include "fc/rc_controls.h"
    #include "fc/runtime_config.h"

    #include "flight/pid.h"
    #include "flight/imu.h"
    #include "flight/mixer.h"

    #include "io/gps.h"

    #include "sensors/gyro.h"
    #include "sensors/acceleration.h"

    gyro_t gyro;
    attitudeEulerAngles_t attitude;

    float getThrottlePIDAttenuation(void) { return simulatedThrottlePIDAttenuation; }
    float getControllerMixRange(void) { return simulatedControllerMixRange; }
    float getSetpointRate(int axis) { return simulatedSetpointRate[axis]; }
    bool mixerIsOutputSaturated(int, float) { return simulateMixerSaturated; }
    float getRcDeflectionAbs(int axis) { return ABS(simulatedRcDeflection[axis]); }
    void systemBeep(bool) { }
    bool gyroOverflowDetected(void) { return false; }
    float getRcDeflection(int axis) { return simulatedRcDeflection[axis]; }
    void beeperConfirmationBeeps(uint8_t) { }
    
    // Additional stubs for pid testing
    void mixerInitProfile(void) { }
    
    bool linearThrustEnabled = false;
    float getThrottlePAttenuation(void) { return 1.0f; }
    float getThrottleIAttenuation(void) { return 1.0f; }
    float getThrottleDAttenuation(void) { return 1.0f; }
    
    float getAngleModeAngles(int axis) { 
        (void)axis;
        return 0.0f; 
    }
    
    float howUpsideDown(void) { 
        return 0.0f; 
    }
}

pidProfile_t *pidProfile;
rollAndPitchTrims_t rollAndPitchTrims = { { 0, 0 } };

// Stub for gyro configuration system
gyroConfig_t gyroConfig_System;

int loopIter = 0;

// Always use same defaults for testing in future releases even when defaults change
void setDefaultTestSettings(void) {
    pgResetAll();
    pidProfile = pidProfilesMutable(1);
    pidProfile->pid[PID_ROLL]  =  { 40, 40, 30, 0 };
    pidProfile->pid[PID_PITCH] =  { 58, 50, 35, 0 };
    pidProfile->pid[PID_YAW]   =  { 70, 45, 20, 0 };
    pidProfile->pid[PID_LEVEL_LOW] =  { 70, 0, 10, 40 };
    pidProfile->pid[PID_LEVEL_HIGH] =  { 35, 0, 1, 0 };


    pidProfile->pidSumLimit = PIDSUM_LIMIT;
    pidProfile->pidSumLimitYaw = PIDSUM_LIMIT_YAW;
    pidProfile->dFilter[ROLL].dLpf = 100;
    pidProfile->dFilter[ROLL].dLpf2 = 0;
    pidProfile->dterm_filter_type = FILTER_BIQUAD;
    pidProfile->itermWindupPointPercent = 50;
    pidProfile->pidAtMinThrottle = PID_STABILISATION_ON;
    pidProfile->levelAngleLimit = 55;
    pidProfile->yawRateAccelLimit = 100;
    pidProfile->rateAccelLimit = 0;
    pidProfile->crash_time = 500;
    pidProfile->crash_delay = 0;
    pidProfile->crash_recovery_angle = 10;
    pidProfile->crash_recovery_rate = 100;
    pidProfile->crash_dthreshold = 50;
    pidProfile->crash_gthreshold = 400;
    pidProfile->crash_setpoint_threshold = 350;
    pidProfile->crash_recovery = PID_CRASH_RECOVERY_OFF;
    pidProfile->horizon_tilt_effect = 75;
    //pidProfile->horizon_tilt_expert_mode = false;
    pidProfile->crash_limit_yaw = 200;
    pidProfile->itermLimit = 150;
    pidProfile->throttle_boost = 0;
    pidProfile->throttle_boost_cutoff = 15;
    pidProfile->iterm_rotation = false;

    gyro.targetLooptime = 4000;
}

timeUs_t currentTestTime(void) {
    return targetPidLooptime * loopIter++;
}

void resetTest(void) {
    loopIter = 0;
    simulateMixerSaturated = false;
    simulatedThrottlePIDAttenuation = 1.0f;
    simulatedControllerMixRange = 0.0f;

    pidStabilisationState(PID_STABILISATION_OFF);
    DISABLE_ARMING_FLAG(ARMED);

    setDefaultTestSettings();
    for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
        pidData[axis].P = 0;
        pidData[axis].I = 0;
        pidData[axis].D = 0;
        pidData[axis].Sum = 0;
        simulatedSetpointRate[axis] = 0;
        simulatedRcDeflection[axis] = 0;
        gyro.gyroADCf[axis] = 0;
    }
    attitude.values.roll = 0;
    attitude.values.pitch = 0;
    attitude.values.yaw = 0;

    flightModeFlags = 0;
    pidInit(pidProfile);

    // Run pidloop for a while after reset
    for (int loop = 0; loop < 20; loop++) {
        pidController(pidProfile, &rollAndPitchTrims, currentTestTime());
    }
}

void setStickPosition(int axis, float stickRatio) {
    simulatedSetpointRate[axis] = 1998.0f * stickRatio;
    simulatedRcDeflection[axis] = stickRatio;
}

// All calculations will have 10% tolerance
float calculateTolerance(float input) {
    return fabs(input * 0.1f);
}

TEST(pidControllerTest, testInitialisation)
{
    resetTest();

    // In initial state PIDsums should be 0
    for (int axis = 0; axis <= FD_YAW; axis++) {
        EXPECT_FLOAT_EQ(0, pidData[axis].P);
        EXPECT_FLOAT_EQ(0, pidData[axis].I);
        EXPECT_FLOAT_EQ(0, pidData[axis].D);
    }
}

TEST(pidControllerTest, testStabilisationDisabled) {
    ENABLE_ARMING_FLAG(ARMED);
    // Run few loops to make sure there is no error building up when stabilisation disabled

    for (int loop = 0; loop < 10; loop++) {
        pidController(pidProfile, &rollAndPitchTrims, currentTestTime());

        // PID controller should not do anything, while stabilisation disabled
        EXPECT_FLOAT_EQ(0, pidData[FD_ROLL].P);
        EXPECT_FLOAT_EQ(0, pidData[FD_PITCH].P);
        EXPECT_FLOAT_EQ(0, pidData[FD_YAW].P);
        EXPECT_FLOAT_EQ(0, pidData[FD_ROLL].I);
        EXPECT_FLOAT_EQ(0, pidData[FD_PITCH].I);
        EXPECT_FLOAT_EQ(0, pidData[FD_YAW].I);
        EXPECT_FLOAT_EQ(0, pidData[FD_ROLL].D);
        EXPECT_FLOAT_EQ(0, pidData[FD_PITCH].D);
        EXPECT_FLOAT_EQ(0, pidData[FD_YAW].D);
    }
}

TEST(pidControllerTest, testPidLoop) {
    // Test fundamental PID controller behavior
    // Make sure to start with fresh values
    resetTest();
    ENABLE_ARMING_FLAG(ARMED);
    pidStabilisationState(PID_STABILISATION_ON);

    pidController(pidProfile, &rollAndPitchTrims, currentTestTime());

    // Loop 1 - No error, expect zero output
    EXPECT_FLOAT_EQ(0, pidData[FD_ROLL].P);
    EXPECT_FLOAT_EQ(0, pidData[FD_PITCH].P);
    EXPECT_FLOAT_EQ(0, pidData[FD_YAW].P);
    EXPECT_FLOAT_EQ(0, pidData[FD_ROLL].I);
    EXPECT_FLOAT_EQ(0, pidData[FD_PITCH].I);
    EXPECT_FLOAT_EQ(0, pidData[FD_YAW].I);
    EXPECT_FLOAT_EQ(0, pidData[FD_ROLL].D);
    EXPECT_FLOAT_EQ(0, pidData[FD_PITCH].D);
    EXPECT_FLOAT_EQ(0, pidData[FD_YAW].D);

    // Add error on ROLL
    gyro.gyroADCf[FD_ROLL] = 100;
    pidController(pidProfile, &rollAndPitchTrims, currentTestTime());

    // Loop 2 - ROLL error should produce non-zero P, I, D for ROLL only
    EXPECT_LT(pidData[FD_ROLL].P, 0);  // Should be negative for positive gyro error
    EXPECT_EQ(pidData[FD_PITCH].P, 0);
    EXPECT_EQ(pidData[FD_YAW].P, 0);
    EXPECT_LT(pidData[FD_ROLL].I, 0);  // Integral should accumulate
    EXPECT_EQ(pidData[FD_PITCH].I, 0);
    EXPECT_EQ(pidData[FD_YAW].I, 0);
    EXPECT_LT(pidData[FD_ROLL].D, 0);  // D-term from error change
    EXPECT_EQ(pidData[FD_PITCH].D, 0);
    EXPECT_EQ(pidData[FD_YAW].D, 0);

    // Add error on PITCH (ROLL error still present)
    gyro.gyroADCf[FD_PITCH] = -100;
    pidController(pidProfile, &rollAndPitchTrims, currentTestTime());

    // Loop 3 - PITCH error now present along with ROLL
    EXPECT_LT(pidData[FD_ROLL].P, 0);
    EXPECT_GT(pidData[FD_PITCH].P, 0);  // Negative gyro error produces positive P
    EXPECT_EQ(pidData[FD_YAW].P, 0);
    EXPECT_LT(pidData[FD_ROLL].I, 0);
    EXPECT_GT(pidData[FD_PITCH].I, 0);
    EXPECT_EQ(pidData[FD_YAW].I, 0);

    // Add error on YAW
    gyro.gyroADCf[FD_YAW] = 100;
    pidController(pidProfile, &rollAndPitchTrims, currentTestTime());

    // Loop 4 - All axes have error
    EXPECT_LT(pidData[FD_ROLL].P, 0);
    EXPECT_GT(pidData[FD_PITCH].P, 0);
    EXPECT_LT(pidData[FD_YAW].P, 0);
    EXPECT_LT(pidData[FD_ROLL].I, 0);
    EXPECT_GT(pidData[FD_PITCH].I, 0);
    EXPECT_LT(pidData[FD_YAW].I, 0);

    // Simulate Iterm behaviour during mixer saturation
    simulatedControllerMixRange = 1.2f;
    float rollIBefore = pidData[FD_ROLL].I;
    float pitchIBefore = pidData[FD_PITCH].I;
    float yawIBefore = pidData[FD_YAW].I;
    pidController(pidProfile, &rollAndPitchTrims, currentTestTime());
    // I-terms should be clamped during saturation
    EXPECT_NEAR(rollIBefore, pidData[FD_ROLL].I, fabs(rollIBefore * 0.1f));
    EXPECT_NEAR(pitchIBefore, pidData[FD_PITCH].I, fabs(pitchIBefore * 0.1f));
    EXPECT_NEAR(yawIBefore, pidData[FD_YAW].I, fabs(yawIBefore * 0.1f));
    simulatedControllerMixRange = 0;

    // Match stick to gyro to eliminate error
    simulatedSetpointRate[FD_ROLL] = 100;
    simulatedSetpointRate[FD_PITCH] = -100;
    simulatedSetpointRate[FD_YAW] = 100;

    for(int loop = 0; loop < 5; loop++) {
        pidController(pidProfile, &rollAndPitchTrims, currentTestTime());
    }
    
    // After error is removed, P and D should go to zero
    EXPECT_NEAR(0, pidData[FD_ROLL].P, 10);
    EXPECT_NEAR(0, pidData[FD_PITCH].P, 10);
    EXPECT_NEAR(0, pidData[FD_YAW].P, 10);
    EXPECT_NEAR(0, pidData[FD_ROLL].D, 10);
    EXPECT_NEAR(0, pidData[FD_PITCH].D, 10);
    EXPECT_NEAR(0, pidData[FD_YAW].D, 10);

    // Now disable Stabilisation
    pidStabilisationState(PID_STABILISATION_OFF);
    pidController(pidProfile, &rollAndPitchTrims, currentTestTime());

    // Should all be zero again
    EXPECT_FLOAT_EQ(0, pidData[FD_ROLL].P);
    EXPECT_FLOAT_EQ(0, pidData[FD_PITCH].P);
    EXPECT_FLOAT_EQ(0, pidData[FD_YAW].P);
    EXPECT_FLOAT_EQ(0, pidData[FD_ROLL].I);
    EXPECT_FLOAT_EQ(0, pidData[FD_PITCH].I);
    EXPECT_FLOAT_EQ(0, pidData[FD_YAW].I);
    EXPECT_FLOAT_EQ(0, pidData[FD_ROLL].D);
    EXPECT_FLOAT_EQ(0, pidData[FD_PITCH].D);
    EXPECT_FLOAT_EQ(0, pidData[FD_YAW].D);
}

TEST(pidControllerTest, testPidLevel) {
    // Test angle/level mode behavior
    resetTest();
    ENABLE_ARMING_FLAG(ARMED);
    pidStabilisationState(PID_STABILISATION_ON);

    // Enter angle mode
    enableFlightMode(ANGLE_MODE);
    pidController(pidProfile, &rollAndPitchTrims, currentTestTime());

    // Loop 1 - No input, expect zero
    EXPECT_FLOAT_EQ(0, pidData[FD_ROLL].P);
    EXPECT_FLOAT_EQ(0, pidData[FD_PITCH].P);
    EXPECT_FLOAT_EQ(0, pidData[FD_YAW].P);

    // Set attitude error in angle mode
    setStickPosition(FD_ROLL, 1.0f);
    setStickPosition(FD_PITCH, -1.0f);
    attitude.values.roll = 550;
    attitude.values.pitch = -550;
    
    // Run several loops to establish stable values
    pidController(pidProfile, &rollAndPitchTrims, currentTestTime());
    pidController(pidProfile, &rollAndPitchTrims, currentTestTime());
    pidController(pidProfile, &rollAndPitchTrims, currentTestTime());

    // Store the angle mode outputs
    float angleModeRollP = pidData[FD_ROLL].P;
    float angleModePitchP = pidData[FD_PITCH].P;

    // Now disable angle mode and run the same stick input in rate mode
    disableFlightMode(ANGLE_MODE);
    pidController(pidProfile, &rollAndPitchTrims, currentTestTime());

    // Expect rate mode to produce different output than angle mode
    EXPECT_NE(pidData[FD_ROLL].P, angleModeRollP);
    EXPECT_NE(pidData[FD_PITCH].P, angleModePitchP);
}


TEST(pidControllerTest, testPidHorizon) {
    // Test horizon mode behavior (blend between rate and angle)
    resetTest();
    ENABLE_ARMING_FLAG(ARMED);
    pidStabilisationState(PID_STABILISATION_ON);
    enableFlightMode(HORIZON_MODE);

    // Loop 1 - No input
    pidController(pidProfile, &rollAndPitchTrims, currentTestTime());
    EXPECT_FLOAT_EQ(0, pidData[FD_ROLL].P);
    EXPECT_FLOAT_EQ(0, pidData[FD_PITCH].P);
    EXPECT_FLOAT_EQ(0, pidData[FD_YAW].P);

    // Test with full stick input
    setStickPosition(FD_ROLL, 1.0f);
    setStickPosition(FD_PITCH, -1.0f);
    attitude.values.roll = 550;
    attitude.values.pitch = -550;
    pidController(pidProfile, &rollAndPitchTrims, currentTestTime());

    // Store full stick output
    float fullStickRollP = pidData[FD_ROLL].P;
    float fullStickPitchP = pidData[FD_PITCH].P;
    
    // Both should be non-zero
    EXPECT_NE(0, fullStickRollP);
    EXPECT_NE(0, fullStickPitchP);

    // Test with partial stick input
    setStickPosition(FD_ROLL, 0.1f);
    setStickPosition(FD_PITCH, -0.1f);
    attitude.values.roll = 536;
    attitude.values.pitch = -536;
    pidController(pidProfile, &rollAndPitchTrims, currentTestTime());

    float partialStickRollP = pidData[FD_ROLL].P;
    float partialStickPitchP = pidData[FD_PITCH].P;

    // Horizon mode should produce proportionally smaller outputs for smaller stick
    EXPECT_LT(fabs(partialStickRollP), fabs(fullStickRollP));
    EXPECT_LT(fabs(partialStickPitchP), fabs(fullStickPitchP));
}

TEST(pidControllerTest, testMixerSaturation) {
    resetTest();
    ENABLE_ARMING_FLAG(ARMED);
    pidStabilisationState(PID_STABILISATION_ON);

    // Test full stick response
    setStickPosition(FD_ROLL, 1.0f);
    setStickPosition(FD_PITCH, -1.0f);
    simulateMixerSaturated = true;
    pidController(pidProfile, &rollAndPitchTrims, currentTestTime());

    // Expect no iterm accumulation
    EXPECT_FLOAT_EQ(0, pidData[FD_YAW].P);
    EXPECT_FLOAT_EQ(0, pidData[FD_ROLL].I);
    EXPECT_FLOAT_EQ(0, pidData[FD_PITCH].I);
    EXPECT_FLOAT_EQ(0, pidData[FD_YAW].I);
}

// TODO - Add more scenarios
TEST(pidControllerTest, testCrashRecoveryMode) {
    resetTest();
    pidProfile->crash_recovery = PID_CRASH_RECOVERY_ON;
    pidInit(pidProfile);
    ENABLE_ARMING_FLAG(ARMED);
    pidStabilisationState(PID_STABILISATION_ON);
    sensorsSet(SENSOR_ACC);

    EXPECT_FALSE(crashRecoveryModeActive());

    // Crash recovery requires multiple conditions to be met simultaneously:
    // 1. getControllerMixRange() >= 1.0f (set to 1.2f in test)
    // 2. !inCrashRecoveryMode (starts false)
    // 3. ABS(pidData[axis].D) > crashDtermThreshold (50)
    // 4. ABS(errorRate) > crashGyroThreshold (400)
    // 5. ABS(getSetpointRate(axis)) < crashSetpointThreshold (350)
    // These conditions must all be true on the same loop iteration
    
    // Build initial state
    for (int i = 0; i < 50; i++) {
        gyro.gyroADCf[FD_ROLL] = 100.0f;
        pidController(pidProfile, &rollAndPitchTrims, currentTestTime());
    }

    // Set up crash conditions
    simulatedControllerMixRange = 1.2f;
    
    // Run with high gyro to build D-term and error
    // Loop must persist long enough to satisfy crash_time threshold
    int crashTimeLoops = (int)((pidProfile->crash_time * 1000) / targetPidLooptime) + 20;
    
    for (int loop = 0; loop < crashTimeLoops; loop++) {
        gyro.gyroADCf[FD_ROLL] = 500.0f;
        pidController(pidProfile, &rollAndPitchTrims, currentTestTime());
        
        // Check if crash recovery was triggered
        if (crashRecoveryModeActive()) {
            break;  // Success!
        }
    }

    // Note: This test verifies crash recovery can be activated
    // The actual triggering depends on all thresholds being met simultaneously
    EXPECT_TRUE(crashRecoveryModeActive());
}

TEST(pidControllerTest, pidSetpointTransition) {
// TODO
}

TEST(pidControllerTest, testDtermFiltering) {
// TODO
}

TEST(pidControllerTest, testItermRotationHandling) {
// TODO
}
