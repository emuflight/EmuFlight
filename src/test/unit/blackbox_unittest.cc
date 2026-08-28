/*
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
#include <string.h>

extern "C" {
    #include "platform.h"

    #include "blackbox/blackbox.h"
    #include "common/utils.h"

    #include "pg/pg.h"
    #include "pg/pg_ids.h"
    #include "pg/rx.h"

    #include "drivers/accgyro/accgyro.h"
    #include "drivers/accgyro/gyro_sync.h"
    #include "drivers/serial.h"

    #include "flight/failsafe.h"
    #include "flight/mixer.h"
    #include "flight/pid.h"

    #include "fc/rc_controls.h"
    #include "fc/rc_modes.h"

    #include "io/gps.h"
    #include "io/serial.h"

    #include "rx/rx.h"

    #include "sensors/battery.h"
    #include "sensors/gyro.h"

    // Register required for gyro_sync.c
    PG_REGISTER(gyroConfig_t, gyroConfig, PG_GYRO_CONFIG, 0);

    extern int16_t blackboxIInterval;
    extern int16_t blackboxPInterval;
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

gyroDev_t gyroDev;

TEST(BlackboxTest, TestInitIntervals)
{
    blackboxConfigMutable()->p_ratio = 32;
    // 250Hz PIDloop
    targetPidLooptime = 4000;
    blackboxInit();
    EXPECT_EQ(8, blackboxIInterval);
    EXPECT_EQ(0, blackboxPInterval);
    EXPECT_EQ(2048, blackboxSInterval);

    // 500Hz PIDloop
    targetPidLooptime = 2000;
    blackboxInit();
    EXPECT_EQ(16, blackboxIInterval);
    EXPECT_EQ(0, blackboxPInterval);
    EXPECT_EQ(4096, blackboxSInterval);

    // 1kHz PIDloop
    gyro.targetLooptime = gyroSetSampleRate(&gyroDev, GYRO_HARDWARE_LPF_1KHZ_SAMPLE, 1, false);
    targetPidLooptime = gyro.targetLooptime * 1;
    blackboxInit();
    EXPECT_EQ(32, blackboxIInterval);
    EXPECT_EQ(1, blackboxPInterval);
    EXPECT_EQ(8192, blackboxSInterval);

    // 2kHz PIDloop
    targetPidLooptime = 500;
    blackboxInit();
    EXPECT_EQ(64, blackboxIInterval);
    EXPECT_EQ(2, blackboxPInterval);
    EXPECT_EQ(16384, blackboxSInterval);

    // 4kHz PIDloop
    targetPidLooptime = 250;
    blackboxInit();
    EXPECT_EQ(128, blackboxIInterval);
    EXPECT_EQ(4, blackboxPInterval);
    EXPECT_EQ(32768, blackboxSInterval);

    // 8kHz PIDloop
    targetPidLooptime = 125;
    gyro.targetLooptime = gyroSetSampleRate(&gyroDev, GYRO_LPF_256HZ, 1, false);
    EXPECT_EQ(125, gyro.targetLooptime);
    blackboxInit();
    EXPECT_EQ(256, blackboxIInterval);
    EXPECT_EQ(8, blackboxPInterval);
    EXPECT_EQ(65536, blackboxSInterval);

    // 16kHz PIDloop
    targetPidLooptime = 63;
    gyro.targetLooptime = gyroSetSampleRate(&gyroDev, GYRO_LPF_256HZ, 2, true);
    EXPECT_EQ(64, gyro.targetLooptime);
    blackboxInit();
    EXPECT_EQ(512, blackboxIInterval); // note rounding
    EXPECT_EQ(16, blackboxPInterval);
    EXPECT_EQ(131072, blackboxSInterval);

    // 32kHz PIDloop
    targetPidLooptime = 31;
    gyro.targetLooptime = gyroSetSampleRate(&gyroDev, GYRO_LPF_256HZ, 1, true);
    EXPECT_EQ(32, gyro.targetLooptime);
    blackboxInit();
    EXPECT_EQ(1024, blackboxIInterval); // note rounding
    EXPECT_EQ(32, blackboxPInterval);
    EXPECT_EQ(262144, blackboxSInterval);
}

TEST(BlackboxTest, Test_500Hz)
{
    blackboxConfigMutable()->p_ratio = 32;
    // 500Hz PIDloop
    targetPidLooptime = 2000;
    blackboxInit();
    EXPECT_EQ(true, blackboxShouldLogIFrame());
    EXPECT_EQ(true, blackboxShouldLogPFrame());
    for (int ii = 0; ii < 15; ++ii) {
        blackboxAdvanceIterationTimers();
        EXPECT_EQ(false, blackboxShouldLogIFrame());
        EXPECT_EQ(true, blackboxShouldLogPFrame());
    }
    blackboxAdvanceIterationTimers();
    EXPECT_EQ(true, blackboxShouldLogIFrame());
    EXPECT_EQ(true, blackboxShouldLogPFrame());
}

TEST(BlackboxTest, Test_1kHz)
{
    blackboxConfigMutable()->p_ratio = 32;
    // 1kHz PIDloop
    targetPidLooptime = 1000;
    blackboxInit();
    EXPECT_EQ(true, blackboxShouldLogIFrame());
    EXPECT_EQ(true, blackboxShouldLogPFrame());
    EXPECT_EQ(0, blackboxSlowFrameIterationTimer);
    EXPECT_EQ(false, blackboxSlowFrameIterationTimer >= blackboxSInterval);
    blackboxSlowFrameIterationTimer = blackboxSInterval;
    EXPECT_EQ(true, writeSlowFrameIfNeeded());
    EXPECT_EQ(0, blackboxSlowFrameIterationTimer);

    for (int ii = 0; ii < 31; ++ii) {
        blackboxAdvanceIterationTimers();
        EXPECT_EQ(false, blackboxShouldLogIFrame());
        EXPECT_EQ(true, blackboxShouldLogPFrame());
        EXPECT_EQ(ii + 1, blackboxSlowFrameIterationTimer);
        EXPECT_EQ(false, writeSlowFrameIfNeeded());
    }
    blackboxAdvanceIterationTimers();
    EXPECT_EQ(true, blackboxShouldLogIFrame());
    EXPECT_EQ(true, blackboxShouldLogPFrame());
}

TEST(BlackboxTest, Test_2kHz)
{
    blackboxConfigMutable()->p_ratio = 32;
    // 2kHz PIDloop
    targetPidLooptime = 500;
    blackboxInit();
    EXPECT_EQ(64, blackboxIInterval);
    EXPECT_EQ(2, blackboxPInterval);
    EXPECT_EQ(true, blackboxShouldLogIFrame());
    EXPECT_EQ(true, blackboxShouldLogPFrame());
    blackboxAdvanceIterationTimers();
    EXPECT_EQ(false, blackboxShouldLogIFrame());
    EXPECT_EQ(false, blackboxShouldLogPFrame());

    for (int ii = 0; ii < 31; ++ii) {
        blackboxAdvanceIterationTimers();
        EXPECT_EQ(false, blackboxShouldLogIFrame());
        EXPECT_EQ(true, blackboxShouldLogPFrame());

        blackboxAdvanceIterationTimers();
        EXPECT_EQ(false, blackboxShouldLogIFrame());
        EXPECT_EQ(false, blackboxShouldLogPFrame());
    }

    blackboxAdvanceIterationTimers();
    EXPECT_EQ(true, blackboxShouldLogIFrame());
    EXPECT_EQ(true, blackboxShouldLogPFrame());

    blackboxAdvanceIterationTimers();
    EXPECT_EQ(false, blackboxShouldLogIFrame());
    EXPECT_EQ(false, blackboxShouldLogPFrame());
}

TEST(BlackboxTest, Test_8kHz)
{
    blackboxConfigMutable()->p_ratio = 32;
    // 8kHz PIDloop
    targetPidLooptime = 125;
    blackboxInit();
    EXPECT_EQ(true, blackboxShouldLogIFrame());
    EXPECT_EQ(true, blackboxShouldLogPFrame());
    EXPECT_EQ(0, blackboxSlowFrameIterationTimer);
    EXPECT_EQ(false, blackboxSlowFrameIterationTimer >= blackboxSInterval);
    blackboxSlowFrameIterationTimer = blackboxSInterval;
    EXPECT_EQ(true, writeSlowFrameIfNeeded());
    EXPECT_EQ(0, blackboxSlowFrameIterationTimer);

    for (int ii = 0; ii < 255; ++ii) {
        blackboxAdvanceIterationTimers();
        EXPECT_EQ(false, blackboxShouldLogIFrame());
        EXPECT_EQ(ii + 1, blackboxSlowFrameIterationTimer);
        EXPECT_EQ(false, writeSlowFrameIfNeeded());
    }
    blackboxAdvanceIterationTimers();
    EXPECT_EQ(true, blackboxShouldLogIFrame());
    EXPECT_EQ(true, blackboxShouldLogPFrame());
}

TEST(BlackboxTest, Test_zero_p_ratio)
{
    blackboxConfigMutable()->p_ratio = 0;
    // 1kHz PIDloop
    targetPidLooptime = 1000;
    blackboxInit();
    EXPECT_EQ(32, blackboxIInterval);
    EXPECT_EQ(0, blackboxPInterval);
    EXPECT_EQ(true, blackboxShouldLogIFrame());
    EXPECT_EQ(false, blackboxShouldLogPFrame());

    for (int ii = 0; ii < 31; ++ii) {
        blackboxAdvanceIterationTimers();
        EXPECT_EQ(false, blackboxShouldLogIFrame());
        EXPECT_EQ(false, blackboxShouldLogPFrame());
    }
    blackboxAdvanceIterationTimers();
    EXPECT_EQ(true, blackboxShouldLogIFrame());
    EXPECT_EQ(false, blackboxShouldLogPFrame());
}

TEST(BlackboxTest, Test_CalculatePDenom)
{
    blackboxConfigMutable()->p_ratio = 0;
    // note I-frame is logged every 32ms regardless of PIDloop rate
    // so p_ratio is 32 when blackbox logging rate is 1kHz

    // 1kHz PIDloop
    targetPidLooptime = 1000;
    blackboxInit();
    EXPECT_EQ(32, blackboxIInterval);
    EXPECT_EQ(32, blackboxCalculatePDenom(1, 1)); // 1kHz logging
    EXPECT_EQ(16, blackboxCalculatePDenom(1, 2));
    EXPECT_EQ(8, blackboxCalculatePDenom(1, 4));

    // 2kHz PIDloop
    targetPidLooptime = 500;
    blackboxInit();
    EXPECT_EQ(64, blackboxIInterval);
    EXPECT_EQ(64, blackboxCalculatePDenom(1, 1));
    EXPECT_EQ(32, blackboxCalculatePDenom(1, 2)); // 1kHz logging
    EXPECT_EQ(16, blackboxCalculatePDenom(1, 4));

    // 4kHz PIDloop
    targetPidLooptime = 250;
    blackboxInit();
    EXPECT_EQ(128, blackboxIInterval);
    EXPECT_EQ(128, blackboxCalculatePDenom(1, 1));
    EXPECT_EQ(64, blackboxCalculatePDenom(1, 2));
    EXPECT_EQ(32, blackboxCalculatePDenom(1, 4)); // 1kHz logging
    EXPECT_EQ(16, blackboxCalculatePDenom(1, 8));

    // 8kHz PIDloop
    targetPidLooptime = 125;
    blackboxInit();
    EXPECT_EQ(256, blackboxIInterval);
    EXPECT_EQ(256, blackboxCalculatePDenom(1, 1));
    EXPECT_EQ(128, blackboxCalculatePDenom(1, 2));
    EXPECT_EQ(64, blackboxCalculatePDenom(1, 4));
    EXPECT_EQ(32, blackboxCalculatePDenom(1, 8)); // 1kHz logging
    EXPECT_EQ(16, blackboxCalculatePDenom(1, 16));
}

TEST(BlackboxTest, Test_CalculateRates)
{
    // 1kHz PIDloop
    targetPidLooptime = 1000;
    blackboxConfigMutable()->p_ratio = 32;
    blackboxInit();
    EXPECT_EQ(32, blackboxIInterval);
    EXPECT_EQ(1, blackboxPInterval);
    EXPECT_EQ(1, blackboxGetRateDenom());

    blackboxConfigMutable()->p_ratio = 16;
    blackboxInit();
    EXPECT_EQ(32, blackboxIInterval);
    EXPECT_EQ(2, blackboxPInterval);
    EXPECT_EQ(2, blackboxGetRateDenom());

    blackboxConfigMutable()->p_ratio = 8;
    blackboxInit();
    EXPECT_EQ(32, blackboxIInterval);
    EXPECT_EQ(4, blackboxPInterval);
    EXPECT_EQ(4, blackboxGetRateDenom());


    // 8kHz PIDloop
    targetPidLooptime = 125;
    blackboxConfigMutable()->p_ratio = 32; // 1kHz logging
    blackboxInit();
    EXPECT_EQ(256, blackboxIInterval);
    EXPECT_EQ(8, blackboxPInterval);
    EXPECT_EQ(8, blackboxGetRateDenom());

    blackboxConfigMutable()->p_ratio = 48; // 1.5kHz logging
    blackboxInit();
    EXPECT_EQ(256, blackboxIInterval);
    EXPECT_EQ(5, blackboxPInterval);
    EXPECT_EQ(5, blackboxGetRateDenom());

    blackboxConfigMutable()->p_ratio = 64; // 2kHz logging
    blackboxInit();
    EXPECT_EQ(256, blackboxIInterval);
    EXPECT_EQ(4, blackboxPInterval);
    EXPECT_EQ(4, blackboxGetRateDenom());

    blackboxConfigMutable()->p_ratio = 128; // 4kHz logging
    blackboxInit();
    EXPECT_EQ(256, blackboxIInterval);
    EXPECT_EQ(2, blackboxPInterval);
    EXPECT_EQ(2, blackboxGetRateDenom());

    blackboxConfigMutable()->p_ratio = 256; // 8kHz logging
    blackboxInit();
    EXPECT_EQ(256, blackboxIInterval);
    EXPECT_EQ(1, blackboxPInterval);
    EXPECT_EQ(1, blackboxGetRateDenom());

    // 0.126 PIDloop
    targetPidLooptime = 126;
    blackboxConfigMutable()->p_ratio = 32; // 1kHz logging
    blackboxInit();
    EXPECT_EQ(253, blackboxIInterval);
    EXPECT_EQ(7, blackboxPInterval);
    EXPECT_EQ(7, blackboxGetRateDenom());

}


// STUBS
extern "C" {

PG_REGISTER(flight3DConfig_t, flight3DConfig, PG_MOTOR_3D_CONFIG, 0);
PG_REGISTER(mixerConfig_t, mixerConfig, PG_MIXER_CONFIG, 0);
PG_REGISTER(motorConfig_t, motorConfig, PG_MOTOR_CONFIG, 0);
PG_REGISTER(batteryConfig_t, batteryConfig, PG_BATTERY_CONFIG, 0);
PG_REGISTER(rxConfig_t, rxConfig, PG_RX_CONFIG, 0);
PG_REGISTER_ARRAY(modeActivationCondition_t, MAX_MODE_ACTIVATION_CONDITION_COUNT, modeActivationConditions, PG_MODE_ACTIVATION_PROFILE, 0);

uint8_t armingFlags;
uint8_t stateFlags;
const uint32_t baudRates[] = {0, 9600, 19200, 38400, 57600, 115200, 230400, 250000,
        400000, 460800, 500000, 921600, 1000000, 1500000, 2000000, 2470000}; // see baudRate_e
uint8_t debugMode;
extern int32_t blackboxHeaderBudget;
gpsSolutionData_t gpsSol;
int32_t GPS_home[2];

gyro_t gyro;

float motorOutputHigh, motorOutputLow;
float motor_disarmed[MAX_SUPPORTED_MOTORS];
struct pidProfile_s;
struct pidProfile_s *currentPidProfile;
uint32_t targetPidLooptime;

boxBitmask_t rcModeActivationMask;

void mspSerialAllocatePorts(void) {}
uint32_t getArmingBeepTimeMicros(void) {return 0;}
uint16_t getBatteryVoltageLatest(void) {return 0;}
uint8_t getMotorCount(void) {return 4;}
bool areMotorsRunning(void) { return false; }
bool IS_RC_MODE_ACTIVE(boxId_e) {return false;}
bool isModeActivationConditionPresent(boxId_e) {return false;}
uint32_t millis(void) {return 0;}
// Defaults preserve every pre-existing test's assumption (sensor/feature/RSSI absent);
// only TestFieldMasking flips these, and resets them before returning.
static bool stubSensorsPresent = false;
static bool stubFeatureEnabled = false;
static bool stubRssiConfigured = false;
bool sensors(uint32_t) {return stubSensorsPresent;}
// TestMotorFieldByteSync resets this before each capture window; no other test writes.
static int capturedSerialByteCount = 0;
void serialWrite(serialPort_t *, uint8_t) {
    capturedSerialByteCount++;
}
uint32_t serialTxBytesFree(const serialPort_t *) {return 0;}
bool isSerialTransmitBufferEmpty(const serialPort_t *) {return false;}
bool feature(uint32_t) {return stubFeatureEnabled;}
void mspSerialReleasePortIfAllocated(serialPort_t *) {}
serialPortConfig_t *findSerialPortConfig(serialPortFunction_e ) {return NULL;}
serialPort_t *findSharedSerialPort(uint32_t, serialPortFunction_e ) {return NULL;}
serialPort_t *openSerialPort(serialPortIdentifier_e, serialPortFunction_e, serialReceiveCallbackPtr, void *, uint32_t, portMode_e, portOptions_e) {return NULL;}
void closeSerialPort(serialPort_t *) {}
portSharing_e determinePortSharing(const serialPortConfig_t *, serialPortFunction_e ) {return PORTSHARING_UNUSED;}
failsafePhase_e failsafePhase(void) {return FAILSAFE_IDLE;}
bool rxAreFlightChannelsValid(void) {return false;}
bool rxIsReceivingSignal(void) {return false;}
bool isRssiConfigured(void) {return stubRssiConfigured;}

}

TEST(BlackboxTest, TestFieldMasking)
{
    // Reset shared global stub state before use, not just at teardown — a fatal failure earlier
    // in the file (or in this test) must not leave these dirty for whatever runs next.
    batteryConfigMutable()->voltageMeterSource = VOLTAGE_METER_NONE;
    batteryConfigMutable()->currentMeterSource = CURRENT_METER_NONE;
    blackboxConfigMutable()->record_acc = 0;
    stubSensorsPresent = false;
    stubFeatureEnabled = false;
    stubRssiConfigured = false;
    debugMode = 0;

    // NONZERO_PID_D_x dereferences currentPidProfile; give it real, nonzero-D backing storage, not the stub's NULL.
    static pidProfile_t testPidProfile = {};
    testPidProfile.pid[PID_ROLL].D = 1;
    testPidProfile.pid[PID_PITCH].D = 1;
    testPidProfile.pid[PID_YAW].D = 1;
    currentPidProfile = &testPidProfile;

    // All fields enabled by default (fields_disabled_mask == 0)
    blackboxConfigMutable()->fields_disabled_mask = 0;
    blackboxBuildConditionCache();
    EXPECT_EQ(true, testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_GYRO));
    EXPECT_EQ(true, testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_RC_COMMANDS));
    EXPECT_EQ(true, testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_SETPOINT));
    EXPECT_EQ(true, testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_PID));
    EXPECT_EQ(true, testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_NONZERO_PID_D_0));

    // Disabling GYRO via the mask must not affect unrelated fields (RC_COMMANDS, SETPOINT, GYROUNFILT)
    blackboxConfigMutable()->fields_disabled_mask = (1 << FLIGHT_LOG_FIELD_SELECT_GYRO);
    blackboxBuildConditionCache();
    EXPECT_EQ(false, testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_GYRO));
    EXPECT_EQ(true, testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_RC_COMMANDS));
    EXPECT_EQ(true, testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_SETPOINT));
    EXPECT_EQ(true, testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_GYROUNFILT));

    // Disabling GYROUNFILT independently must leave GYRO untouched (merged in from PR #1331)
    blackboxConfigMutable()->fields_disabled_mask = (1 << FLIGHT_LOG_FIELD_SELECT_GYROUNFILT);
    blackboxBuildConditionCache();
    EXPECT_EQ(false, testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_GYROUNFILT));
    EXPECT_EQ(true, testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_GYRO));

    // Disabling a different field (RC_COMMANDS) must re-enable GYRO and leave SETPOINT untouched
    blackboxConfigMutable()->fields_disabled_mask = (1 << FLIGHT_LOG_FIELD_SELECT_RC_COMMANDS);
    blackboxBuildConditionCache();
    EXPECT_EQ(true, testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_GYRO));
    EXPECT_EQ(false, testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_RC_COMMANDS));
    EXPECT_EQ(true, testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_SETPOINT));

    // Multiple bits combine (PID + SETPOINT disabled simultaneously); disabling PID also masks nonzero D-term
    blackboxConfigMutable()->fields_disabled_mask =
        (1 << FLIGHT_LOG_FIELD_SELECT_PID) | (1 << FLIGHT_LOG_FIELD_SELECT_SETPOINT);
    blackboxBuildConditionCache();
    EXPECT_EQ(false, testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_PID));
    EXPECT_EQ(false, testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_NONZERO_PID_D_0));
    EXPECT_EQ(false, testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_SETPOINT));
    EXPECT_EQ(true, testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_RC_COMMANDS));
    EXPECT_EQ(true, testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_GYRO));

    // Sensor-gated fields: give each underlying sensor/feature a "present" reading so the
    // mask bit is what actually flips true->false, not the pre-existing sensor-absent default.
    batteryConfigMutable()->voltageMeterSource = VOLTAGE_METER_ADC;
    batteryConfigMutable()->currentMeterSource = CURRENT_METER_ADC;
    blackboxConfigMutable()->record_acc = 1; // ACC condition also requires this, independent of the mask
    stubSensorsPresent = true; // backs MAG, BARO, RANGEFINDER, ACC conditions
    stubRssiConfigured = true;
    debugMode = 1; // != DEBUG_NONE

    blackboxConfigMutable()->fields_disabled_mask = 0;
    blackboxBuildConditionCache();
    EXPECT_EQ(true, testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_VBAT));
    EXPECT_EQ(true, testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_AMPERAGE_ADC));
    EXPECT_EQ(true, testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_MAG));
    EXPECT_EQ(true, testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_BARO));
#ifdef USE_RANGEFINDER
    EXPECT_EQ(true, testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_RANGEFINDER));
#endif
    EXPECT_EQ(true, testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_RSSI));
    EXPECT_EQ(true, testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_ACC));
    EXPECT_EQ(true, testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_DEBUG));
    EXPECT_EQ(true, testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_AT_LEAST_MOTORS_1));

    // Disabling BATTERY masks both VBAT and AMPERAGE_ADC, leaves the others untouched
    blackboxConfigMutable()->fields_disabled_mask = (1 << FLIGHT_LOG_FIELD_SELECT_BATTERY);
    blackboxBuildConditionCache();
    EXPECT_EQ(false, testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_VBAT));
    EXPECT_EQ(false, testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_AMPERAGE_ADC));
    EXPECT_EQ(true, testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_MAG));
    EXPECT_EQ(true, testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_ACC));

    // Disabling MAG masks only MAG
    blackboxConfigMutable()->fields_disabled_mask = (1 << FLIGHT_LOG_FIELD_SELECT_MAG);
    blackboxBuildConditionCache();
    EXPECT_EQ(false, testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_MAG));
    EXPECT_EQ(true, testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_BARO));
    EXPECT_EQ(true, testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_VBAT));

    // Disabling ALTITUDE masks both BARO and RANGEFINDER together
    blackboxConfigMutable()->fields_disabled_mask = (1 << FLIGHT_LOG_FIELD_SELECT_ALTITUDE);
    blackboxBuildConditionCache();
    EXPECT_EQ(false, testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_BARO));
#ifdef USE_RANGEFINDER
    EXPECT_EQ(false, testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_RANGEFINDER));
#endif
    EXPECT_EQ(true, testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_MAG));

    // Disabling RSSI masks only RSSI
    blackboxConfigMutable()->fields_disabled_mask = (1 << FLIGHT_LOG_FIELD_SELECT_RSSI);
    blackboxBuildConditionCache();
    EXPECT_EQ(false, testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_RSSI));
    EXPECT_EQ(true, testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_ACC));

    // Disabling ACC masks only ACC
    blackboxConfigMutable()->fields_disabled_mask = (1 << FLIGHT_LOG_FIELD_SELECT_ACC);
    blackboxBuildConditionCache();
    EXPECT_EQ(false, testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_ACC));
    EXPECT_EQ(true, testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_DEBUG));

    // Disabling DEBUG masks only DEBUG
    blackboxConfigMutable()->fields_disabled_mask = (1 << FLIGHT_LOG_FIELD_SELECT_DEBUG);
    blackboxBuildConditionCache();
    EXPECT_EQ(false, testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_DEBUG));
    EXPECT_EQ(true, testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_AT_LEAST_MOTORS_1));

    // Disabling MOTOR masks the motor-count-gated condition
    blackboxConfigMutable()->fields_disabled_mask = (1 << FLIGHT_LOG_FIELD_SELECT_MOTOR);
    blackboxBuildConditionCache();
    EXPECT_EQ(false, testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_AT_LEAST_MOTORS_1));
    EXPECT_EQ(true, testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_DEBUG));

    // GPS has no FLIGHT_LOG_FIELD_CONDITION_* wrapper; call sites check isFieldEnabled() directly
    blackboxConfigMutable()->fields_disabled_mask = 0;
    EXPECT_EQ(true, isFieldEnabled(FLIGHT_LOG_FIELD_SELECT_GPS));
    blackboxConfigMutable()->fields_disabled_mask = (1 << FLIGHT_LOG_FIELD_SELECT_GPS);
    EXPECT_EQ(false, isFieldEnabled(FLIGHT_LOG_FIELD_SELECT_GPS));
    EXPECT_EQ(true, isFieldEnabled(FLIGHT_LOG_FIELD_SELECT_DEBUG));

    // Reset all global stub state so no later test observes this fixture
    blackboxConfigMutable()->fields_disabled_mask = 0;
    blackboxBuildConditionCache();
    currentPidProfile = NULL;
    batteryConfigMutable()->voltageMeterSource = VOLTAGE_METER_NONE;
    batteryConfigMutable()->currentMeterSource = CURRENT_METER_NONE;
    stubSensorsPresent = false;
    stubFeatureEnabled = false;
    stubRssiConfigured = false;
    debugMode = 0;
}

TEST(BlackboxTest, TestMotorFieldByteSync)
{
    // Regression test for a header/data-write desync class: a mask that hides a field's
    // header must also stop write*Frame() emitting that field's bytes, not just flip the
    // condition-cache bit.

    // Reset shared global stub state before use, not just at teardown — matches TestFieldMasking.
    batteryConfigMutable()->voltageMeterSource = VOLTAGE_METER_NONE;
    batteryConfigMutable()->currentMeterSource = CURRENT_METER_NONE;
    stubSensorsPresent = false;
    stubFeatureEnabled = false;
    stubRssiConfigured = false;
    debugMode = 0;

    static pidProfile_t testPidProfile = {};
    testPidProfile.pid[PID_ROLL].D = 1;
    testPidProfile.pid[PID_PITCH].D = 1;
    testPidProfile.pid[PID_YAW].D = 1;
    currentPidProfile = &testPidProfile;

    blackboxConfigMutable()->fields_disabled_mask = 0;
    blackboxBuildConditionCache();
    const int headerFieldsWithMotor = unitTestCountVisibleMotorHeaderFields();
    capturedSerialByteCount = 0;
    unitTestWriteIntraframe();
    const int intraBytesWithMotor = capturedSerialByteCount;

    capturedSerialByteCount = 0;
    unitTestWriteInterframe();
    const int interBytesWithMotor = capturedSerialByteCount;

    blackboxConfigMutable()->fields_disabled_mask = (1 << FLIGHT_LOG_FIELD_SELECT_MOTOR);
    blackboxBuildConditionCache();
    const int headerFieldsWithoutMotor = unitTestCountVisibleMotorHeaderFields();
    capturedSerialByteCount = 0;
    unitTestWriteIntraframe();
    const int intraBytesWithoutMotor = capturedSerialByteCount;

    capturedSerialByteCount = 0;
    unitTestWriteInterframe();
    const int interBytesWithoutMotor = capturedSerialByteCount;

    // getMotorCount() stub returns 4: motor[0] (1 VB byte) + 3 deltas (1 VB byte each) = 4 bytes,
    // for both intraframe's raw encoding and interframe's average-predictor encoding.
    EXPECT_EQ(4, intraBytesWithMotor - intraBytesWithoutMotor);
    EXPECT_EQ(4, interBytesWithMotor - interBytesWithoutMotor);

    // Header field count (blackboxMainFields[], same table sendFieldDefinition() emits from)
    // must drop in lockstep with the serialized byte count, not just the byte count alone.
    EXPECT_EQ(4, headerFieldsWithMotor);
    EXPECT_EQ(0, headerFieldsWithoutMotor);
    EXPECT_EQ(headerFieldsWithMotor - headerFieldsWithoutMotor, intraBytesWithMotor - intraBytesWithoutMotor);

    blackboxConfigMutable()->fields_disabled_mask = 0;
    blackboxBuildConditionCache();
    currentPidProfile = NULL;
}
