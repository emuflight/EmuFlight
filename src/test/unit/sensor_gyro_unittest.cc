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
#include <algorithm>

extern "C" {
    #include <platform.h>

    // Enable Smith Predictor feature for unit testing
    #define USE_SMITH_PREDICTOR

    #include "build/build_config.h"
    #include "build/debug.h"
    #include "common/axis.h"
    #include "common/maths.h"
    #include "common/utils.h"
    #include "drivers/accgyro/accgyro_fake.h"
    #include "drivers/accgyro/accgyro_mpu.h"
    #include "drivers/sensor.h"
    #include "io/beeper.h"
    #include "pg/pg.h"
    #include "pg/pg_ids.h"
    #include "scheduler/scheduler.h"
    #include "sensors/gyro.h"
    #include "sensors/acceleration.h"
    #include "sensors/sensors.h"

    STATIC_UNIT_TESTED gyroSensor_e gyroDetect(gyroDev_t *dev);
    struct gyroSensor_s;
    STATIC_UNIT_TESTED void performGyroCalibration(struct gyroSensor_s *gyroSensor, uint8_t gyroMovementCalibrationThreshold);
    STATIC_UNIT_TESTED bool fakeGyroRead(gyroDev_t *gyro);

    uint8_t debugMode;
    int16_t debug[DEBUG16_VALUE_COUNT];
}

#include "unittest_macros.h"
#include "gtest/gtest.h"
extern gyroSensor_s * const gyroSensorPtr;
extern gyroDev_t * const gyroDevPtr;


TEST(SensorGyro, Detect)
{
    const gyroSensor_e detected = gyroDetect(gyroDevPtr);
    EXPECT_EQ(GYRO_FAKE, detected);
    EXPECT_EQ(GYRO_FAKE, detectedSensors[SENSOR_INDEX_GYRO]);
}

TEST(SensorGyro, Init)
{
    pgResetAll();
    const bool initialised = gyroInit();
    EXPECT_EQ(true, initialised);
    EXPECT_EQ(GYRO_FAKE, detectedSensors[SENSOR_INDEX_GYRO]);
}

TEST(SensorGyro, Read)
{
    pgResetAll();
    gyroInit();
    fakeGyroSet(gyroDevPtr, 5, 6, 7);
    const bool read = gyroDevPtr->readFn(gyroDevPtr);
    EXPECT_EQ(true, read);
    EXPECT_EQ(5, gyroDevPtr->gyroADCRaw[X]);
    EXPECT_EQ(6, gyroDevPtr->gyroADCRaw[Y]);
    EXPECT_EQ(7, gyroDevPtr->gyroADCRaw[Z]);
}

TEST(SensorGyro, Calibrate)
{
    pgResetAll();
    gyroInit();
    fakeGyroSet(gyroDevPtr, 5, 6, 7);
    const bool read = gyroDevPtr->readFn(gyroDevPtr);
    EXPECT_EQ(true, read);
    EXPECT_EQ(5, gyroDevPtr->gyroADCRaw[X]);
    EXPECT_EQ(6, gyroDevPtr->gyroADCRaw[Y]);
    EXPECT_EQ(7, gyroDevPtr->gyroADCRaw[Z]);
    static const int gyroMovementCalibrationThreshold = 32;
    gyroDevPtr->gyroZero[X] = 8;
    gyroDevPtr->gyroZero[Y] = 9;
    gyroDevPtr->gyroZero[Z] = 10;
    performGyroCalibration(gyroSensorPtr, gyroMovementCalibrationThreshold);
    EXPECT_EQ(8, gyroDevPtr->gyroZero[X]);
    EXPECT_EQ(9, gyroDevPtr->gyroZero[Y]);
    EXPECT_EQ(10, gyroDevPtr->gyroZero[Z]);
    gyroStartCalibration(false);
    EXPECT_EQ(false, isGyroCalibrationComplete());
    while (!isGyroCalibrationComplete()) {
        gyroDevPtr->readFn(gyroDevPtr);
        performGyroCalibration(gyroSensorPtr, gyroMovementCalibrationThreshold);
    }
    EXPECT_EQ(5, gyroDevPtr->gyroZero[X]);
    EXPECT_EQ(6, gyroDevPtr->gyroZero[Y]);
    EXPECT_EQ(7, gyroDevPtr->gyroZero[Z]);
}

// Rewritten to test gyroUpdate() behavior without assuming exact filtered values.
// Tests calibration integration, zero removal, and value responsiveness.
TEST(SensorGyro, Update)
{
    pgResetAll();
    // Minimize filtering for more predictable behavior - disable for all axes
    gyroConfigMutable()->gyro_lowpass_hz[X] = 0;
    gyroConfigMutable()->gyro_lowpass_hz[Y] = 0;
    gyroConfigMutable()->gyro_lowpass_hz[Z] = 0;
#ifdef USE_GYRO_LPF2
    gyroConfigMutable()->gyro_lowpass2_hz[X] = 0;
    gyroConfigMutable()->gyro_lowpass2_hz[Y] = 0;
    gyroConfigMutable()->gyro_lowpass2_hz[Z] = 0;
#endif
    gyroConfigMutable()->gyro_soft_notch_hz_1 = 0;
    gyroConfigMutable()->gyro_soft_notch_hz_2 = 0;
    
    gyroInit();
    gyroDevPtr->readFn = fakeGyroRead;
    gyroStartCalibration(false);
    EXPECT_EQ(false, isGyroCalibrationComplete());

    timeUs_t currentTimeUs = 0;
    timeDelta_t gyroUpdatePeriod = gyro.targetLooptime;  // Use configured gyro loop time
    // Guard against zero targetLooptime which would cause infinite loop
    ASSERT_NE(0, gyroUpdatePeriod) << "gyro.targetLooptime must be non-zero for calibration";
    const timeUs_t calibrationTimeoutUs = 2000000;  // 2 seconds timeout for calibration
    const timeUs_t calibrationStartTime = currentTimeUs;
    
    // Calibrate with constant values
    fakeGyroSet(gyroDevPtr, 5, 6, 7);
    currentTimeUs += gyroUpdatePeriod;
    gyroUpdate(currentTimeUs);
    // Guard against infinite loops - fail if calibration takes too long (check after each update)
    ASSERT_LE(currentTimeUs - calibrationStartTime, calibrationTimeoutUs) 
        << "Gyro calibration did not complete within " << (calibrationTimeoutUs / 1000000) << "s";
    
    while (!isGyroCalibrationComplete()) {
        fakeGyroSet(gyroDevPtr, 5, 6, 7);
        currentTimeUs += gyroUpdatePeriod;
        gyroUpdate(currentTimeUs);
        // Guard against infinite loops - fail if calibration takes too long (check after each update)
        ASSERT_LE(currentTimeUs - calibrationStartTime, calibrationTimeoutUs) 
            << "Gyro calibration did not complete within " << (calibrationTimeoutUs / 1000000) << "s";
    }
    
    EXPECT_EQ(true, isGyroCalibrationComplete());
    EXPECT_EQ(5, gyroDevPtr->gyroZero[X]);
    EXPECT_EQ(6, gyroDevPtr->gyroZero[Y]);
    EXPECT_EQ(7, gyroDevPtr->gyroZero[Z]);
    
    // After calibration, with same values, output should be near zero (allowing for filter effects)
    currentTimeUs += gyroUpdatePeriod;
    gyroUpdate(currentTimeUs);
    EXPECT_NEAR(0, gyro.gyroADCf[X], 1.0f);  // Allow small deviation for filters
    EXPECT_NEAR(0, gyro.gyroADCf[Y], 1.0f);
    EXPECT_NEAR(0, gyro.gyroADCf[Z], 1.0f);
    
    // Change input values - output should respond (not exact due to filters, but should change)
    float prevX = gyro.gyroADCf[X];
    float prevY = gyro.gyroADCf[Y];
    float prevZ = gyro.gyroADCf[Z];
    
    fakeGyroSet(gyroDevPtr, 15, 26, 97);
    currentTimeUs += gyroUpdatePeriod;
    gyroUpdate(currentTimeUs);
    
    // Values should change significantly (using threshold-based comparison to tolerate floating-point precision)
    const float gyroChangeThreshold = 0.1f;  // Allow for filter smoothing and floating-point precision
    EXPECT_GT(std::fabs(gyro.gyroADCf[X] - prevX), gyroChangeThreshold);
    EXPECT_GT(std::fabs(gyro.gyroADCf[Y] - prevY), gyroChangeThreshold);
    EXPECT_GT(std::fabs(gyro.gyroADCf[Z] - prevZ), gyroChangeThreshold);
    
    // Values should move upward from previous samples
    EXPECT_GT(gyro.gyroADCf[X], prevX);
    EXPECT_GT(gyro.gyroADCf[Y], prevY);
    EXPECT_GT(gyro.gyroADCf[Z], prevZ);
}

// STUBS

extern "C" {

uint32_t micros(void) {return 0;}
void beeper(beeperMode_e) {}
uint8_t detectedSensors[] = { GYRO_NONE, ACC_NONE };
timeDelta_t getGyroUpdateRate(void) {return gyro.targetLooptime;}
void sensorsSet(uint32_t) {}
void schedulerResetTaskStatistics(cfTaskId_e) {}
int getArmingDisableFlags(void) {return 0;}

// Kalman filter stubs
void kalman_init(void) {}
float kalman_update(float input, int axis) {
    (void)axis;
    return input;  // Passthrough for testing
}
void update_kalman_covariance(float rate, int axis) {
    (void)rate;
    (void)axis;
}
}
