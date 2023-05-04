/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "common/axis.h"
#include "common/time.h"
#include "common/maths.h"
#include "common/filter.h"
#include "pg/pg.h"
#include "drivers/bus.h"
#include "drivers/sensor.h"

#ifdef USE_SMITH_PREDICTOR
#define MAX_SMITH_SAMPLES 12 * 32
#endif // USE_SMITH_PREDICTOR

#ifdef USE_YAW_SPIN_RECOVERY
#define YAW_SPIN_RECOVERY_THRESHOLD_MIN 500
#define YAW_SPIN_RECOVERY_THRESHOLD_MAX 1950
#endif

extern float vGyroStdDevModulus;
typedef enum {
    GYRO_NONE = 0,
    GYRO_DEFAULT,
    GYRO_MPU6050,
    GYRO_L3G4200D,
    GYRO_MPU3050,
    GYRO_L3GD20,
    GYRO_MPU6000,
    GYRO_MPU6500,
    GYRO_MPU9250,
    GYRO_ICM20601,
    GYRO_ICM20602,
    GYRO_ICM20608G,
    GYRO_ICM20649,
    GYRO_ICM20689,
    GYRO_BMI160,
    GYRO_IMUF9001,
    GYRO_FAKE
} gyroSensor_e;

typedef struct gyro_s {
    uint32_t targetLooptime;
    float gyroADCf[XYZ_AXIS_COUNT];
} gyro_t;

extern gyro_t gyro;

typedef enum {
    GYRO_OVERFLOW_CHECK_NONE = 0,
    GYRO_OVERFLOW_CHECK_YAW,
    GYRO_OVERFLOW_CHECK_ALL_AXES
} gyroOverflowCheck_e;

typedef enum {
    YAW_SPIN_RECOVERY_OFF,
    YAW_SPIN_RECOVERY_ON,
    YAW_SPIN_RECOVERY_AUTO
} yawSpinRecoveryMode_e;

#define GYRO_CONFIG_USE_GYRO_1      0
#define GYRO_CONFIG_USE_GYRO_2      1
#define GYRO_CONFIG_USE_GYRO_BOTH   2

typedef enum {
    FILTER_LOWPASS = 0,
    FILTER_LOWPASS2
} filterSlots;

#if defined(USE_GYRO_IMUF9001)
typedef enum {
    IMUF_RATE_32K = 0,
    IMUF_RATE_16K = 1,
    IMUF_RATE_8K = 2,
    IMUF_RATE_4K = 3,
    IMUF_RATE_2K = 4,
    IMUF_RATE_1K = 5
} imufRate_e;
#endif

#ifdef USE_SMITH_PREDICTOR
typedef struct smithPredictor_s {
    uint8_t enabled;
    uint8_t samples;
    uint8_t idx;

    float data[MAX_SMITH_SAMPLES + 1]; // This is gonna be a ring buffer. Max of 8ms delay at 8khz

    pt1Filter_t smithPredictorFilter; // filter the smith predictor output for RPY

    float smithPredictorStrength;
} smithPredictor_t;
#endif // USE_SMITH_PREDICTOR

typedef struct gyroConfig_s {
    uint8_t  gyro_align;                       // gyro alignment
    uint8_t  gyroMovementCalibrationThreshold; // people keep forgetting that moving model while init results in wrong gyro offsets. and then they never reset gyro. so this is now on by default.
    uint8_t  gyro_sync_denom;                  // Gyro sample divider
    uint8_t  gyro_hardware_lpf;                // gyro DLPF setting
    uint8_t  gyro_32khz_hardware_lpf;          // gyro 32khz DLPF setting

    uint8_t  gyro_high_fsr;
    uint8_t  gyro_use_32khz;
    uint8_t  gyro_to_use;

    uint16_t gyro_lowpass_hz[XYZ_AXIS_COUNT];
    uint16_t gyro_lowpass2_hz[XYZ_AXIS_COUNT];

    uint16_t gyro_ABG_alpha;
    uint16_t gyro_ABG_boost;
    uint8_t gyro_ABG_half_life;

    uint16_t gyro_soft_notch_hz_1;
    uint16_t gyro_soft_notch_cutoff_1;
    uint16_t gyro_soft_notch_hz_2;
    uint16_t gyro_soft_notch_cutoff_2;
    int16_t  gyro_offset_yaw;
    uint8_t  checkOverflow;

    // Lowpass primary/secondary
    uint8_t  gyro_lowpass_type;
    uint8_t  gyro_lowpass2_type;

    uint8_t  yaw_spin_recovery;
    int16_t  yaw_spin_threshold;

    uint16_t gyroCalibrationDuration;  // Gyro calibration duration in 1/100 second
    uint16_t dyn_notch_q;
    uint8_t dyn_notch_count;
    uint16_t dyn_notch_min_hz;
    uint16_t dyn_notch_max_hz;
#if defined(USE_GYRO_IMUF9001)
    uint16_t imuf_mode;
    uint16_t imuf_rate;
    uint16_t imuf_pitch_lpf_cutoff_hz;
    uint16_t imuf_roll_lpf_cutoff_hz;
    uint16_t imuf_yaw_lpf_cutoff_hz;
    uint16_t imuf_acc_lpf_cutoff_hz;
    uint8_t imuf_ptn_order;
#endif
    uint16_t imuf_pitch_q;
    uint16_t imuf_roll_q;
    uint16_t imuf_yaw_q;
    uint16_t imuf_w;

    uint8_t smithPredictorEnabled;
    uint8_t smithPredictorStrength;
    uint8_t smithPredictorDelay;
    uint8_t smithPredictorFilterHz;
} gyroConfig_t;

PG_DECLARE(gyroConfig_t, gyroConfig);

bool gyroInit(void);

void gyroInitFilters(void);

#ifdef USE_DMA_SPI_DEVICE
void gyroDmaSpiFinishRead(void);
void gyroDmaSpiStartRead(void);
#endif
void gyroUpdate(timeUs_t currentTimeUs);
bool gyroGetAverage(quaternion *vAverage);
const busDevice_t *gyroSensorBus(void);
struct mpuConfiguration_s;
const struct mpuConfiguration_s *gyroMpuConfiguration(void);
struct mpuDetectionResult_s;
const struct mpuDetectionResult_s *gyroMpuDetectionResult(void);
void gyroStartCalibration(bool isFirstArmingCalibration);
bool isFirstArmingGyroCalibrationRunning(void);
bool isGyroCalibrationComplete(void);
void gyroReadTemperature(void);
int16_t gyroGetTemperature(void);
int16_t gyroRateDps(int axis);
bool gyroOverflowDetected(void);
bool gyroYawSpinDetected(void);
uint16_t gyroAbsRateDps(int axis);
uint8_t gyroReadRegister(uint8_t whichSensor, uint8_t reg);
float applySmithPredictor(smithPredictor_t *smithPredictor, float gyroFiltered);
bool isDynamicFilterActive(void);
#ifdef USE_YAW_SPIN_RECOVERY
void initYawSpinRecovery(int maxYawRate);
#endif
