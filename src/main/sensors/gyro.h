/*
 * This file is part of Cleanflight and Betaflight and EmuFlight.
 *
 * Cleanflight and Betaflight and EmuFlight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight and EmuFlight are distributed in the hope that they
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
#include "common/filter.h"
#include "common/time.h"
#include "common/kalman.h"

#include "drivers/accgyro/accgyro.h"
#include "drivers/bus.h"
#include "drivers/sensor.h"

#ifdef USE_GYRO_DATA_ANALYSE
#include "flight/gyroanalyse.h"
#endif

#include "flight/pid.h"

#include "pg/pg.h"

#define FILTER_FREQUENCY_MAX 4000 // maximum frequency for filter cutoffs (nyquist limit of 8K max sampling)

#ifdef USE_YAW_SPIN_RECOVERY
#define YAW_SPIN_RECOVERY_THRESHOLD_MIN 500
#define YAW_SPIN_RECOVERY_THRESHOLD_MAX 1950
#endif

#ifdef USE_SMITH_PREDICTOR
#define MAX_SMITH_SAMPLES 12 * 32
#endif // USE_SMITH_PREDICTOR

typedef enum gyroDetectionFlags_e {
    GYRO_NONE_MASK = 0,
    GYRO_1_MASK = BIT(0),
#if defined(USE_MULTI_GYRO)
    GYRO_2_MASK = BIT(1),
    GYRO_ALL_MASK = (GYRO_1_MASK | GYRO_2_MASK),
    GYRO_IDENTICAL_MASK = BIT(7), // All gyros are of the same hardware type
#endif
} gyroDetectionFlags_t;

typedef struct gyroCalibration_s {
    float sum[XYZ_AXIS_COUNT];
    stdev_t var[XYZ_AXIS_COUNT];
    int32_t cyclesRemaining;
} gyroCalibration_t;

typedef struct gyroSensor_s {
    gyroDev_t gyroDev;
    gyroCalibration_t calibration;
} gyroSensor_t;

#ifdef USE_SMITH_PREDICTOR
typedef struct smithPredictor_s {
    uint8_t samples;
    uint8_t idx;

    float data[MAX_SMITH_SAMPLES + 1]; // This is gonna be a ring buffer. Max of 8ms delay at 8khz

    pt1Filter_t smithPredictorFilter; // filter the smith predictor output for RPY

    float smithPredictorStrength;
} smithPredictor_t;
#endif // USE_SMITH_PREDICTOR

typedef struct gyro_s {
    uint16_t sampleRateHz;
    uint32_t targetLooptime;
    uint32_t sampleLooptime;
    float scale;
    float gyroADC[XYZ_AXIS_COUNT];     // aligned, calibrated, scaled, but unfiltered data from the sensor(s)
    float gyroADCf[XYZ_AXIS_COUNT];    // filtered gyro data

    gyroSensor_t gyroSensor1;
#ifdef USE_MULTI_GYRO
    gyroSensor_t gyroSensor2;
#endif

    gyroDev_t *rawSensorDev;           // pointer to the sensor providing the raw data for DEBUG_GYRO_RAW

    // lowpass gyro soft filter
    filterApplyFnPtr lowpassFilterApplyFn;
    ptnFilter_t lowpassFilter[XYZ_AXIS_COUNT];

    // notch filters
    filterApplyFnPtr notchFilter1ApplyFn;
    biquadFilter_t notchFilter1[XYZ_AXIS_COUNT];

    filterApplyFnPtr notchFilterDynApplyFn;
    biquadFilter_t notchFilterDyn[XYZ_AXIS_COUNT][XYZ_AXIS_COUNT];

    filterApplyFnPtr alphaBetaGammaApplyFn;
    alphaBetaGammaFilter_t alphaBetaGamma[XYZ_AXIS_COUNT];

    kalman_t kalmanFilterStateRate[XYZ_AXIS_COUNT];

#ifdef USE_GYRO_DATA_ANALYSE
    gyroAnalyseState_t gyroAnalyseState;
    float dynNotchQ;
#endif

#ifdef USE_SMITH_PREDICTOR
    smithPredictor_t smithPredictor[XYZ_AXIS_COUNT];
#endif // USE_SMITH_PREDICTOR

    uint16_t accSampleRateHz;
    uint8_t gyroToUse;
    uint8_t gyroDebugMode;
    bool gyroHasOverflowProtection;
    bool useDualGyroDebugging;
    flight_dynamics_index_t gyroDebugAxis;

#ifdef USE_DYN_LPF
    uint8_t dynLpfFilter;
    uint16_t dynLpfMin;
    uint16_t dynLpfMax;
    uint8_t dynLpfCurveExpo;
    uint16_t dynLpf2Gain;
    uint16_t dynLpf2Max;
#endif

#ifdef USE_GYRO_OVERFLOW_CHECK
    uint8_t overflowAxisMask;
#endif
} gyro_t;

extern gyro_t gyro;
extern uint8_t activePidLoopDenom;

enum {
    GYRO_OVERFLOW_CHECK_NONE = 0,
    GYRO_OVERFLOW_CHECK_YAW,
    GYRO_OVERFLOW_CHECK_ALL_AXES
};

enum {
    DYN_LPF_NONE = 0,
    DYN_LPF_PT1,
    DYN_LPF_PT2,
    DYN_LPF_PT3,
    DYN_LPF_PT4
};

typedef enum {
    YAW_SPIN_RECOVERY_OFF,
    YAW_SPIN_RECOVERY_ON,
    YAW_SPIN_RECOVERY_AUTO
} yawSpinRecoveryMode_e;

#define GYRO_CONFIG_USE_GYRO_1      0
#define GYRO_CONFIG_USE_GYRO_2      1
#define GYRO_CONFIG_USE_GYRO_BOTH_SIMPLE   2
#define GYRO_CONFIG_USE_GYRO_BOTH_VARIANCE  3

typedef struct gyroConfig_s {
    uint8_t  gyroMovementCalibrationThreshold; // people keep forgetting that moving model while init results in wrong gyro offsets. and then they never reset gyro. so this is now on by default.
    uint8_t  gyro_hardware_lpf;                // gyro DLPF setting

    uint8_t  gyro_high_fsr;
    uint8_t  gyro_use_32khz;
    uint8_t  gyro_to_use;

    uint16_t alpha;
    uint16_t abg_boost;
    uint16_t abg_half_life;

    uint16_t gyro_soft_notch_hz_1;
    uint16_t gyro_soft_notch_cutoff_1;
    int16_t  gyro_offset_yaw;
    uint8_t  checkOverflow;

    // Lowpass primary/secondary
    uint8_t  gyro_lowpass_type;

    uint8_t  yaw_spin_recovery;
    int16_t  yaw_spin_threshold;

    uint16_t gyroCalibrationDuration;   // Gyro calibration duration in 1/100 second

    uint16_t dyn_lpf_gyro_min_hz;
    uint8_t  dyn_lpf_gyro_width;
    uint8_t  dyn_lpf_gyro_gain;

    uint16_t dyn_notch_max_hz;
    uint16_t dyn_notch_q;
    uint16_t dyn_notch_min_hz;

#if defined(USE_GYRO_IMUF9001)
    uint16_t imuf_mode;
    uint16_t imuf_rate;
    uint16_t imuf_pitch_lpf_cutoff_hz;
    uint16_t imuf_roll_lpf_cutoff_hz;
    uint16_t imuf_yaw_lpf_cutoff_hz;
    uint16_t imuf_acc_lpf_cutoff_hz;
#endif
    uint16_t imuf_pitch_q;
    uint16_t imuf_roll_q;
    uint16_t imuf_yaw_q;
    uint16_t imuf_w;

    uint8_t  gyro_filter_debug_axis;

    uint8_t gyrosDetected; // What gyros should detection be attempted for on startup. Automatically set on first startup.
    uint8_t dyn_lpf_curve_expo; // set the curve for dynamic gyro lowpass filter

    uint8_t smithPredictorStrength;
    uint8_t smithPredictorDelay;
    uint16_t smithPredictorFilterHz;
} gyroConfig_t;

PG_DECLARE(gyroConfig_t, gyroConfig);

void gyroUpdate(void);
void gyroFiltering(timeUs_t currentTimeUs);
bool gyroGetAccumulationAverage(float *accumulation);
void gyroStartCalibration(bool isFirstArmingCalibration);
bool isFirstArmingGyroCalibrationRunning(void);
bool gyroIsCalibrationComplete(void);
void gyroReadTemperature(void);
int16_t gyroGetTemperature(void);
bool gyroOverflowDetected(void);
bool gyroYawSpinDetected(void);
uint16_t gyroAbsRateDps(int axis);
#ifdef USE_DYN_LPF
float dynThrottle(float throttle);
void dynLpfGyroUpdate(float cutoff[XYZ_AXIS_COUNT]);
uint16_t dynLpfGyroThrCut(float throttle);
float dynLpfGyroCutoff(uint16_t throttle, float dynlpf2_cutoff);
#endif
#ifdef USE_YAW_SPIN_RECOVERY
void initYawSpinRecovery(int maxYawRate);
#endif
