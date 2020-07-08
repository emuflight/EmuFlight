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

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>

#include "platform.h"

#include "build/debug.h"

#include "common/axis.h"
#include "common/maths.h"
#include "common/filter.h"

#include "config/feature.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "drivers/accgyro/accgyro.h"
#include "drivers/accgyro/accgyro_fake.h"
#include "drivers/accgyro/accgyro_mpu.h"
#include "drivers/accgyro/accgyro_mpu3050.h"
#include "drivers/accgyro/accgyro_mpu6050.h"
#include "drivers/accgyro/accgyro_mpu6500.h"
#include "drivers/accgyro/accgyro_spi_bmi160.h"
#include "drivers/accgyro/accgyro_spi_icm20649.h"
#include "drivers/accgyro/accgyro_spi_icm20689.h"
#include "drivers/accgyro/accgyro_spi_mpu6000.h"
#include "drivers/accgyro/accgyro_spi_mpu6500.h"
#include "drivers/accgyro/accgyro_spi_mpu9250.h"

#ifdef USE_GYRO_L3G4200D
#include "drivers/accgyro_legacy/accgyro_l3g4200d.h"
#endif

#ifdef USE_GYRO_L3GD20
#include "drivers/accgyro_legacy/accgyro_l3gd20.h"
#endif

#ifdef USE_GYRO_IMUF9001
#include "drivers/accgyro/accgyro_imuf9001.h"
#include "drivers/time.h"
#endif //USE_GYRO_IMUF9001
#include "drivers/accgyro/gyro_sync.h"
#include "drivers/bus_spi.h"
#include "drivers/dma_spi.h"
#include "drivers/io.h"
#include "drivers/time.h"
#include "drivers/accgyro/gyro_sync.h"
#include "drivers/bus_spi.h"
#include "drivers/io.h"

#include "fc/config.h"
#include "fc/runtime_config.h"

#include "io/beeper.h"
#include "io/statusindicator.h"

#include "scheduler/scheduler.h"

#include "fc/fc_rc.h"
#include "fc/rc_controls.h"
#include "rx/rx.h"

#include "sensors/boardalignment.h"
#include "sensors/gyro.h"
#ifdef USE_GYRO_DATA_ANALYSE
#include "sensors/gyroanalyse.h"
#endif
#include "sensors/sensors.h"
#ifdef USE_GYRO_IMUF9001

#include "drivers/accgyro/accgyro_imuf9001.h"
#endif

#ifdef USE_HARDWARE_REVISION_DETECTION
#include "hardware_revision.h"
#endif

#ifndef USE_GYRO_IMUF9001
#include "common/kalman.h"
#endif

#if ((FLASH_SIZE > 128) && (defined(USE_GYRO_SPI_ICM20601) || defined(USE_GYRO_SPI_ICM20689) || defined(USE_GYRO_SPI_MPU6500)))
#define USE_GYRO_SLEW_LIMITER
#endif

FAST_RAM_ZERO_INIT gyro_t gyro;
static FAST_RAM_ZERO_INIT uint8_t gyroDebugMode;

static uint8_t gyroToUse = 0;
static FAST_RAM_ZERO_INIT bool overflowDetected;

#ifdef USE_GYRO_OVERFLOW_CHECK
static FAST_RAM_ZERO_INIT uint8_t overflowAxisMask;
#endif

#ifdef USE_YAW_SPIN_RECOVERY
static FAST_RAM_ZERO_INIT bool yawSpinDetected;
#endif

static FAST_RAM_ZERO_INIT float accumulatedMeasurements[XYZ_AXIS_COUNT];
static FAST_RAM_ZERO_INIT float gyroPrevious[XYZ_AXIS_COUNT];
static FAST_RAM_ZERO_INIT timeUs_t accumulatedMeasurementTimeUs;
static FAST_RAM_ZERO_INIT timeUs_t accumulationLastTimeSampledUs;

float FAST_RAM_ZERO_INIT vGyroStdDevModulus;


static FAST_RAM_ZERO_INIT int16_t gyroSensorTemperature;


static bool gyroHasOverflowProtection = true;

typedef struct gyroCalibration_s {
    float sum[XYZ_AXIS_COUNT];
    stdev_t var[XYZ_AXIS_COUNT];
    int32_t cyclesRemaining;
} gyroCalibration_t;

bool firstArmingCalibrationWasStarted = false;

typedef union gyroLowpassFilter_u {
    pt1Filter_t pt1FilterState;
    biquadFilter_t biquadFilterState;
} gyroLowpassFilter_t;

typedef struct gyroSensor_s {
    gyroDev_t gyroDev;
    gyroCalibration_t calibration;

    // lowpass gyro soft filter
    filterApplyFnPtr lowpassFilterApplyFn;
    gyroLowpassFilter_t lowpassFilter[XYZ_AXIS_COUNT];

    // lowpass2 gyro soft filter
    filterApplyFnPtr lowpass2FilterApplyFn;
    gyroLowpassFilter_t lowpass2Filter[XYZ_AXIS_COUNT];

    // notch filters
    filterApplyFnPtr notchFilter1ApplyFn;
    biquadFilter_t notchFilter1[XYZ_AXIS_COUNT];

    filterApplyFnPtr notchFilter2ApplyFn;
    biquadFilter_t notchFilter2[XYZ_AXIS_COUNT];

    filterApplyFnPtr notchFilterDynApplyFn;
    biquadFilter_t notchFilterDyn[XYZ_AXIS_COUNT][XYZ_AXIS_COUNT];

    // overflow and recovery
    timeUs_t overflowTimeUs;
    bool overflowDetected;
#ifdef USE_YAW_SPIN_RECOVERY
    timeUs_t yawSpinTimeUs;
    bool yawSpinDetected;
#endif // USE_YAW_SPIN_RECOVERY

#ifdef USE_GYRO_DATA_ANALYSE
    gyroAnalyseState_t gyroAnalyseState;
    float dynNotchQ;
#endif
} gyroSensor_t;

STATIC_UNIT_TESTED FAST_RAM_ZERO_INIT gyroSensor_t gyroSensor1;
#ifdef USE_DUAL_GYRO
STATIC_UNIT_TESTED FAST_RAM_ZERO_INIT gyroSensor_t gyroSensor2;
#endif

#ifdef UNIT_TEST
STATIC_UNIT_TESTED gyroSensor_t * const gyroSensorPtr = &gyroSensor1;
STATIC_UNIT_TESTED gyroDev_t * const gyroDevPtr = &gyroSensor1.gyroDev;
#endif

static void gyroInitSensorFilters(gyroSensor_t *gyroSensor);
static void gyroInitLowpassFilterLpf(gyroSensor_t *gyroSensor, int slot, int type);

#define DEBUG_GYRO_CALIBRATION 3

#ifdef STM32F10X
#define GYRO_SYNC_DENOM_DEFAULT 8
#elif defined(USE_GYRO_SPI_MPU6000) || defined(USE_GYRO_SPI_MPU6500) || defined(USE_GYRO_SPI_ICM20601) || defined(USE_GYRO_SPI_ICM20649) \
   || defined(USE_GYRO_SPI_ICM20689)
#define GYRO_SYNC_DENOM_DEFAULT 1
#else
#define GYRO_SYNC_DENOM_DEFAULT 3
#endif

#define GYRO_OVERFLOW_TRIGGER_THRESHOLD 31980  // 97.5% full scale (1950dps for 2000dps gyro)
#define GYRO_OVERFLOW_RESET_THRESHOLD 30340    // 92.5% full scale (1850dps for 2000dps gyro)

PG_REGISTER_WITH_RESET_TEMPLATE(gyroConfig_t, gyroConfig, PG_GYRO_CONFIG, 6);

#ifndef GYRO_CONFIG_USE_GYRO_DEFAULT
#define GYRO_CONFIG_USE_GYRO_DEFAULT GYRO_CONFIG_USE_GYRO_1
#endif

#ifdef USE_GYRO_IMUF9001
PG_RESET_TEMPLATE(gyroConfig_t, gyroConfig,
    .gyro_align = ALIGN_DEFAULT,
    .gyroCalibrationDuration = 125,        // 1.25 seconds
    .gyroMovementCalibrationThreshold = 48,
    .gyro_sync_denom = 1,
    .gyro_hardware_lpf = GYRO_HARDWARE_LPF_NORMAL,
    .gyro_32khz_hardware_lpf = GYRO_32KHZ_HARDWARE_LPF_NORMAL,
    .gyro_lowpass_type = FILTER_PT1,
    .gyro_lowpass_hz[ROLL] = 0,
    .gyro_lowpass_hz[PITCH] = 0,
    .gyro_lowpass_hz[YAW] = 0,
    .gyro_lowpass2_type = FILTER_PT1,
    .gyro_lowpass2_hz[ROLL] = 0,
    .gyro_lowpass2_hz[PITCH] = 0,
    .gyro_lowpass2_hz[YAW] = 0,
    .gyro_high_fsr = false,
    .gyro_use_32khz = true,
    .gyro_to_use = GYRO_CONFIG_USE_GYRO_DEFAULT,
    .gyro_soft_notch_hz_1 = 0,
    .gyro_soft_notch_cutoff_1 = 0,
    .gyro_soft_notch_hz_2 = 0,
    .gyro_soft_notch_cutoff_2 = 0,
    .checkOverflow = GYRO_OVERFLOW_CHECK_ALL_AXES,
    .yaw_spin_recovery = true,
    .yaw_spin_threshold = 1950,
    .dyn_notch_q_factor = 300,
    .dyn_notch_min_hz = 150,
    .dyn_notch_max_hz = 600,
    .imuf_mode = GTBCM_GYRO_ACC_FILTER_F,
    .imuf_rate = IMUF_RATE_16K,
    .imuf_roll_q = IMUF_DEFAULT_ROLL_Q,
    .imuf_pitch_q = IMUF_DEFAULT_PITCH_Q,
    .imuf_yaw_q = IMUF_DEFAULT_YAW_Q,
    .imuf_w = IMUF_DEFAULT_W,
    .imuf_roll_lpf_cutoff_hz = IMUF_DEFAULT_LPF_HZ,
    .imuf_pitch_lpf_cutoff_hz = IMUF_DEFAULT_LPF_HZ,
    .imuf_yaw_lpf_cutoff_hz = IMUF_DEFAULT_LPF_HZ,
   	.imuf_acc_lpf_cutoff_hz = IMUF_DEFAULT_ACC_LPF_HZ,
    .imuf_sharpness = 2500,
    .gyro_offset_yaw = 0,
);
#else //USE_GYRO_IMUF9001
PG_RESET_TEMPLATE(gyroConfig_t, gyroConfig,
    .gyro_align = ALIGN_DEFAULT,
    .gyroCalibrationDuration = 125,        // 1.25 seconds
    .gyroMovementCalibrationThreshold = 48,
    .gyro_sync_denom = GYRO_SYNC_DENOM_DEFAULT,
    .gyro_hardware_lpf = GYRO_HARDWARE_LPF_NORMAL,
    .gyro_32khz_hardware_lpf = GYRO_32KHZ_HARDWARE_LPF_NORMAL,
    .gyro_lowpass_type = FILTER_PT1,
    .gyro_lowpass_hz[ROLL] = 115,
    .gyro_lowpass_hz[PITCH] = 115,
    .gyro_lowpass_hz[YAW] = 105,
    .gyro_lowpass2_type = FILTER_PT1,
    .gyro_lowpass2_hz[ROLL] = 0,
    .gyro_lowpass2_hz[PITCH] = 0,
    .gyro_lowpass2_hz[YAW] = 0,
    .gyro_high_fsr = false,
    .gyro_use_32khz = false,
    .gyro_to_use = GYRO_CONFIG_USE_GYRO_DEFAULT,
    .gyro_soft_notch_hz_1 = 0,
    .gyro_soft_notch_cutoff_1 = 0,
    .gyro_soft_notch_hz_2 = 0,
    .gyro_soft_notch_cutoff_2 = 0,
    .checkOverflow = GYRO_OVERFLOW_CHECK_ALL_AXES,
    .imuf_roll_q = 3000,
    .imuf_pitch_q = 3000,
    .imuf_yaw_q = 3000,
    .imuf_w = 32,
    .imuf_sharpness = 2500,
    .gyro_offset_yaw = 0,
    .yaw_spin_recovery = true,
    .yaw_spin_threshold = 1950,
    .dyn_notch_q_factor = 250,
    .dyn_notch_min_hz = 150,
    .dyn_notch_max_hz = 600,
);
#endif //USE_GYRO_IMUF9001


const busDevice_t *gyroSensorBus(void)
{
#ifdef USE_DUAL_GYRO
    if (gyroToUse == GYRO_CONFIG_USE_GYRO_2) {
        return &gyroSensor2.gyroDev.bus;
    } else {
        return &gyroSensor1.gyroDev.bus;
    }
#else
    return &gyroSensor1.gyroDev.bus;
#endif
}

#ifdef USE_GYRO_REGISTER_DUMP
const busDevice_t *gyroSensorBusByDevice(uint8_t whichSensor)
{
#ifdef USE_DUAL_GYRO
    if (whichSensor == GYRO_CONFIG_USE_GYRO_2) {
        return &gyroSensor2.gyroDev.bus;
    } else {
        return &gyroSensor1.gyroDev.bus;
    }
#else
    UNUSED(whichSensor);
    return &gyroSensor1.gyroDev.bus;
#endif
}
#endif // USE_GYRO_REGISTER_DUMP

const mpuConfiguration_t *gyroMpuConfiguration(void)
{
#ifdef USE_DUAL_GYRO
    if (gyroToUse == GYRO_CONFIG_USE_GYRO_2) {
        return &gyroSensor2.gyroDev.mpuConfiguration;
    } else {
        return &gyroSensor1.gyroDev.mpuConfiguration;
    }
#else
    return &gyroSensor1.gyroDev.mpuConfiguration;
#endif
}

const mpuDetectionResult_t *gyroMpuDetectionResult(void)
{
#ifdef USE_DUAL_GYRO
    if (gyroToUse == GYRO_CONFIG_USE_GYRO_2) {
        return &gyroSensor2.gyroDev.mpuDetectionResult;
    } else {
        return &gyroSensor1.gyroDev.mpuDetectionResult;
    }
#else
    return &gyroSensor1.gyroDev.mpuDetectionResult;
#endif
}

STATIC_UNIT_TESTED gyroSensor_e gyroDetect(gyroDev_t *dev)
{
    gyroSensor_e gyroHardware = GYRO_DEFAULT;

    switch (gyroHardware) {
    case GYRO_DEFAULT:
        FALLTHROUGH;

#ifdef USE_GYRO_MPU6050
    case GYRO_MPU6050:
        if (mpu6050GyroDetect(dev)) {
            gyroHardware = GYRO_MPU6050;
#ifdef GYRO_MPU6050_ALIGN
            dev->gyroAlign = GYRO_MPU6050_ALIGN;
#endif
            break;
        }
        FALLTHROUGH;
#endif

#ifdef USE_GYRO_L3G4200D
    case GYRO_L3G4200D:
        if (l3g4200dDetect(dev)) {
            gyroHardware = GYRO_L3G4200D;
#ifdef GYRO_L3G4200D_ALIGN
            dev->gyroAlign = GYRO_L3G4200D_ALIGN;
#endif
            break;
        }
        FALLTHROUGH;
#endif

#ifdef USE_GYRO_MPU3050
    case GYRO_MPU3050:
        if (mpu3050Detect(dev)) {
            gyroHardware = GYRO_MPU3050;
#ifdef GYRO_MPU3050_ALIGN
            dev->gyroAlign = GYRO_MPU3050_ALIGN;
#endif
            break;
        }
        FALLTHROUGH;
#endif

#ifdef USE_GYRO_L3GD20
    case GYRO_L3GD20:
        if (l3gd20Detect(dev)) {
            gyroHardware = GYRO_L3GD20;
#ifdef GYRO_L3GD20_ALIGN
            dev->gyroAlign = GYRO_L3GD20_ALIGN;
#endif
            break;
        }
        FALLTHROUGH;
#endif

#ifdef USE_GYRO_SPI_MPU6000
    case GYRO_MPU6000:
        if (mpu6000SpiGyroDetect(dev)) {
            gyroHardware = GYRO_MPU6000;
#ifdef GYRO_MPU6000_ALIGN
            dev->gyroAlign = GYRO_MPU6000_ALIGN;
#endif
            break;
        }
        FALLTHROUGH;
#endif

#if defined(USE_GYRO_MPU6500) || defined(USE_GYRO_SPI_MPU6500)
    case GYRO_MPU6500:
    case GYRO_ICM20601:
    case GYRO_ICM20602:
    case GYRO_ICM20608G:
#ifdef USE_GYRO_SPI_MPU6500
        if (mpu6500GyroDetect(dev) || mpu6500SpiGyroDetect(dev)) {
#else
        if (mpu6500GyroDetect(dev)) {
#endif
            switch (dev->mpuDetectionResult.sensor) {
            case MPU_9250_SPI:
                gyroHardware = GYRO_MPU9250;
                break;
            case ICM_20601_SPI:
                gyroHardware = GYRO_ICM20601;
                break;
            case ICM_20602_SPI:
                gyroHardware = GYRO_ICM20602;
                break;
            case ICM_20608_SPI:
                gyroHardware = GYRO_ICM20608G;
                break;
            default:
                gyroHardware = GYRO_MPU6500;
            }
#ifdef GYRO_MPU6500_ALIGN
            dev->gyroAlign = GYRO_MPU6500_ALIGN;
#endif
            break;
        }
        FALLTHROUGH;
#endif

#ifdef USE_GYRO_SPI_MPU9250
    case GYRO_MPU9250:
        if (mpu9250SpiGyroDetect(dev)) {
            gyroHardware = GYRO_MPU9250;
#ifdef GYRO_MPU9250_ALIGN
            dev->gyroAlign = GYRO_MPU9250_ALIGN;
#endif
            break;
        }
        FALLTHROUGH;
#endif

#ifdef USE_GYRO_SPI_ICM20649
    case GYRO_ICM20649:
        if (icm20649SpiGyroDetect(dev)) {
            gyroHardware = GYRO_ICM20649;
#ifdef GYRO_ICM20649_ALIGN
            dev->gyroAlign = GYRO_ICM20649_ALIGN;
#endif
            break;
        }
        FALLTHROUGH;
#endif

#ifdef USE_GYRO_SPI_ICM20689
    case GYRO_ICM20689:
        if (icm20689SpiGyroDetect(dev)) {
            gyroHardware = GYRO_ICM20689;
#ifdef GYRO_ICM20689_ALIGN
            dev->gyroAlign = GYRO_ICM20689_ALIGN;
#endif
            break;
        }
        FALLTHROUGH;
#endif

#ifdef USE_ACCGYRO_BMI160
    case GYRO_BMI160:
        if (bmi160SpiGyroDetect(dev)) {
            gyroHardware = GYRO_BMI160;
#ifdef GYRO_BMI160_ALIGN
            dev->gyroAlign = GYRO_BMI160_ALIGN;
#endif
            break;
        }
        FALLTHROUGH;
#endif

#ifdef USE_GYRO_IMUF9001
    case GYRO_IMUF9001:
        if (imufSpiGyroDetect(dev)) {
            gyroHardware = GYRO_IMUF9001;
            break;
        }
        FALLTHROUGH;
#endif //USE_GYRO_IMUF9001

#ifdef USE_FAKE_GYRO
    case GYRO_FAKE:
        if (fakeGyroDetect(dev)) {
            gyroHardware = GYRO_FAKE;
            break;
        }
        FALLTHROUGH;
#endif

    default:
        gyroHardware = GYRO_NONE;
    }

    if (gyroHardware != GYRO_NONE) {
        detectedSensors[SENSOR_INDEX_GYRO] = gyroHardware;
        sensorsSet(SENSOR_GYRO);
    }


    return gyroHardware;
}

static bool gyroInitSensor(gyroSensor_t *gyroSensor)
{
#ifdef USE_GYRO_DATA_ANALYSE
    gyroSensor->dynNotchQ = gyroConfig()->dyn_notch_q_factor / 100.0f;
#endif

    gyroSensor->gyroDev.gyro_high_fsr = gyroConfig()->gyro_high_fsr;

#if defined(USE_GYRO_MPU6050) || defined(USE_GYRO_MPU3050) || defined(USE_GYRO_MPU6500) || defined(USE_GYRO_SPI_MPU6500) || defined(USE_GYRO_SPI_MPU6000) \
 || defined(USE_ACC_MPU6050) || defined(USE_GYRO_SPI_MPU9250) || defined(USE_GYRO_SPI_ICM20601) || defined(USE_GYRO_SPI_ICM20649) || defined(USE_GYRO_SPI_ICM20689) || defined(USE_GYRO_IMUF9001)

    mpuDetect(&gyroSensor->gyroDev);
    mpuResetFn = gyroSensor->gyroDev.mpuConfiguration.resetFn; // must be set after mpuDetect
#endif

    const gyroSensor_e gyroHardware = gyroDetect(&gyroSensor->gyroDev);
    gyroSensor->gyroDev.gyroHardware = gyroHardware;
    if (gyroHardware == GYRO_NONE) {
        return false;
    }

    switch (gyroHardware) {
    case GYRO_MPU6500:
    case GYRO_MPU9250:
    case GYRO_ICM20601:
    case GYRO_ICM20602:
    case GYRO_ICM20608G:
    case GYRO_ICM20689:
    case GYRO_IMUF9001:
        // do nothing, as gyro supports 32kHz
        break;
    default:
        // gyro does not support 32kHz
        gyroConfigMutable()->gyro_use_32khz = false;
        break;
    }

    // Must set gyro targetLooptime before gyroDev.init and initialisation of filters
    gyro.targetLooptime = gyroSetSampleRate(&gyroSensor->gyroDev, gyroConfig()->gyro_hardware_lpf, gyroConfig()->gyro_sync_denom, gyroConfig()->gyro_use_32khz);
    gyroSensor->gyroDev.hardware_lpf = gyroConfig()->gyro_hardware_lpf;
    gyroSensor->gyroDev.hardware_32khz_lpf = gyroConfig()->gyro_32khz_hardware_lpf;
    gyroSensor->gyroDev.initFn(&gyroSensor->gyroDev);


#ifndef USE_GYRO_IMUF9001
    if (gyroConfig()->gyro_align != ALIGN_DEFAULT) {
        gyroSensor->gyroDev.gyroAlign = gyroConfig()->gyro_align;
    }
#endif //!USE_GYRO_IMUF9001

    // As new gyros are supported, be sure to add them below based on whether they are subject to the overflow/inversion bug
    // Any gyro not explicitly defined will default to not having built-in overflow protection as a safe alternative.
    switch (gyroHardware) {
    case GYRO_NONE:    // Won't ever actually get here, but included to account for all gyro types
    case GYRO_DEFAULT:
    case GYRO_FAKE:
    case GYRO_MPU6050:
    case GYRO_L3G4200D:
    case GYRO_MPU3050:
    case GYRO_L3GD20:
    case GYRO_BMI160:
    case GYRO_MPU6000:
    case GYRO_MPU6500:
    case GYRO_MPU9250:
        gyroSensor->gyroDev.gyroHasOverflowProtection = true;
        break;

    case GYRO_ICM20601:
    case GYRO_ICM20602:
    case GYRO_ICM20608G:
    case GYRO_ICM20649:  // we don't actually know if this is affected, but as there are currently no flight controllers using it we err on the side of caution
    case GYRO_ICM20689:
        gyroSensor->gyroDev.gyroHasOverflowProtection = false;
        break;

    default:
        gyroSensor->gyroDev.gyroHasOverflowProtection = false;  // default catch for newly added gyros until proven to be unaffected
        break;
    }

    gyroInitSensorFilters(gyroSensor);

#ifdef USE_GYRO_DATA_ANALYSE
    gyroDataAnalyseStateInit(&gyroSensor->gyroAnalyseState, gyro.targetLooptime);
#endif

    return true;
}

bool gyroInit(void)
{
#ifdef USE_GYRO_OVERFLOW_CHECK
    overflowAxisMask = gyroConfig()->checkOverflow;
#endif //USE_GYRO_OVERFLOW_CHECK

    switch (debugMode) {
    case DEBUG_FFT:
    case DEBUG_FFT_FREQ:
    case DEBUG_GYRO_RAW:
    case DEBUG_GYRO_SCALED:
    case DEBUG_GYRO_FILTERED:
        gyroDebugMode = debugMode;
        break;
    default:
        // debugMode is not gyro-related
        gyroDebugMode = DEBUG_NONE;
        break;
    }
    firstArmingCalibrationWasStarted = false;

    bool ret = false;
    memset(&gyro, 0, sizeof(gyro));
    gyroToUse = gyroConfig()->gyro_to_use;

#if defined(USE_DUAL_GYRO) && defined(GYRO_1_CS_PIN)
    if (gyroToUse == GYRO_CONFIG_USE_GYRO_1 || gyroToUse == GYRO_CONFIG_USE_GYRO_BOTH) {
        gyroSensor1.gyroDev.bus.busdev_u.spi.csnPin = IOGetByTag(IO_TAG(GYRO_1_CS_PIN));
        IOInit(gyroSensor1.gyroDev.bus.busdev_u.spi.csnPin, OWNER_MPU_CS, RESOURCE_INDEX(0));
        IOHi(gyroSensor1.gyroDev.bus.busdev_u.spi.csnPin); // Ensure device is disabled, important when two devices are on the same bus.
        IOConfigGPIO(gyroSensor1.gyroDev.bus.busdev_u.spi.csnPin, SPI_IO_CS_CFG);
    }
#endif

#if defined(USE_DUAL_GYRO) && defined(GYRO_2_CS_PIN)
    if (gyroToUse == GYRO_CONFIG_USE_GYRO_2 || gyroToUse == GYRO_CONFIG_USE_GYRO_BOTH) {
        gyroSensor2.gyroDev.bus.busdev_u.spi.csnPin = IOGetByTag(IO_TAG(GYRO_2_CS_PIN));
        IOInit(gyroSensor2.gyroDev.bus.busdev_u.spi.csnPin, OWNER_MPU_CS, RESOURCE_INDEX(1));
        IOHi(gyroSensor2.gyroDev.bus.busdev_u.spi.csnPin); // Ensure device is disabled, important when two devices are on the same bus.
        IOConfigGPIO(gyroSensor2.gyroDev.bus.busdev_u.spi.csnPin, SPI_IO_CS_CFG);
    }
#endif

    gyroSensor1.gyroDev.gyroAlign = ALIGN_DEFAULT;

#if defined(GYRO_1_EXTI_PIN)
    gyroSensor1.gyroDev.mpuIntExtiTag =  IO_TAG(GYRO_1_EXTI_PIN);
#elif defined(MPU_INT_EXTI)
    gyroSensor1.gyroDev.mpuIntExtiTag =  IO_TAG(MPU_INT_EXTI);
#elif defined(USE_HARDWARE_REVISION_DETECTION)
    gyroSensor1.gyroDev.mpuIntExtiTag =  selectMPUIntExtiConfigByHardwareRevision();
#else
    gyroSensor1.gyroDev.mpuIntExtiTag =  IO_TAG_NONE;
#endif // GYRO_1_EXTI_PIN
#ifdef USE_DUAL_GYRO
#ifdef GYRO_1_ALIGN
    gyroSensor1.gyroDev.gyroAlign = GYRO_1_ALIGN;
#endif
    gyroSensor1.gyroDev.bus.bustype = BUSTYPE_SPI;
    spiBusSetInstance(&gyroSensor1.gyroDev.bus, GYRO_1_SPI_INSTANCE);
    if (gyroToUse == GYRO_CONFIG_USE_GYRO_1 || gyroToUse == GYRO_CONFIG_USE_GYRO_BOTH) {
        ret = gyroInitSensor(&gyroSensor1);
        if (!ret) {
            return false; // TODO handle failure of first gyro detection better. - Perhaps update the config to use second gyro then indicate a new failure mode and reboot.
        }
        gyroHasOverflowProtection =  gyroHasOverflowProtection && gyroSensor1.gyroDev.gyroHasOverflowProtection;
    }
#else // USE_DUAL_GYRO
    ret = gyroInitSensor(&gyroSensor1);
    gyroHasOverflowProtection =  gyroHasOverflowProtection && gyroSensor1.gyroDev.gyroHasOverflowProtection;
#endif // USE_DUAL_GYRO

#ifdef USE_DUAL_GYRO

    gyroSensor2.gyroDev.gyroAlign = ALIGN_DEFAULT;

#if defined(GYRO_2_EXTI_PIN)
    gyroSensor2.gyroDev.mpuIntExtiTag =  IO_TAG(GYRO_2_EXTI_PIN);
#elif defined(USE_HARDWARE_REVISION_DETECTION)
    gyroSensor2.gyroDev.mpuIntExtiTag =  selectMPUIntExtiConfigByHardwareRevision();
#else
    gyroSensor2.gyroDev.mpuIntExtiTag =  IO_TAG_NONE;
#endif // GYRO_2_EXTI_PIN
#ifdef GYRO_2_ALIGN
    gyroSensor2.gyroDev.gyroAlign = GYRO_2_ALIGN;
#endif
    gyroSensor2.gyroDev.bus.bustype = BUSTYPE_SPI;
    spiBusSetInstance(&gyroSensor2.gyroDev.bus, GYRO_2_SPI_INSTANCE);
    if (gyroToUse == GYRO_CONFIG_USE_GYRO_2 || gyroToUse == GYRO_CONFIG_USE_GYRO_BOTH) {
        ret = gyroInitSensor(&gyroSensor2);
        if (!ret) {
            return false; // TODO handle failure of second gyro detection better. - Perhaps update the config to use first gyro then indicate a new failure mode and reboot.
        }
        gyroHasOverflowProtection =  gyroHasOverflowProtection && gyroSensor2.gyroDev.gyroHasOverflowProtection;
    }
#endif // USE_DUAL_GYRO

#ifdef USE_DUAL_GYRO
    // Only allow using both gyros simultaneously if they are the same hardware type.
    // If the user selected "BOTH" and they are not the same type, then reset to using only the first gyro.
    if (gyroToUse == GYRO_CONFIG_USE_GYRO_BOTH) {
        if (gyroSensor1.gyroDev.gyroHardware != gyroSensor2.gyroDev.gyroHardware) {
            gyroToUse = GYRO_CONFIG_USE_GYRO_1;
            gyroConfigMutable()->gyro_to_use = GYRO_CONFIG_USE_GYRO_1;
            detectedSensors[SENSOR_INDEX_GYRO] = gyroSensor1.gyroDev.gyroHardware;
            sensorsSet(SENSOR_GYRO);

        }
    }
#endif // USE_DUAL_GYRO
    return ret;
}

void gyroInitLowpassFilterLpf(gyroSensor_t *gyroSensor, int slot, int type)
{
    filterApplyFnPtr *lowpassFilterApplyFn;
    gyroLowpassFilter_t *lowpassFilter = NULL;
    uint16_t lpfHz[3];

    switch (slot) {
    case FILTER_LOWPASS:
        lowpassFilterApplyFn = &gyroSensor->lowpassFilterApplyFn;
        lowpassFilter = gyroSensor->lowpassFilter;
        lpfHz[ROLL] = gyroConfig()->gyro_lowpass_hz[ROLL];
        lpfHz[PITCH] = gyroConfig()->gyro_lowpass_hz[PITCH];
        lpfHz[YAW] = gyroConfig()->gyro_lowpass_hz[YAW];
        break;

    case FILTER_LOWPASS2:
        lowpassFilterApplyFn = &gyroSensor->lowpass2FilterApplyFn;
        lowpassFilter = gyroSensor->lowpass2Filter;
        lpfHz[ROLL] = gyroConfig()->gyro_lowpass2_hz[ROLL];
        lpfHz[PITCH] = gyroConfig()->gyro_lowpass2_hz[PITCH];
        lpfHz[YAW] = gyroConfig()->gyro_lowpass2_hz[YAW];
        break;

    default:
        return;
    }

    // Establish some common constants
    const uint32_t gyroFrequencyNyquist = 1000000 / 2 / gyro.targetLooptime;
    const float gyroDt = gyro.targetLooptime * 1e-6f;

    // Gain could be calculated a little later as it is specific to the pt1/bqrcf2/fkf branches
    // Dereference the pointer to null before checking valid cutoff and filter
    // type. It will be overridden for positive cases.
    *lowpassFilterApplyFn = &nullFilterApply;

    // If lowpass cutoff has been specified and is less than the Nyquist frequency
    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
    float gain = pt1FilterGain(lpfHz[axis], gyroDt);
    if (lpfHz[axis] && lpfHz[axis] <= gyroFrequencyNyquist) {
        switch (type) {
        case FILTER_PT1:
            *lowpassFilterApplyFn = (filterApplyFnPtr) pt1FilterApply;
            pt1FilterInit(&lowpassFilter[axis].pt1FilterState, gain);
            break;
        case FILTER_BIQUAD:
            *lowpassFilterApplyFn = (filterApplyFnPtr) biquadFilterApply;
            biquadFilterInitLPF(&lowpassFilter[axis].biquadFilterState, lpfHz[axis], gyro.targetLooptime);
            break;
        }
    }
}
}

static uint16_t calculateNyquistAdjustedNotchHz(uint16_t notchHz, uint16_t notchCutoffHz)
{
    const uint32_t gyroFrequencyNyquist = 1000000 / 2 / gyro.targetLooptime;
    if (notchHz > gyroFrequencyNyquist) {
        if (notchCutoffHz < gyroFrequencyNyquist) {
            notchHz = gyroFrequencyNyquist;
        } else {
            notchHz = 0;
        }
    }

    return notchHz;
}

#ifdef USE_GYRO_SLEW_LIMITER
void gyroInitSlewLimiter(gyroSensor_t *gyroSensor) {

    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        gyroSensor->gyroDev.gyroADCRawPrevious[axis] = 0;
    }
}
#endif

static void gyroInitFilterNotch1(gyroSensor_t *gyroSensor, uint16_t notchHz, uint16_t notchCutoffHz)
{
    gyroSensor->notchFilter1ApplyFn = nullFilterApply;

    notchHz = calculateNyquistAdjustedNotchHz(notchHz, notchCutoffHz);

    if (notchHz != 0 && notchCutoffHz != 0) {
        gyroSensor->notchFilter1ApplyFn = (filterApplyFnPtr)biquadFilterApply;
        const float notchQ = filterGetNotchQ(notchHz, notchCutoffHz);
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            biquadFilterInit(&gyroSensor->notchFilter1[axis], notchHz, gyro.targetLooptime, notchQ, FILTER_NOTCH);
        }
    }
}

static void gyroInitFilterNotch2(gyroSensor_t *gyroSensor, uint16_t notchHz, uint16_t notchCutoffHz)
{
    gyroSensor->notchFilter2ApplyFn = nullFilterApply;

    notchHz = calculateNyquistAdjustedNotchHz(notchHz, notchCutoffHz);

    if (notchHz != 0 && notchCutoffHz != 0) {
        gyroSensor->notchFilter2ApplyFn = (filterApplyFnPtr)biquadFilterApply;
        const float notchQ = filterGetNotchQ(notchHz, notchCutoffHz);
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            biquadFilterInit(&gyroSensor->notchFilter2[axis], notchHz, gyro.targetLooptime, notchQ, FILTER_NOTCH);
        }
    }
}

#ifdef USE_GYRO_DATA_ANALYSE
static bool isDynamicFilterActive(void)
{
    return feature(FEATURE_DYNAMIC_FILTER);
}

static void gyroInitFilterDynamicNotch(gyroSensor_t *gyroSensor)
{
    gyroSensor->notchFilterDynApplyFn = nullFilterApply;

    if (isDynamicFilterActive()) {
        gyroSensor->notchFilterDynApplyFn = (filterApplyFnPtr)biquadFilterApplyDF1; // must be this function, not DF2
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            for (int axis2 = 0; axis2 < XYZ_AXIS_COUNT; axis2++) {
                biquadFilterInit(&gyroSensor->notchFilterDyn[axis][axis2], 400, gyro.targetLooptime, gyroSensor->dynNotchQ, FILTER_NOTCH);
            }
        }
    }
}
#endif

static void gyroInitSensorFilters(gyroSensor_t *gyroSensor)
{
#if defined(USE_GYRO_SLEW_LIMITER)
    gyroInitSlewLimiter(gyroSensor);
#endif

#ifndef USE_GYRO_IMUF9001
    kalman_init();
#endif //USE_GYRO_IMUF9001

    gyroInitLowpassFilterLpf(
      gyroSensor,
      FILTER_LOWPASS,
      gyroConfig()->gyro_lowpass_type
    );

    gyroInitLowpassFilterLpf(
      gyroSensor,
      FILTER_LOWPASS2,
      gyroConfig()->gyro_lowpass2_type
    );

    gyroInitFilterNotch1(gyroSensor, gyroConfig()->gyro_soft_notch_hz_1, gyroConfig()->gyro_soft_notch_cutoff_1);
    gyroInitFilterNotch2(gyroSensor, gyroConfig()->gyro_soft_notch_hz_2, gyroConfig()->gyro_soft_notch_cutoff_2);
#ifdef USE_GYRO_DATA_ANALYSE
    gyroInitFilterDynamicNotch(gyroSensor);
#endif
}

void gyroInitFilters(void)
{
    gyroInitSensorFilters(&gyroSensor1);
#ifdef USE_DUAL_GYRO
    gyroInitSensorFilters(&gyroSensor2);
#endif
}

FAST_CODE bool isGyroSensorCalibrationComplete(const gyroSensor_t *gyroSensor)
{
    return gyroSensor->calibration.cyclesRemaining == 0;
}

FAST_CODE bool isGyroCalibrationComplete(void)
{
#ifdef USE_DUAL_GYRO
    switch (gyroToUse) {
        default:
        case GYRO_CONFIG_USE_GYRO_1: {
            return isGyroSensorCalibrationComplete(&gyroSensor1);
        }
        case GYRO_CONFIG_USE_GYRO_2: {
            return isGyroSensorCalibrationComplete(&gyroSensor2);
        }
        case GYRO_CONFIG_USE_GYRO_BOTH: {
            return isGyroSensorCalibrationComplete(&gyroSensor1) && isGyroSensorCalibrationComplete(&gyroSensor2);
        }
    }
#else
    return isGyroSensorCalibrationComplete(&gyroSensor1);
#endif
}

static bool isOnFinalGyroCalibrationCycle(const gyroCalibration_t *gyroCalibration)
{
    return gyroCalibration->cyclesRemaining == 1;
}

static int32_t gyroCalculateCalibratingCycles(void)
{
    return (gyroConfig()->gyroCalibrationDuration * 10000) / gyro.targetLooptime;
}

static bool isOnFirstGyroCalibrationCycle(const gyroCalibration_t *gyroCalibration)
{
    return gyroCalibration->cyclesRemaining == gyroCalculateCalibratingCycles();
}

static void gyroSetCalibrationCycles(gyroSensor_t *gyroSensor)
{
    #ifdef USE_GYRO_IMUF9001
    imufStartCalibration();
    #endif
    gyroSensor->calibration.cyclesRemaining = gyroCalculateCalibratingCycles();
}

void gyroStartCalibration(bool isFirstArmingCalibration)
{
    if (!(isFirstArmingCalibration && firstArmingCalibrationWasStarted)) {
        gyroSetCalibrationCycles(&gyroSensor1);
#ifdef USE_DUAL_GYRO
        gyroSetCalibrationCycles(&gyroSensor2);
#endif

        if (isFirstArmingCalibration) {
            firstArmingCalibrationWasStarted = true;
        }
    }
}

bool isFirstArmingGyroCalibrationRunning(void)
{
    return firstArmingCalibrationWasStarted && !isGyroCalibrationComplete();
}

STATIC_UNIT_TESTED void performGyroCalibration(gyroSensor_t *gyroSensor, uint8_t gyroMovementCalibrationThreshold)
{
    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        // Reset g[axis] at start of calibration
        if (isOnFirstGyroCalibrationCycle(&gyroSensor->calibration)) {
            gyroSensor->calibration.sum[axis] = 0.0f;
            devClear(&gyroSensor->calibration.var[axis]);
            // gyroZero is set to zero until calibration complete
            gyroSensor->gyroDev.gyroZero[axis] = 0.0f;
        }

        // Sum up CALIBRATING_GYRO_TIME_US readings
        gyroSensor->calibration.sum[axis] += gyroSensor->gyroDev.gyroADCRaw[axis];
        devPush(&gyroSensor->calibration.var[axis], gyroSensor->gyroDev.gyroADCRaw[axis]);

        if (isOnFinalGyroCalibrationCycle(&gyroSensor->calibration)) {
            const float stddev = devStandardDeviation(&gyroSensor->calibration.var[axis]);
            // DEBUG_GYRO_CALIBRATION records the standard deviation of roll
            // into the spare field - debug[3], in DEBUG_GYRO_RAW
            if (axis == X) {
                DEBUG_SET(DEBUG_GYRO_RAW, DEBUG_GYRO_CALIBRATION, lrintf(stddev));
            }

            // check deviation and startover in case the model was moved
            if (gyroMovementCalibrationThreshold && stddev > gyroMovementCalibrationThreshold) {
                gyroSetCalibrationCycles(gyroSensor);
                return;
            }

            // please take care with exotic boardalignment !!
            gyroSensor->gyroDev.gyroZero[axis] = gyroSensor->calibration.sum[axis] / gyroCalculateCalibratingCycles();
            if (axis == Z) {
              gyroSensor->gyroDev.gyroZero[axis] -= ((float)gyroConfig()->gyro_offset_yaw / 100);
            }
        }
    }

    if (isOnFinalGyroCalibrationCycle(&gyroSensor->calibration)) {
        schedulerResetTaskStatistics(TASK_SELF); // so calibration cycles do not pollute tasks statistics
        if (!firstArmingCalibrationWasStarted || (getArmingDisableFlags() & ~ARMING_DISABLED_CALIBRATING) == 0) {
            // calculate gyro noise standard deviation modulus
            static quaternion vStdDev = VECTOR_INITIALIZE;
            vStdDev.x =  devStandardDeviation(&gyroSensor->calibration.var[X]);
            vStdDev.y =  devStandardDeviation(&gyroSensor->calibration.var[Y]);
            vStdDev.z =  devStandardDeviation(&gyroSensor->calibration.var[Z]);
            vGyroStdDevModulus =  quaternionModulus(&vStdDev) / 1000.0f;

            beeper(BEEPER_GYRO_CALIBRATED);
        }
    }
    --gyroSensor->calibration.cyclesRemaining;

}

#if defined(USE_GYRO_SLEW_LIMITER)
FAST_CODE_NOINLINE int32_t gyroSlewLimiter(gyroSensor_t *gyroSensor, int axis)
{
    int32_t ret = (int32_t)gyroSensor->gyroDev.gyroADCRaw[axis];
    if (gyroConfig()->checkOverflow || gyroHasOverflowProtection) {
        // don't use the slew limiter if overflow checking is on or gyro is not subject to overflow bug
        return ret;
    }
    if (abs(ret - gyroSensor->gyroDev.gyroADCRawPrevious[axis]) > (1<<14)) {
        // there has been a large change in value, so assume overflow has occurred and return the previous value
        ret = gyroSensor->gyroDev.gyroADCRawPrevious[axis];
    } else {
        gyroSensor->gyroDev.gyroADCRawPrevious[axis] = ret;
    }
    return ret;
}
#endif

#ifdef USE_GYRO_OVERFLOW_CHECK
static FAST_CODE_NOINLINE void handleOverflow(gyroSensor_t *gyroSensor, timeUs_t currentTimeUs)
{
    const float gyroOverflowResetRate = GYRO_OVERFLOW_RESET_THRESHOLD * gyroSensor->gyroDev.scale;
    if ((fabsf(gyroSensor->gyroDev.gyroADCf[X]) < gyroOverflowResetRate)
          && (fabsf(gyroSensor->gyroDev.gyroADCf[Y]) < gyroOverflowResetRate)
          && (fabsf(gyroSensor->gyroDev.gyroADCf[Z]) < gyroOverflowResetRate)) {
        // if we have 50ms of consecutive OK gyro vales, then assume yaw readings are OK again and reset overflowDetected
        // reset requires good OK values on all axes
        if (cmpTimeUs(currentTimeUs, gyroSensor->overflowTimeUs) > 50000) {
            gyroSensor->overflowDetected = false;
        }
    } else {
        // not a consecutive OK value, so reset the overflow time
        gyroSensor->overflowTimeUs = currentTimeUs;
    }
}

static FAST_CODE void checkForOverflow(gyroSensor_t *gyroSensor, timeUs_t currentTimeUs)
{
    // check for overflow to handle Yaw Spin To The Moon (YSTTM)
    // ICM gyros are specified to +/- 2000 deg/sec, in a crash they can go out of spec.
    // This can cause an overflow and sign reversal in the output.
    // Overflow and sign reversal seems to result in a gyro value of +1996 or -1996.
    if (gyroSensor->overflowDetected) {
        handleOverflow(gyroSensor, currentTimeUs);
    } else {
#ifndef SIMULATOR_BUILD
        // check for overflow in the axes set in overflowAxisMask
        gyroOverflow_e overflowCheck = GYRO_OVERFLOW_NONE;
        const float gyroOverflowTriggerRate = GYRO_OVERFLOW_TRIGGER_THRESHOLD * gyroSensor->gyroDev.scale;
        if (fabsf(gyroSensor->gyroDev.gyroADCf[X]) > gyroOverflowTriggerRate) {
            overflowCheck |= GYRO_OVERFLOW_X;
        }
        if (fabsf(gyroSensor->gyroDev.gyroADCf[Y]) > gyroOverflowTriggerRate) {
            overflowCheck |= GYRO_OVERFLOW_Y;
        }
        if (fabsf(gyroSensor->gyroDev.gyroADCf[Z]) > gyroOverflowTriggerRate) {
            overflowCheck |= GYRO_OVERFLOW_Z;
        }
        if (overflowCheck & overflowAxisMask) {
            gyroSensor->overflowDetected = true;
            gyroSensor->overflowTimeUs = currentTimeUs;
#ifdef USE_YAW_SPIN_RECOVERY
            gyroSensor->yawSpinDetected = false;
#endif // USE_YAW_SPIN_RECOVERY
        }
#endif // SIMULATOR_BUILD
    }
}
#endif // USE_GYRO_OVERFLOW_CHECK

#ifdef USE_YAW_SPIN_RECOVERY
static FAST_CODE_NOINLINE void handleYawSpin(gyroSensor_t *gyroSensor, timeUs_t currentTimeUs)
{
    const float yawSpinResetRate = gyroConfig()->yaw_spin_threshold - 100.0f;
    if (fabsf(gyroSensor->gyroDev.gyroADCf[Z]) < yawSpinResetRate) {
        // testing whether 20ms of consecutive OK gyro yaw values is enough
        if (cmpTimeUs(currentTimeUs, gyroSensor->yawSpinTimeUs) > 20000) {
            gyroSensor->yawSpinDetected = false;
        }
    } else {
        // reset the yaw spin time
        gyroSensor->yawSpinTimeUs = currentTimeUs;
    }
}

static FAST_CODE void checkForYawSpin(gyroSensor_t *gyroSensor, timeUs_t currentTimeUs)
{
    // if not in overflow mode, handle yaw spins above threshold
#ifdef USE_GYRO_OVERFLOW_CHECK
    if (gyroSensor->overflowDetected) {
        gyroSensor->yawSpinDetected = false;
        return;
    }
#endif // USE_GYRO_OVERFLOW_CHECK

    if (gyroSensor->yawSpinDetected) {
        handleYawSpin(gyroSensor, currentTimeUs);
    } else {
#ifndef SIMULATOR_BUILD
        // check for spin on yaw axis only
         if (fabsf(gyroSensor->gyroDev.gyroADCf[Z]) > gyroConfig()->yaw_spin_threshold) {
            gyroSensor->yawSpinDetected = true;
            gyroSensor->yawSpinTimeUs = currentTimeUs;
        }
#endif // SIMULATOR_BUILD
    }
}
#endif // USE_YAW_SPIN_RECOVERY

#define GYRO_FILTER_FUNCTION_NAME filterGyro
#define GYRO_FILTER_DEBUG_SET(...)
#include "gyro_filter_impl.h"
#undef GYRO_FILTER_FUNCTION_NAME
#undef GYRO_FILTER_DEBUG_SET

#define GYRO_FILTER_FUNCTION_NAME filterGyroDebug
#define GYRO_FILTER_DEBUG_SET DEBUG_SET
#include "gyro_filter_impl.h"
#undef GYRO_FILTER_FUNCTION_NAME
#undef GYRO_FILTER_DEBUG_SET

static FAST_CODE void dynamicGyroNotchFiltersUpdate(gyroSensor_t* gyroSensor) {
    if (gyroSensor->gyroAnalyseState.filterUpdateExecute) {
        const uint8_t axis = gyroSensor->gyroAnalyseState.filterUpdateAxis;
        const uint16_t frequency = gyroSensor->gyroAnalyseState.filterUpdateFrequency;

        biquadFilterUpdate(&gyroSensor->notchFilterDyn[0][axis], frequency, gyro.targetLooptime, gyroSensor->dynNotchQ, FILTER_NOTCH);
        biquadFilterUpdate(&gyroSensor->notchFilterDyn[1][axis], frequency, gyro.targetLooptime, gyroSensor->dynNotchQ, FILTER_NOTCH);
        biquadFilterUpdate(&gyroSensor->notchFilterDyn[2][axis], frequency, gyro.targetLooptime, gyroSensor->dynNotchQ, FILTER_NOTCH);
    }
}

static FAST_CODE_NOINLINE void gyroUpdateSensor(gyroSensor_t* gyroSensor, timeUs_t currentTimeUs)
{
    #ifndef USE_DMA_SPI_DEVICE
        if (!gyroSensor->gyroDev.readFn(&gyroSensor->gyroDev)) {
        return;
    }
    #endif
    gyroSensor->gyroDev.dataReady = false;

    const timeDelta_t sampleDeltaUs = currentTimeUs - accumulationLastTimeSampledUs;
    accumulationLastTimeSampledUs = currentTimeUs;
    accumulatedMeasurementTimeUs += sampleDeltaUs;
#ifdef USE_GYRO_IMUF9001
    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        // NOTE: this branch optimized for when there is no gyro debugging, ensure it is kept in step with non-optimized branch
        DEBUG_SET(DEBUG_GYRO_SCALED, axis, lrintf(gyroSensor->gyroDev.gyroADCf[axis]));
        if (!gyroSensor->overflowDetected) {
            // integrate using trapezium rule to avoid bias
            accumulatedMeasurements[axis] += 0.5f * (gyroPrevious[axis] + gyroSensor->gyroDev.gyroADCf[axis]) * sampleDeltaUs;
            gyroPrevious[axis] = gyroSensor->gyroDev.gyroADCf[axis];
        }
    }
    if (!isGyroSensorCalibrationComplete(gyroSensor)) {
        performGyroCalibration(gyroSensor, gyroConfig()->gyroMovementCalibrationThreshold);
        // Reset gyro values to zero to prevent other code from using uncalibrated data
        gyroSensor->gyroDev.gyroADCf[X] = 0.0f;
        gyroSensor->gyroDev.gyroADCf[Y] = 0.0f;
        gyroSensor->gyroDev.gyroADCf[Z] = 0.0f;
        // still calibrating, so no need to further process gyro data
    }
#else
    if (isGyroSensorCalibrationComplete(gyroSensor)) {
        // move 16-bit gyro data into 32-bit variables to avoid overflows in calculations

#if defined(USE_GYRO_SLEW_LIMITER)
        gyroSensor->gyroDev.gyroADC[X] = gyroSlewLimiter(gyroSensor, X) - gyroSensor->gyroDev.gyroZero[X];
        gyroSensor->gyroDev.gyroADC[Y] = gyroSlewLimiter(gyroSensor, Y) - gyroSensor->gyroDev.gyroZero[Y];
        gyroSensor->gyroDev.gyroADC[Z] = gyroSlewLimiter(gyroSensor, Z) - gyroSensor->gyroDev.gyroZero[Z];
#else
        gyroSensor->gyroDev.gyroADC[X] = gyroSensor->gyroDev.gyroADCRaw[X] - gyroSensor->gyroDev.gyroZero[X];
        gyroSensor->gyroDev.gyroADC[Y] = gyroSensor->gyroDev.gyroADCRaw[Y] - gyroSensor->gyroDev.gyroZero[Y];
        gyroSensor->gyroDev.gyroADC[Z] = gyroSensor->gyroDev.gyroADCRaw[Z] - gyroSensor->gyroDev.gyroZero[Z];
#endif

        alignSensors(gyroSensor->gyroDev.gyroADC, gyroSensor->gyroDev.gyroAlign);
    } else {
        performGyroCalibration(gyroSensor, gyroConfig()->gyroMovementCalibrationThreshold);
        // still calibrating, so no need to further process gyro data
        return;
    }
#endif

    if (gyroDebugMode == DEBUG_NONE) {
        filterGyro(gyroSensor);
    } else {
        filterGyroDebug(gyroSensor);
    }

#ifdef USE_GYRO_OVERFLOW_CHECK
    if (gyroConfig()->checkOverflow && !gyroHasOverflowProtection) {
        checkForOverflow(gyroSensor, currentTimeUs);
    }
#endif

#ifdef USE_YAW_SPIN_RECOVERY
    if (gyroConfig()->yaw_spin_recovery) {
        checkForYawSpin(gyroSensor, currentTimeUs);
    }
#endif

#ifdef USE_GYRO_DATA_ANALYSE
    if (isDynamicFilterActive()) {
        gyroDataAnalyse(&gyroSensor->gyroAnalyseState);
        dynamicGyroNotchFiltersUpdate(gyroSensor);
    }
#endif


#if (!defined(USE_GYRO_OVERFLOW_CHECK) && !defined(USE_YAW_SPIN_RECOVERY))
    UNUSED(currentTimeUs);
#endif
}

#ifdef USE_DMA_SPI_DEVICE
FAST_CODE_NOINLINE void gyroDmaSpiFinishRead(void)
{
    //called by dma callback
    mpuGyroDmaSpiReadFinish(&gyroSensor1.gyroDev);

}

FAST_CODE_NOINLINE void gyroDmaSpiStartRead(void)
{
    //called by exti
    mpuGyroDmaSpiReadStart(&gyroSensor1.gyroDev);
}
#endif

FAST_CODE_NOINLINE void gyroUpdate(timeUs_t currentTimeUs)
{
    const timeDelta_t sampleDeltaUs = currentTimeUs - accumulationLastTimeSampledUs;
    accumulationLastTimeSampledUs = currentTimeUs;
    accumulatedMeasurementTimeUs += sampleDeltaUs;

#ifdef USE_DUAL_GYRO
    switch (gyroToUse) {
    case GYRO_CONFIG_USE_GYRO_1:
        gyroUpdateSensor(&gyroSensor1, currentTimeUs);
        if (isGyroSensorCalibrationComplete(&gyroSensor1)) {
            gyro.gyroADCf[X] = gyroSensor1.gyroDev.gyroADCf[X];
            gyro.gyroADCf[Y] = gyroSensor1.gyroDev.gyroADCf[Y];
            gyro.gyroADCf[Z] = gyroSensor1.gyroDev.gyroADCf[Z];
#ifdef USE_GYRO_OVERFLOW_CHECK
            overflowDetected = gyroSensor1.overflowDetected;
#endif
#ifdef USE_YAW_SPIN_RECOVERY
            yawSpinDetected = gyroSensor1.yawSpinDetected;
#endif
        }
        DEBUG_SET(DEBUG_DUAL_GYRO_RAW, 0, gyroSensor1.gyroDev.gyroADCRaw[X]);
        DEBUG_SET(DEBUG_DUAL_GYRO_RAW, 1, gyroSensor1.gyroDev.gyroADCRaw[Y]);
        DEBUG_SET(DEBUG_DUAL_GYRO, 0, lrintf(gyroSensor1.gyroDev.gyroADCf[X]));
        DEBUG_SET(DEBUG_DUAL_GYRO, 1, lrintf(gyroSensor1.gyroDev.gyroADCf[Y]));
        DEBUG_SET(DEBUG_DUAL_GYRO_COMBINE, 0, lrintf(gyro.gyroADCf[X]));
        DEBUG_SET(DEBUG_DUAL_GYRO_COMBINE, 1, lrintf(gyro.gyroADCf[Y]));
        break;
    case GYRO_CONFIG_USE_GYRO_2:
        gyroUpdateSensor(&gyroSensor2, currentTimeUs);
        if (isGyroSensorCalibrationComplete(&gyroSensor2)) {
            gyro.gyroADCf[X] = gyroSensor2.gyroDev.gyroADCf[X];
            gyro.gyroADCf[Y] = gyroSensor2.gyroDev.gyroADCf[Y];
            gyro.gyroADCf[Z] = gyroSensor2.gyroDev.gyroADCf[Z];
#ifdef USE_GYRO_OVERFLOW_CHECK
            overflowDetected = gyroSensor2.overflowDetected;
#endif
#ifdef USE_YAW_SPIN_RECOVERY
            yawSpinDetected = gyroSensor2.yawSpinDetected;
#endif
        }
        DEBUG_SET(DEBUG_DUAL_GYRO_RAW, 2, gyroSensor2.gyroDev.gyroADCRaw[X]);
        DEBUG_SET(DEBUG_DUAL_GYRO_RAW, 3, gyroSensor2.gyroDev.gyroADCRaw[Y]);
        DEBUG_SET(DEBUG_DUAL_GYRO, 2, lrintf(gyroSensor2.gyroDev.gyroADCf[X]));
        DEBUG_SET(DEBUG_DUAL_GYRO, 3, lrintf(gyroSensor2.gyroDev.gyroADCf[Y]));
        DEBUG_SET(DEBUG_DUAL_GYRO_COMBINE, 2, lrintf(gyro.gyroADCf[X]));
        DEBUG_SET(DEBUG_DUAL_GYRO_COMBINE, 3, lrintf(gyro.gyroADCf[Y]));
        break;
    case GYRO_CONFIG_USE_GYRO_BOTH:
        gyroUpdateSensor(&gyroSensor1, currentTimeUs);
        gyroUpdateSensor(&gyroSensor2, currentTimeUs);
        if (isGyroSensorCalibrationComplete(&gyroSensor1) && isGyroSensorCalibrationComplete(&gyroSensor2)) {
            gyro.gyroADCf[X] = (gyroSensor1.gyroDev.gyroADCf[X] + gyroSensor2.gyroDev.gyroADCf[X]) * 0.5f;
            gyro.gyroADCf[Y] = (gyroSensor1.gyroDev.gyroADCf[Y] + gyroSensor2.gyroDev.gyroADCf[Y]) * 0.5f;
            gyro.gyroADCf[Z] = (gyroSensor1.gyroDev.gyroADCf[Z] + gyroSensor2.gyroDev.gyroADCf[Z]) * 0.5f;
#ifdef USE_GYRO_OVERFLOW_CHECK
            overflowDetected = gyroSensor1.overflowDetected || gyroSensor2.overflowDetected;
#endif
#ifdef USE_YAW_SPIN_RECOVERY
            yawSpinDetected = gyroSensor1.yawSpinDetected || gyroSensor2.yawSpinDetected;
#endif
        }

        DEBUG_SET(DEBUG_DUAL_GYRO_RAW, 0, gyroSensor1.gyroDev.gyroADCRaw[X]);
        DEBUG_SET(DEBUG_DUAL_GYRO_RAW, 1, gyroSensor1.gyroDev.gyroADCRaw[Y]);
        DEBUG_SET(DEBUG_DUAL_GYRO, 0, lrintf(gyroSensor1.gyroDev.gyroADCf[X]));
        DEBUG_SET(DEBUG_DUAL_GYRO, 1, lrintf(gyroSensor1.gyroDev.gyroADCf[Y]));
        DEBUG_SET(DEBUG_DUAL_GYRO_RAW, 2, gyroSensor2.gyroDev.gyroADCRaw[X]);
        DEBUG_SET(DEBUG_DUAL_GYRO_RAW, 3, gyroSensor2.gyroDev.gyroADCRaw[Y]);
        DEBUG_SET(DEBUG_DUAL_GYRO, 2, lrintf(gyroSensor2.gyroDev.gyroADCf[X]));
        DEBUG_SET(DEBUG_DUAL_GYRO, 3, lrintf(gyroSensor2.gyroDev.gyroADCf[Y]));
        DEBUG_SET(DEBUG_DUAL_GYRO_COMBINE, 1, lrintf(gyro.gyroADCf[X]));
        DEBUG_SET(DEBUG_DUAL_GYRO_COMBINE, 2, lrintf(gyro.gyroADCf[Y]));
        DEBUG_SET(DEBUG_DUAL_GYRO_DIFF, 0, lrintf(gyroSensor1.gyroDev.gyroADCf[X] - gyroSensor2.gyroDev.gyroADCf[X]));
        DEBUG_SET(DEBUG_DUAL_GYRO_DIFF, 1, lrintf(gyroSensor1.gyroDev.gyroADCf[Y] - gyroSensor2.gyroDev.gyroADCf[Y]));
        DEBUG_SET(DEBUG_DUAL_GYRO_DIFF, 2, lrintf(gyroSensor1.gyroDev.gyroADCf[Z] - gyroSensor2.gyroDev.gyroADCf[Z]));
        break;
    }
#else
    gyroUpdateSensor(&gyroSensor1, currentTimeUs);
    gyro.gyroADCf[X] = gyroSensor1.gyroDev.gyroADCf[X];
    gyro.gyroADCf[Y] = gyroSensor1.gyroDev.gyroADCf[Y];
    gyro.gyroADCf[Z] = gyroSensor1.gyroDev.gyroADCf[Z];

#ifdef USE_GYRO_OVERFLOW_CHECK
    overflowDetected = gyroSensor1.overflowDetected;
#endif
#ifdef USE_YAW_SPIN_RECOVERY
    yawSpinDetected = gyroSensor1.yawSpinDetected;
#endif
#endif

    if (!overflowDetected) {
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            // integrate using trapezium rule to avoid bias
            accumulatedMeasurements[axis] += 0.5f * (gyroPrevious[axis] + gyro.gyroADCf[axis]) * sampleDeltaUs;
            gyroPrevious[axis] = gyro.gyroADCf[axis];
        }
    }
}

bool gyroGetAverage(quaternion *vAverage) {
    if (accumulatedMeasurementTimeUs > 0) {
        vAverage->w = 0;
        vAverage->x = DEGREES_TO_RADIANS(accumulatedMeasurements[X] / accumulatedMeasurementTimeUs);
        vAverage->y = DEGREES_TO_RADIANS(accumulatedMeasurements[Y] / accumulatedMeasurementTimeUs);
        vAverage->z = DEGREES_TO_RADIANS(accumulatedMeasurements[Z] / accumulatedMeasurementTimeUs);

        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            accumulatedMeasurements[axis] = 0.0f;
        }
        accumulatedMeasurementTimeUs = 0;
        return true;
    } else {
        quaternionInitVector(vAverage);
        return false;
    }
}

int16_t gyroReadSensorTemperature(gyroSensor_t gyroSensor)
{
    if (gyroSensor.gyroDev.temperatureFn) {
        gyroSensor.gyroDev.temperatureFn(&gyroSensor.gyroDev, &gyroSensor.gyroDev.temperature);
    }
    return gyroSensor.gyroDev.temperature;
}

void gyroReadTemperature(void)
{
    switch (gyroToUse) {
    case GYRO_CONFIG_USE_GYRO_1:
        gyroSensorTemperature = gyroReadSensorTemperature(gyroSensor1);
        break;

#ifdef USE_DUAL_GYRO
    case GYRO_CONFIG_USE_GYRO_2:
        gyroSensorTemperature = gyroReadSensorTemperature(gyroSensor2);
        break;

    case GYRO_CONFIG_USE_GYRO_BOTH:
        gyroSensorTemperature = MAX(gyroReadSensorTemperature(gyroSensor1), gyroReadSensorTemperature(gyroSensor2));
        break;
#endif // USE_DUAL_GYRO
    }
}

int16_t gyroGetTemperature(void)
{
    return gyroSensorTemperature;
}

int16_t gyroRateDps(int axis)
{
#ifdef USE_DUAL_GYRO
    if (gyroToUse == GYRO_CONFIG_USE_GYRO_2) {
        return lrintf(gyro.gyroADCf[axis] / gyroSensor2.gyroDev.scale);
    } else {
        return lrintf(gyro.gyroADCf[axis] / gyroSensor1.gyroDev.scale);
    }
#elif defined(USE_GYRO_IMUF9001)
    return lrintf(gyro.gyroADCf[axis]);
#else
    return lrintf(gyro.gyroADCf[axis] / gyroSensor1.gyroDev.scale);
#endif
}

bool gyroOverflowDetected(void)
{
#ifdef USE_GYRO_OVERFLOW_CHECK
    return overflowDetected;
#else
    return false;
#endif // USE_GYRO_OVERFLOW_CHECK
}

#ifdef USE_YAW_SPIN_RECOVERY
bool gyroYawSpinDetected(void)
{
    return yawSpinDetected;
}
#endif // USE_YAW_SPIN_RECOVERY

uint16_t gyroAbsRateDps(int axis)
{
    return fabsf(gyro.gyroADCf[axis]);
}

#ifdef USE_GYRO_REGISTER_DUMP
uint8_t gyroReadRegister(uint8_t whichSensor, uint8_t reg)
{
    return mpuGyroReadRegister(gyroSensorBusByDevice(whichSensor), reg);
}
#endif // USE_GYRO_REGISTER_DUMP
