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

#include "platform.h"

#include "common/axis.h"
#include "drivers/exti.h"
#include "drivers/bus.h"
#include "drivers/sensor.h"
#include "drivers/accgyro/accgyro_mpu.h"
#include "sensors/gyro.h"
#pragma GCC diagnostic push
#if defined(SIMULATOR_BUILD) && defined(SIMULATOR_MULTITHREAD)
#include <pthread.h>
#endif

#ifndef MPU_I2C_INSTANCE
#define MPU_I2C_INSTANCE I2C_DEVICE
#endif

#define GYRO_HARDWARE_LPF_NORMAL       0
#define GYRO_HARDWARE_LPF_EXPERIMENTAL 1
#define GYRO_HARDWARE_LPF_1KHZ_SAMPLE  2

#define GYRO_32KHZ_HARDWARE_LPF_NORMAL       0
#define GYRO_32KHZ_HARDWARE_LPF_EXPERIMENTAL 1

#define GYRO_LPF_256HZ      0
#define GYRO_LPF_188HZ      1
#define GYRO_LPF_98HZ       2
#define GYRO_LPF_42HZ       3
#define GYRO_LPF_20HZ       4
#define GYRO_LPF_10HZ       5
#define GYRO_LPF_5HZ        6
#define GYRO_LPF_NONE       7

//This optimizes the frequencies instead of calculating them 
//in the case of 1100 and 9000, they would divide as irrational numbers.
#define GYRO_RATE_1_kHz     1000.0f
#define GYRO_RATE_1100_Hz   909.09f
#define GYRO_RATE_3200_Hz   312.5f
#define GYRO_RATE_8_kHz     125.0f
#define GYRO_RATE_9_kHz     111.11f
#define GYRO_RATE_16_kHz    64.0f
#define GYRO_RATE_32_kHz    32.0f

typedef struct gyroDev_s {
#if defined(SIMULATOR_BUILD) && defined(SIMULATOR_MULTITHREAD)
    pthread_mutex_t lock;
#endif
    sensorGyroInitFuncPtr initFn;                             // initialize function
    sensorGyroReadFuncPtr readFn;                             // read 3 axis data function
    sensorGyroReadDataFuncPtr temperatureFn;                  // read temperature if available
    extiCallbackRec_t exti;
    busDevice_t bus;
    float scale;                                            // scalefactor
    float gyroZero[XYZ_AXIS_COUNT];
    float gyroADC[XYZ_AXIS_COUNT];                        // gyro data after calibration and alignment
    float gyroADCf[XYZ_AXIS_COUNT];
    int32_t gyroADCRawPrevious[XYZ_AXIS_COUNT];
    int16_t gyroADCRaw[XYZ_AXIS_COUNT];
    int16_t temperature;
    mpuConfiguration_t mpuConfiguration;
    mpuDetectionResult_t mpuDetectionResult;
    sensor_align_e gyroAlign;
    float gyroRateKHz;
    bool dataReady;
    bool gyro_high_fsr;
    uint8_t hardware_lpf;
    uint8_t hardware_32khz_lpf;
    uint8_t mpuDividerDrops;
    ioTag_t mpuIntExtiTag;
    uint8_t gyroHasOverflowProtection;
    gyroSensor_e gyroHardware;
} gyroDev_t;

typedef struct accDev_s {
#if defined(SIMULATOR_BUILD) && defined(SIMULATOR_MULTITHREAD)
    pthread_mutex_t lock;
#endif
    sensorAccInitFuncPtr initFn;                              // initialize function
    sensorAccReadFuncPtr readFn;                              // read 3 axis data function
    busDevice_t bus;
    uint16_t acc_1G;
    int16_t ADCRaw[XYZ_AXIS_COUNT];
    mpuDetectionResult_t mpuDetectionResult;
    sensor_align_e accAlign;
    bool dataReady;
    bool acc_high_fsr;
    char revisionCode;                                      // a revision code for the sensor, if known
}  __attribute__((packed)) accDev_t;

static inline void accDevLock(accDev_t *acc)
{
#if defined(SIMULATOR_BUILD) && defined(SIMULATOR_MULTITHREAD)
    pthread_mutex_lock(&acc->lock);
#else
    (void)acc;
#endif
}

static inline void accDevUnLock(accDev_t *acc)
{
#if defined(SIMULATOR_BUILD) && defined(SIMULATOR_MULTITHREAD)
    pthread_mutex_unlock(&acc->lock);
#else
    (void)acc;
#endif
}

static inline void gyroDevLock(gyroDev_t *gyro)
{
#if defined(SIMULATOR_BUILD) && defined(SIMULATOR_MULTITHREAD)
    pthread_mutex_lock(&gyro->lock);
#else
    (void)gyro;
#endif
}

static inline void gyroDevUnLock(gyroDev_t *gyro)
{
#if defined(SIMULATOR_BUILD) && defined(SIMULATOR_MULTITHREAD)
    pthread_mutex_unlock(&gyro->lock);
#else
    (void)gyro;
#endif
}
#pragma GCC diagnostic pop
