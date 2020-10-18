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

#include "platform.h"

#include "build/debug.h"

#include "common/axis.h"
#include "common/filter.h"
#include "common/utils.h"

#include "config/config_reset.h"
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

#ifdef USE_ACC_ADXL345
#include "drivers/accgyro_legacy/accgyro_adxl345.h"
#endif

#ifdef USE_ACC_BMA280
#include "drivers/accgyro_legacy/accgyro_bma280.h"
#endif

#ifdef USE_ACC_LSM303DLHC
#include "drivers/accgyro_legacy/accgyro_lsm303dlhc.h"
#endif

#ifdef USE_ACC_MMA8452
#include "drivers/accgyro_legacy/accgyro_mma845x.h"
#endif

#ifdef USE_ACC_IMUF9001
#include "drivers/accgyro/accgyro_imuf9001.h"
#endif //USE_ACC_IMUF9001
#include "drivers/bus_spi.h"

#include "fc/config.h"
#include "fc/runtime_config.h"

#include "io/beeper.h"

#include "sensors/acceleration.h"
#include "sensors/boardalignment.h"
#include "sensors/gyro.h"
#include "sensors/sensors.h"

#ifdef USE_HARDWARE_REVISION_DETECTION
#include "hardware_revision.h"
#endif


FAST_RAM_ZERO_INIT acc_t acc;                       // acc access functions

static float accumulatedMeasurements[XYZ_AXIS_COUNT];
static int accumulatedMeasurementCount;

static uint16_t calibratingA = 0;      // the calibration is done is the main loop. Calibrating decreases at each cycle down to 0, then we enter in a normal mode.

extern uint16_t InflightcalibratingA;
extern bool AccInflightCalibrationMeasurementDone;
extern bool AccInflightCalibrationSavetoEEProm;
extern bool AccInflightCalibrationActive;

static flightDynamicsTrims_t *accelerationTrims;

static uint16_t accLpfCutHz = 0;
static pt1Filter_t accFilterPt1[XYZ_AXIS_COUNT];

PG_REGISTER_WITH_RESET_FN(accelerometerConfig_t, accelerometerConfig, PG_ACCELEROMETER_CONFIG, 0);

void resetRollAndPitchTrims(rollAndPitchTrims_t *rollAndPitchTrims) {
    RESET_CONFIG_2(rollAndPitchTrims_t, rollAndPitchTrims,
                   .values.roll = 0,
                   .values.pitch = 0,
                  );
}

void accResetRollAndPitchTrims(void) {
    resetRollAndPitchTrims(&accelerometerConfigMutable()->accelerometerTrims);
}

static void resetFlightDynamicsTrims(flightDynamicsTrims_t *accZero) {
    accZero->values.roll = 0;
    accZero->values.pitch = 0;
    accZero->values.yaw = 0;
}

void accResetFlightDynamicsTrims(void) {
    resetFlightDynamicsTrims(&accelerometerConfigMutable()->accZero);
}

void pgResetFn_accelerometerConfig(accelerometerConfig_t *instance) {
    RESET_CONFIG_2(accelerometerConfig_t, instance,
                   .acc_lpf_hz = 40,
                   .acc_align = ALIGN_DEFAULT,
                   .acc_hardware = ACC_DEFAULT,
                   .acc_high_fsr = false,
                  );
    resetRollAndPitchTrims(&instance->accelerometerTrims);
    resetFlightDynamicsTrims(&instance->accZero);
}

bool accDetect(accDev_t *dev, accelerationSensor_e accHardwareToUse) {
    accelerationSensor_e accHardware = ACC_NONE;
#ifdef USE_ACC_ADXL345
    drv_adxl345_config_t acc_params;
#endif
retry:
    switch (accHardwareToUse) {
    case ACC_DEFAULT:
        FALLTHROUGH;
#ifdef USE_ACC_ADXL345
    case ACC_ADXL345: // ADXL345
        acc_params.useFifo = false;
        acc_params.dataRate = 800; // unused currently
        if (adxl345Detect(&acc_params, dev)) {
#ifdef ACC_ADXL345_ALIGN
            dev->accAlign = ACC_ADXL345_ALIGN;
#endif
            accHardware = ACC_ADXL345;
            break;
        }
        FALLTHROUGH;
#endif
#ifdef USE_ACC_LSM303DLHC
    case ACC_LSM303DLHC:
        if (lsm303dlhcAccDetect(dev)) {
#ifdef ACC_LSM303DLHC_ALIGN
            dev->accAlign = ACC_LSM303DLHC_ALIGN;
#endif
            accHardware = ACC_LSM303DLHC;
            break;
        }
        FALLTHROUGH;
#endif
#ifdef USE_ACC_MPU6050
    case ACC_MPU6050: // MPU6050
        if (mpu6050AccDetect(dev)) {
#ifdef ACC_MPU6050_ALIGN
            dev->accAlign = ACC_MPU6050_ALIGN;
#endif
            accHardware = ACC_MPU6050;
            break;
        }
        FALLTHROUGH;
#endif
#ifdef USE_ACC_MMA8452
    case ACC_MMA8452: // MMA8452
        if (mma8452Detect(dev)) {
#ifdef ACC_MMA8452_ALIGN
            dev->accAlign = ACC_MMA8452_ALIGN;
#endif
            accHardware = ACC_MMA8452;
            break;
        }
        FALLTHROUGH;
#endif
#ifdef USE_ACC_BMA280
    case ACC_BMA280: // BMA280
        if (bma280Detect(dev)) {
#ifdef ACC_BMA280_ALIGN
            dev->accAlign = ACC_BMA280_ALIGN;
#endif
            accHardware = ACC_BMA280;
            break;
        }
        FALLTHROUGH;
#endif
#ifdef USE_ACC_SPI_MPU6000
    case ACC_MPU6000:
        if (mpu6000SpiAccDetect(dev)) {
#ifdef ACC_MPU6000_ALIGN
            dev->accAlign = ACC_MPU6000_ALIGN;
#endif
            accHardware = ACC_MPU6000;
            break;
        }
        FALLTHROUGH;
#endif
#ifdef USE_ACC_SPI_MPU9250
    case ACC_MPU9250:
        if (mpu9250SpiAccDetect(dev)) {
#ifdef ACC_MPU9250_ALIGN
            dev->accAlign = ACC_MPU9250_ALIGN;
#endif
            accHardware = ACC_MPU9250;
            break;
        }
        FALLTHROUGH;
#endif
    case ACC_MPU6500:
    case ACC_ICM20601:
    case ACC_ICM20602:
    case ACC_ICM20608G:
#if defined(USE_ACC_MPU6500) || defined(USE_ACC_SPI_MPU6500)
#ifdef USE_ACC_SPI_MPU6500
        if (mpu6500AccDetect(dev) || mpu6500SpiAccDetect(dev)) {
#else
        if (mpu6500AccDetect(dev)) {
#endif
#ifdef ACC_MPU6500_ALIGN
            dev->accAlign = ACC_MPU6500_ALIGN;
#endif
            switch (dev->mpuDetectionResult.sensor) {
            case MPU_9250_SPI:
                accHardware = ACC_MPU9250;
                break;
            case ICM_20601_SPI:
                accHardware = ACC_ICM20601;
                break;
            case ICM_20602_SPI:
                accHardware = ACC_ICM20602;
                break;
            case ICM_20608_SPI:
                accHardware = ACC_ICM20608G;
                break;
            default:
                accHardware = ACC_MPU6500;
            }
            break;
        }
#endif
        FALLTHROUGH;
#ifdef USE_ACC_SPI_ICM20649
    case ACC_ICM20649:
        if (icm20649SpiAccDetect(dev)) {
            accHardware = ACC_ICM20649;
#ifdef ACC_ICM20649_ALIGN
            dev->accAlign = ACC_ICM20649_ALIGN;
#endif
            break;
        }
        FALLTHROUGH;
#endif
#ifdef USE_ACC_IMUF9001
    case ACC_IMUF9001:
        if (imufSpiAccDetect(dev)) {
            accHardware = ACC_IMUF9001;
            break;
        }
        FALLTHROUGH;
#endif
#ifdef USE_ACC_SPI_ICM20689
    case ACC_ICM20689:
        if (icm20689SpiAccDetect(dev)) {
            accHardware = ACC_ICM20689;
#ifdef ACC_ICM20689_ALIGN
            dev->accAlign = ACC_ICM20689_ALIGN;
#endif
            break;
        }
        FALLTHROUGH;
#endif
#ifdef USE_ACCGYRO_BMI160
    case ACC_BMI160:
        if (bmi160SpiAccDetect(dev)) {
            accHardware = ACC_BMI160;
#ifdef ACC_BMI160_ALIGN
            dev->accAlign = ACC_BMI160_ALIGN;
#endif
            break;
        }
        FALLTHROUGH;
#endif
#ifdef USE_FAKE_ACC
    case ACC_FAKE:
        if (fakeAccDetect(dev)) {
            accHardware = ACC_FAKE;
            break;
        }
        FALLTHROUGH;
#endif
    default:
    case ACC_NONE: // disable ACC
        accHardware = ACC_NONE;
        break;
    }
    // Found anything? Check if error or ACC is really missing.
    if (accHardware == ACC_NONE && accHardwareToUse != ACC_DEFAULT && accHardwareToUse != ACC_NONE) {
        // Nothing was found and we have a forced sensor that isn't present.
        accHardwareToUse = ACC_DEFAULT;
        goto retry;
    }
    if (accHardware == ACC_NONE) {
        return false;
    }
    detectedSensors[SENSOR_INDEX_ACC] = accHardware;
    sensorsSet(SENSOR_ACC);
    return true;
}

bool accInit(void) {
    memset(&acc, 0, sizeof(acc));
    // copy over the common gyro mpu settings
    acc.dev.bus = *gyroSensorBus();
    acc.dev.mpuDetectionResult = *gyroMpuDetectionResult();
    acc.dev.acc_high_fsr = accelerometerConfig()->acc_high_fsr;
#ifdef USE_DUAL_GYRO
    if (gyroConfig()->gyro_to_use == GYRO_CONFIG_USE_GYRO_2) {
        acc.dev.accAlign = ACC_2_ALIGN;
    } else {
        acc.dev.accAlign = ACC_1_ALIGN;
    }
#else
    acc.dev.accAlign = ALIGN_DEFAULT;
#endif
    if (!accDetect(&acc.dev, accelerometerConfig()->acc_hardware)) {
        return false;
    }
    acc.dev.acc_1G = 256; // set default
    acc.dev.initFn(&acc.dev); // driver initialisation
    // set the acc sampling interval according to the gyro sampling interval
    if (accLpfCutHz) {
        const float k = pt1FilterGain(accLpfCutHz, 1.0f / (float)DEFAULT_ACC_SAMPLE_INTERVAL);
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            pt1FilterInit(&accFilterPt1[axis], k);
        }
    }
#ifndef USE_ACC_IMUF9001
    if (accelerometerConfig()->acc_align != ALIGN_DEFAULT) {
        acc.dev.accAlign = accelerometerConfig()->acc_align;
    }
#endif //USE_ACC_IMUF9001
    return true;
}

void accSetCalibrationCycles(uint16_t calibrationCyclesRequired) {
    calibratingA = calibrationCyclesRequired;
}

bool accIsCalibrationComplete(void) {
    return calibratingA == 0;
}

static bool isOnFinalAccelerationCalibrationCycle(void) {
    return calibratingA == 1;
}

static bool isOnFirstAccelerationCalibrationCycle(void) {
    return calibratingA == CALIBRATING_ACC_CYCLES;
}

static void performAccelerationCalibration(rollAndPitchTrims_t *rollAndPitchTrims) {
    static int32_t a[3];
    for (int axis = 0; axis < 3; axis++) {
        // Reset a[axis] at start of calibration
        if (isOnFirstAccelerationCalibrationCycle()) {
            a[axis] = 0;
        }
        // Sum up CALIBRATING_ACC_CYCLES readings
        a[axis] += acc.accADC[axis];
        // Reset global variables to prevent other code from using un-calibrated data
        acc.accADC[axis] = 0;
        accelerationTrims->raw[axis] = 0;
    }
    if (isOnFinalAccelerationCalibrationCycle()) {
        // Calculate average, shift Z down by acc_1G and store values in EEPROM at end of calibration
        accelerationTrims->raw[X] = a[X] / CALIBRATING_ACC_CYCLES;
        accelerationTrims->raw[Y] = a[Y] / CALIBRATING_ACC_CYCLES;
        accelerationTrims->raw[Z] = a[Z] / CALIBRATING_ACC_CYCLES - acc.dev.acc_1G;
        resetRollAndPitchTrims(rollAndPitchTrims);
        saveConfigAndNotify();
    }
    calibratingA--;
}

static void performInflightAccelerationCalibration(rollAndPitchTrims_t *rollAndPitchTrims) {
    static int32_t b[3];
    static int16_t accZero_saved[3] = { 0, 0, 0 };
    static rollAndPitchTrims_t angleTrim_saved = { { 0, 0 } };
    // Saving old zeropoints before measurement
    if (InflightcalibratingA == 50) {
        accZero_saved[X] = accelerationTrims->raw[X];
        accZero_saved[Y] = accelerationTrims->raw[Y];
        accZero_saved[Z] = accelerationTrims->raw[Z];
        angleTrim_saved.values.roll = rollAndPitchTrims->values.roll;
        angleTrim_saved.values.pitch = rollAndPitchTrims->values.pitch;
    }
    if (InflightcalibratingA > 0) {
        for (int axis = 0; axis < 3; axis++) {
            // Reset a[axis] at start of calibration
            if (InflightcalibratingA == 50)
                b[axis] = 0;
            // Sum up 50 readings
            b[axis] += acc.accADC[axis];
            // Clear global variables for next reading
            acc.accADC[axis] = 0;
            accelerationTrims->raw[axis] = 0;
        }
        // all values are measured
        if (InflightcalibratingA == 1) {
            AccInflightCalibrationActive = false;
            AccInflightCalibrationMeasurementDone = true;
            beeper(BEEPER_ACC_CALIBRATION); // indicate end of calibration
            // recover saved values to maintain current flight behaviour until new values are transferred
            accelerationTrims->raw[X] = accZero_saved[X];
            accelerationTrims->raw[Y] = accZero_saved[Y];
            accelerationTrims->raw[Z] = accZero_saved[Z];
            rollAndPitchTrims->values.roll = angleTrim_saved.values.roll;
            rollAndPitchTrims->values.pitch = angleTrim_saved.values.pitch;
        }
        InflightcalibratingA--;
    }
    // Calculate average, shift Z down by acc_1G and store values in EEPROM at end of calibration
    if (AccInflightCalibrationSavetoEEProm) {      // the aircraft is landed, disarmed and the combo has been done again
        AccInflightCalibrationSavetoEEProm = false;
        accelerationTrims->raw[X] = b[X] / 50;
        accelerationTrims->raw[Y] = b[Y] / 50;
        accelerationTrims->raw[Z] = b[Z] / 50 - acc.dev.acc_1G;    // for nunchuck 200=1G
        resetRollAndPitchTrims(rollAndPitchTrims);
        saveConfigAndNotify();
    }
}

void accUpdate(timeUs_t currentTimeUs, rollAndPitchTrims_t *rollAndPitchTrims) {
    UNUSED(currentTimeUs);
    if (!acc.dev.readFn(&acc.dev)) {
        return;
    }
    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        DEBUG_SET(DEBUG_ACCELEROMETER, axis, acc.dev.ADCRaw[axis]);
        acc.accADC[axis] = acc.dev.ADCRaw[axis];
    }
#ifndef USE_ACC_IMUF9001
    alignSensors(acc.accADC, acc.dev.accAlign);
#endif
    if (accLpfCutHz) {
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            acc.accADC[axis] = pt1FilterApply(&accFilterPt1[axis], (float)acc.accADC[axis]);
        }
    }
    if (!accIsCalibrationComplete()) {
        performAccelerationCalibration(rollAndPitchTrims);
    } else if (feature(FEATURE_INFLIGHT_ACC_CAL)) {
        performInflightAccelerationCalibration(rollAndPitchTrims);
    }
    acc.accADC[X] -= accelerationTrims->raw[X];
    acc.accADC[Y] -= accelerationTrims->raw[Y];
    acc.accADC[Z] -= accelerationTrims->raw[Z];
    ++accumulatedMeasurementCount;
    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        accumulatedMeasurements[axis] += acc.accADC[axis];
    }
    acc.isAccelUpdatedAtLeastOnce = true;
}

bool accGetAverage(quaternion *vAverage) {
    if (accumulatedMeasurementCount > 0) {
        vAverage->w = 0;
        vAverage->x = accumulatedMeasurements[X] / accumulatedMeasurementCount;
        vAverage->y = accumulatedMeasurements[Y] / accumulatedMeasurementCount;
        vAverage->z = accumulatedMeasurements[Z] / accumulatedMeasurementCount;
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            accumulatedMeasurements[axis] = 0.0f;
        }
        accumulatedMeasurementCount = 0;
        return true;
    } else {
        quaternionInitVector(vAverage);
        return false;
    }
}

void setAccelerationTrims(flightDynamicsTrims_t *accelerationTrimsToUse) {
    accelerationTrims = accelerationTrimsToUse;
}

void accInitFilters(void) {
    accLpfCutHz = accelerometerConfig()->acc_lpf_hz;
    if (accLpfCutHz) {
        const float k = pt1FilterGain(accLpfCutHz, 1.0f / (float)DEFAULT_ACC_SAMPLE_INTERVAL);
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            pt1FilterInit(&accFilterPt1[axis], k);
        }
    }
}

bool accIsHealthy(quaternion *q) {
    // acc calibbration error max 2.4% (non Z axes)
    // accept 7% deviation
    float accModulus = quaternionModulus(q);
    accModulus = accModulus / acc.dev.acc_1G;
    return ((0.93f < accModulus) && (accModulus < 1.07f));
}
