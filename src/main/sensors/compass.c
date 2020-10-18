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
#include <math.h>

#include "platform.h"

#include "build/debug.h"
#include "common/axis.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "drivers/bus_i2c.h"
#include "drivers/bus_spi.h"
#include "drivers/bus.h"
#include "drivers/accgyro/accgyro_mpu.h"
#include "drivers/compass/compass.h"
#include "drivers/compass/compass_ak8975.h"
#include "drivers/compass/compass_ak8963.h"
#include "drivers/compass/compass_fake.h"
#include "drivers/compass/compass_hmc5883l.h"
#include "drivers/compass/compass_qmc5883l.h"
#include "drivers/io.h"
#include "drivers/light_led.h"

#include "fc/config.h"
#include "fc/runtime_config.h"

#include "sensors/boardalignment.h"
#include "sensors/compass.h"
#include "sensors/gyro.h"
#include "sensors/sensors.h"

#ifdef USE_HARDWARE_REVISION_DETECTION
#include "hardware_revision.h"
#endif

magDev_t magDev;
mag_t mag;                   // mag access functions

#ifdef MAG_INT_EXTI
#define COMPASS_INTERRUPT_TAG   IO_TAG(MAG_INT_EXTI)
#else
#define COMPASS_INTERRUPT_TAG   IO_TAG_NONE
#endif

PG_REGISTER_WITH_RESET_FN(compassConfig_t, compassConfig, PG_COMPASS_CONFIG, 1);

void pgResetFn_compassConfig(compassConfig_t *compassConfig) {
    compassConfig->mag_align = ALIGN_DEFAULT;
    compassConfig->mag_declination = 0;
    compassConfig->mag_hardware = MAG_DEFAULT;
// Generate a reasonable default for backward compatibility
// Strategy is
// 1. If SPI device is defined, it will take precedence, assuming it's onboard.
// 2. I2C devices are will be handled by address = 0 (per device default).
// 3. Slave I2C device on SPI gyro
#if defined(USE_SPI) && (defined(USE_MAG_SPI_HMC5883) || defined(USE_MAG_SPI_AK8963))
    compassConfig->mag_bustype = BUSTYPE_SPI;
#ifdef USE_MAG_SPI_HMC5883
    compassConfig->mag_spi_device = SPI_DEV_TO_CFG(spiDeviceByInstance(HMC5883_SPI_INSTANCE));
    compassConfig->mag_spi_csn = IO_TAG(HMC5883_CS_PIN);
#else
    compassConfig->mag_spi_device = SPI_DEV_TO_CFG(spiDeviceByInstance(AK8963_SPI_INSTANCE));
    compassConfig->mag_spi_csn = IO_TAG(AK8963_CS_PIN);
#endif
    compassConfig->mag_i2c_device = I2C_DEV_TO_CFG(I2CINVALID);
    compassConfig->mag_i2c_address = 0;
#elif defined(USE_MAG_HMC5883) || defined(USE_MAG_QMC5883) || defined(USE_MAG_AK8975) || (defined(USE_MAG_AK8963) && !(defined(USE_GYRO_SPI_MPU6500) || defined(USE_GYRO_SPI_MPU9250)))
    compassConfig->mag_bustype = BUSTYPE_I2C;
    compassConfig->mag_i2c_device = I2C_DEV_TO_CFG(MAG_I2C_INSTANCE);
    compassConfig->mag_i2c_address = 0;
    compassConfig->mag_spi_device = SPI_DEV_TO_CFG(SPIINVALID);
    compassConfig->mag_spi_csn = IO_TAG_NONE;
#elif defined(USE_MAG_AK8963) && (defined(USE_GYRO_SPI_MPU6500) || defined(USE_GYRO_SPI_MPU9250))
    compassConfig->mag_bustype = BUSTYPE_MPU_SLAVE;
    compassConfig->mag_i2c_device = I2C_DEV_TO_CFG(I2CINVALID);
    compassConfig->mag_i2c_address = 0;
    compassConfig->mag_spi_device = SPI_DEV_TO_CFG(SPIINVALID);
    compassConfig->mag_spi_csn = IO_TAG_NONE;
#else
    compassConfig->mag_hardware = MAG_NONE;
    compassConfig->mag_bustype = BUSTYPE_NONE;
    compassConfig->mag_i2c_device = I2C_DEV_TO_CFG(I2CINVALID);
    compassConfig->mag_i2c_address = 0;
    compassConfig->mag_spi_device = SPI_DEV_TO_CFG(SPIINVALID);
    compassConfig->mag_spi_csn = IO_TAG_NONE;
#endif
    compassConfig->interruptTag = COMPASS_INTERRUPT_TAG;
}

#if defined(USE_MAG)

static int16_t magADCRaw[XYZ_AXIS_COUNT];
static uint8_t magInit = 0;

#if !defined(SIMULATOR_BUILD)
bool compassDetect(magDev_t *dev) {
    magSensor_e magHardware = MAG_NONE;
    busDevice_t *busdev = &dev->busdev;
#ifdef USE_MAG_DATA_READY_SIGNAL
    dev->magIntExtiTag = compassConfig()->interruptTag;
#endif
    switch (compassConfig()->mag_bustype) {
#ifdef USE_I2C
    case BUSTYPE_I2C:
        busdev->bustype = BUSTYPE_I2C;
        busdev->busdev_u.i2c.device = I2C_CFG_TO_DEV(compassConfig()->mag_i2c_device);
        busdev->busdev_u.i2c.address = compassConfig()->mag_i2c_address;
#endif
        break;
#ifdef USE_SPI
    case BUSTYPE_SPI:
        busdev->bustype = BUSTYPE_SPI;
        spiBusSetInstance(busdev, spiInstanceByDevice(SPI_CFG_TO_DEV(compassConfig()->mag_spi_device)));
        busdev->busdev_u.spi.csnPin = IOGetByTag(compassConfig()->mag_spi_csn);
#endif
        break;
#if defined(USE_MAG_AK8963) && (defined(USE_GYRO_SPI_MPU6500) || defined(USE_GYRO_SPI_MPU9250))
    case BUSTYPE_MPU_SLAVE: {
        if (gyroMpuDetectionResult()->sensor == MPU_9250_SPI) {
            busdev->bustype = BUSTYPE_MPU_SLAVE;
            busdev->busdev_u.mpuSlave.master = gyroSensorBus();
            busdev->busdev_u.mpuSlave.address = compassConfig()->mag_i2c_address;
        } else {
            return false;
        }
    }
#endif
    break;
    default:
        return false;
    }
    dev->magAlign = ALIGN_DEFAULT;
    switch (compassConfig()->mag_hardware) {
    case MAG_DEFAULT:
        FALLTHROUGH;
    case MAG_HMC5883:
#if defined(USE_MAG_HMC5883) || defined(USE_MAG_SPI_HMC5883)
        if (busdev->bustype == BUSTYPE_I2C) {
            busdev->busdev_u.i2c.address = compassConfig()->mag_i2c_address;
        }
        if (hmc5883lDetect(dev)) {
#ifdef MAG_HMC5883_ALIGN
            dev->magAlign = MAG_HMC5883_ALIGN;
#endif
            magHardware = MAG_HMC5883;
            break;
        }
#endif
        FALLTHROUGH;
    case MAG_QMC5883:
#ifdef USE_MAG_QMC5883
        if (busdev->bustype == BUSTYPE_I2C) {
            busdev->busdev_u.i2c.address = compassConfig()->mag_i2c_address;
        }
        if (qmc5883lDetect(dev)) {
#ifdef MAG_QMC5883L_ALIGN
            dev->magAlign = MAG_QMC5883L_ALIGN;
#endif
            magHardware = MAG_QMC5883;
            break;
        }
#endif
        FALLTHROUGH;
    case MAG_AK8975:
#ifdef USE_MAG_AK8975
        if (busdev->bustype == BUSTYPE_I2C) {
            busdev->busdev_u.i2c.address = compassConfig()->mag_i2c_address;
        }
        if (ak8975Detect(dev)) {
#ifdef MAG_AK8975_ALIGN
            dev->magAlign = MAG_AK8975_ALIGN;
#endif
            magHardware = MAG_AK8975;
            break;
        }
#endif
        FALLTHROUGH;
    case MAG_AK8963:
#if defined(USE_MAG_AK8963) || defined(USE_MAG_SPI_AK8963)
        if (busdev->bustype == BUSTYPE_I2C) {
            busdev->busdev_u.i2c.address = compassConfig()->mag_i2c_address;
        }
        if (gyroMpuDetectionResult()->sensor == MPU_9250_SPI) {
            dev->busdev.bustype = BUSTYPE_MPU_SLAVE;
            busdev->busdev_u.mpuSlave.address = compassConfig()->mag_i2c_address;
            dev->busdev.busdev_u.mpuSlave.master = gyroSensorBus();
        }
        if (ak8963Detect(dev)) {
#ifdef MAG_AK8963_ALIGN
            dev->magAlign = MAG_AK8963_ALIGN;
#endif
            magHardware = MAG_AK8963;
            break;
        }
#endif
        FALLTHROUGH;
    case MAG_NONE:
        magHardware = MAG_NONE;
        break;
    }
    if (magHardware == MAG_NONE) {
        return false;
    }
    detectedSensors[SENSOR_INDEX_MAG] = magHardware;
    sensorsSet(SENSOR_MAG);
    return true;
}
#else
bool compassDetect(magDev_t *dev) {
    UNUSED(dev);
    return false;
}
#endif // !SIMULATOR_BUILD

bool compassInit(void) {
    // initialize and calibration. turn on led during mag calibration (calibration routine blinks it)
    // calculate magnetic declination
    mag.magneticDeclination = 0.0f; // TODO investigate if this is actually needed if there is no mag sensor or if the value stored in the config should be used.
    if (!compassDetect(&magDev)) {
        return false;
    }
    const int16_t deg = compassConfig()->mag_declination / 100;
    const int16_t min = compassConfig()->mag_declination % 100;
    mag.magneticDeclination = (deg + ((float)min * (1.0f / 60.0f))) * 10; // heading is in 0.1deg units
    LED1_ON;
    magDev.init(&magDev);
    LED1_OFF;
    magInit = 1;
    if (compassConfig()->mag_align != ALIGN_DEFAULT) {
        magDev.magAlign = compassConfig()->mag_align;
    }
    return true;
}

bool compassIsHealthy(quaternion *q) {
    const float magModulus = quaternionModulus(q);
    //todo findout mag healthy limits
    return ((1 < magModulus) && (magModulus < 10000));
}

void compassUpdate(timeUs_t currentTimeUs) {
    static timeUs_t tCal = 0;
    static flightDynamicsTrims_t magZeroTempMin;
    static flightDynamicsTrims_t magZeroTempMax;
    magDev.read(&magDev, magADCRaw);
    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        mag.magADC[axis] = magADCRaw[axis];
    }
    alignSensors(mag.magADC, magDev.magAlign);
    flightDynamicsTrims_t *magZero = &compassConfigMutable()->magZero;
    if (STATE(CALIBRATE_MAG)) {
        tCal = currentTimeUs;
        for (int axis = 0; axis < 3; axis++) {
            magZero->raw[axis] = 0;
            magZeroTempMin.raw[axis] = mag.magADC[axis];
            magZeroTempMax.raw[axis] = mag.magADC[axis];
        }
        DISABLE_STATE(CALIBRATE_MAG);
    }
    if (magInit) {              // we apply offset only once mag calibration is done
        mag.magADC[X] -= magZero->raw[X];
        mag.magADC[Y] -= magZero->raw[Y];
        mag.magADC[Z] -= magZero->raw[Z];
    }
    if (tCal != 0) {
        if ((currentTimeUs - tCal) < 30000000) {    // 30s: you have 30s to turn the multi in all directions
            LED0_TOGGLE;
            for (int axis = 0; axis < 3; axis++) {
                if (mag.magADC[axis] < magZeroTempMin.raw[axis])
                    magZeroTempMin.raw[axis] = mag.magADC[axis];
                if (mag.magADC[axis] > magZeroTempMax.raw[axis])
                    magZeroTempMax.raw[axis] = mag.magADC[axis];
            }
        } else {
            tCal = 0;
            for (int axis = 0; axis < 3; axis++) {
                magZero->raw[axis] = (magZeroTempMin.raw[axis] + magZeroTempMax.raw[axis]) / 2; // Calculate offsets
            }
            saveConfigAndNotify();
        }
    }
}

bool compassGetAverage(quaternion *vAverage) {
    vAverage->w = 0;
    vAverage->x = mag.magADC[X];
    vAverage->y = mag.magADC[Y];
    vAverage->z = mag.magADC[Z];
    return(true);
}
#endif // USE_MAG
