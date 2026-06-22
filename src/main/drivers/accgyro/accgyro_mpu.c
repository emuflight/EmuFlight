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
#include <stdlib.h>
#include <string.h>

#include "platform.h"

#include "build/atomic.h"
#include "build/build_config.h"
#include "build/debug.h"

#include "common/maths.h"
#include "common/utils.h"

#include "drivers/bus.h"
#include "drivers/bus_i2c.h"
#include "drivers/bus_i2c_busdev.h"
#include "drivers/bus_spi.h"
#include "drivers/exti.h"
#include "drivers/io.h"
#include "drivers/nvic.h"
#include "drivers/sensor.h"
#include "drivers/system.h"
#include "drivers/time.h"

#ifdef USE_GYRO_IMUF9001
#include "sensors/gyro.h"
#include "sensors/acceleration.h"
#endif

#include "drivers/accgyro/accgyro.h"
#include "drivers/accgyro/accgyro_mpu3050.h"
#include "drivers/accgyro/accgyro_mpu6050.h"
#include "drivers/accgyro/accgyro_mpu6500.h"
#include "drivers/accgyro/accgyro_spi_bmi160.h"
#include "drivers/accgyro/accgyro_spi_bmi270.h"
#include "drivers/accgyro/accgyro_spi_icm20649.h"
#include "drivers/accgyro/accgyro_spi_icm20689.h"
#include "drivers/accgyro/accgyro_spi_icm426xx.h"
#include "drivers/accgyro/accgyro_spi_mpu6000.h"
#include "drivers/accgyro/accgyro_spi_mpu6500.h"
#include "drivers/accgyro/accgyro_spi_mpu9250.h"
#include "drivers/accgyro/accgyro_mpu.h"
#ifdef USE_GYRO_IMUF9001
#include "drivers/accgyro/accgyro_imuf9001.h"
#include "rx/rx.h"
#include "fc/fc_rc.h"
#include "fc/runtime_config.h"
#endif //USE_GYRO_IMUF9001

mpuResetFnPtr mpuResetFn;

#ifdef USE_GYRO_IMUF9001
imufData_t imufData;
#endif
#ifndef MPU_I2C_INSTANCE
#define MPU_I2C_INSTANCE I2C_DEVICE
#endif

#ifndef MPU_ADDRESS
#define MPU_ADDRESS             0x68
#endif

#define MPU_INQUIRY_MASK   0x7E

#define GYRO_EXTI_DETECT_THRESHOLD 1000

#if defined(USE_I2C)
static void mpu6050FindRevision(gyroDev_t *gyro) {
    // There is a map of revision contained in the android source tree which is quite comprehensive and may help to understand this code
    // See https://android.googlesource.com/kernel/msm.git/+/eaf36994a3992b8f918c18e4f7411e8b2320a35f/drivers/misc/mpu6050/mldl_cfg.c
    // determine product ID and revision
    uint8_t readBuffer[6];
    bool ack = busReadRegisterBuffer(&gyro->dev, MPU_RA_XA_OFFS_H, readBuffer, 6);
    uint8_t revision = ((readBuffer[5] & 0x01) << 2) | ((readBuffer[3] & 0x01) << 1) | (readBuffer[1] & 0x01);
    if (ack && revision) {
        // Congrats, these parts are better
        if (revision == 1) {
            gyro->mpuDetectionResult.resolution = MPU_HALF_RESOLUTION;
        } else if (revision == 2) {
            gyro->mpuDetectionResult.resolution = MPU_FULL_RESOLUTION;
        } else if ((revision == 3) || (revision == 7)) {
            gyro->mpuDetectionResult.resolution = MPU_FULL_RESOLUTION;
        } else {
            failureMode(FAILURE_ACC_INCOMPATIBLE);
        }
    } else {
        uint8_t productId;
        ack = busReadRegisterBuffer(&gyro->dev, MPU_RA_PRODUCT_ID, &productId, 1);
        revision = productId & 0x0F;
        if (!ack || revision == 0) {
            failureMode(FAILURE_ACC_INCOMPATIBLE);
        } else if (revision == 4) {
            gyro->mpuDetectionResult.resolution = MPU_HALF_RESOLUTION;
        } else {
            gyro->mpuDetectionResult.resolution = MPU_FULL_RESOLUTION;
        }
    }
}
#endif

/*
 * Gyro interrupt service routine
 */
#if defined(MPU_INT_EXTI)
// Called in ISR context after DMA transfer completes
busStatus_e mpuIntCallback(uint32_t arg)
{
    gyroDev_t *gyro = (gyroDev_t *)arg;
    int32_t gyroDmaDuration = cmpTimeCycles(getCycleCounter(), gyro->gyroLastEXTI);
    if (gyroDmaDuration > gyro->gyroDmaMaxDuration) {
        gyro->gyroDmaMaxDuration = gyroDmaDuration;
    }
    gyro->dataReady = true;
    return BUS_READY;
}

FAST_CODE static void mpuIntExtiHandler(extiCallbackRec_t *cb) {
    gyroDev_t *gyro = container_of(cb, gyroDev_t, exti);
    uint32_t nowCycles = getCycleCounter();
    int32_t gyroLastPeriod = cmpTimeCycles(nowCycles, gyro->gyroLastEXTI);
    if ((gyro->gyroShortPeriod == 0) || (gyroLastPeriod < gyro->gyroShortPeriod)) {
        gyro->gyroSyncEXTI = gyro->gyroLastEXTI + gyro->gyroDmaMaxDuration;
    }
    gyro->gyroLastEXTI = nowCycles;
#if defined(USE_GYRO_IMUF9001)
    imufPrepareDmaRead(gyro);
    spiSequence(&gyro->dev, gyro->segments);
#elif defined(GYRO_USES_SPI)
    if (gyro->gyroModeSPI == GYRO_EXTI_INT_DMA) {
        spiSequence(&gyro->dev, gyro->segments);
    }
#endif
    gyro->detectedEXTI++;
}

static void mpuIntExtiInit(gyroDev_t *gyro) {
    if (gyro->mpuIntExtiTag == IO_TAG_NONE) {
        return;
    }
    const IO_t mpuIntIO = IOGetByTag(gyro->mpuIntExtiTag);
#ifdef ENSURE_MPU_DATA_READY_IS_LOW
    uint8_t status = IORead(mpuIntIO);
    if (status) {
        return;
    }
#endif
    IOInit(mpuIntIO, OWNER_MPU_EXTI, 0);
    IOConfigGPIO(mpuIntIO, IOCFG_IN_FLOATING);
    EXTIHandlerInit(&gyro->exti, mpuIntExtiHandler);
    EXTIConfig(mpuIntIO, &gyro->exti, NVIC_PRIO_MPU_INT_EXTI, IOCFG_IN_FLOATING, EXTI_TRIGGER_RISING);
    EXTIEnable(mpuIntIO, true);
}
#endif // MPU_INT_EXTI

bool mpuAccRead(accDev_t *acc) {
    uint8_t data[6];
    const bool ack = busReadRegisterBuffer(&acc->dev, MPU_RA_ACCEL_XOUT_H, data, 6);
    if (!ack) {
        return false;
    }
    acc->ADCRaw[X] = (int16_t)((data[0] << 8) | data[1]);
    acc->ADCRaw[Y] = (int16_t)((data[2] << 8) | data[3]);
    acc->ADCRaw[Z] = (int16_t)((data[4] << 8) | data[5]);
    return true;
}

#ifdef USE_GYRO_IMUF9001
FAST_RAM_ZERO_INIT static uint8_t imufTxBuf[58];
FAST_RAM_ZERO_INIT static uint8_t imufRxBuf[58];
FAST_RAM_ZERO_INIT volatile uint32_t crcErrorCount = 0;

// DMA completion callback: CRC-validate, copy data into gyroADCf and acc.dev.ADCRaw.
FAST_CODE busStatus_e imufIntCallback(uint32_t arg) {
    gyroDev_t *gyro = (gyroDev_t *)arg;
    const uint32_t xferLen = gyro->segments[0].len;
    const uint32_t crc1 = (*(uint32_t *)(imufRxBuf + xferLen - 4)) & 0xFF;
    const uint32_t crc2 = getCrcImuf9001((uint32_t *)imufRxBuf, (xferLen >> 2) - 1) & 0xFF;
    if (crc1 == crc2) {
        memcpy(&imufData, imufRxBuf, sizeof(imufData_t));
        acc.dev.ADCRaw[X]   = (int16_t)(imufData.accX * acc.dev.acc_1G);
        acc.dev.ADCRaw[Y]   = (int16_t)(imufData.accY * acc.dev.acc_1G);
        acc.dev.ADCRaw[Z]   = (int16_t)(imufData.accZ * acc.dev.acc_1G);
        gyro->gyroADCf[X]   = imufData.gyroX;
        gyro->gyroADCf[Y]   = imufData.gyroY;
        gyro->gyroADCf[Z]   = imufData.gyroZ;
        gyro->gyroADCRaw[X] = (int16_t)(imufData.gyroX * 16.4f);
        gyro->gyroADCRaw[Y] = (int16_t)(imufData.gyroY * 16.4f);
        gyro->gyroADCRaw[Z] = (int16_t)(imufData.gyroZ * 16.4f);
        gyro->dataReady = true;
    } else {
        if (++crcErrorCount > 100000) {
            crcErrorCount = 0;
        }
    }
    return BUS_READY;
}

// Prepare TX command and transfer length before each real-time DMA read.
FAST_CODE void imufPrepareDmaRead(gyroDev_t *gyro) {
    imufCommand_t *txCmd = (imufCommand_t *)imufTxBuf;
    if (isImufCalibrating == IMUF_IS_CALIBRATING) {
        memset(imufTxBuf, 0, sizeof(imufCommand_t));
        txCmd->command = IMUF_COMMAND_CALIBRATE;
        txCmd->crc     = getCrcImuf9001((uint32_t *)imufTxBuf, 11);
        isImufCalibrating = IMUF_DONE_CALIBRATING;
    } else if (isImufCalibrating == IMUF_DONE_CALIBRATING) {
        txCmd->command = 0;
        txCmd->crc     = 0;
        imufEndCalibration();
    } else if (isSetpointNew) {
        txCmd->command = IMUF_COMMAND_SETPOINT;
        txCmd->param1  = getSetpointRateInt(0);
        txCmd->param2  = getSetpointRateInt(1);
        txCmd->param3  = getSetpointRateInt(2);
        txCmd->crc     = getCrcImuf9001((uint32_t *)imufTxBuf, 11);
        isSetpointNew = 0;
    }
    const uint32_t xferLen = MIN((uint32_t)gyroConfig()->imuf_mode, (uint32_t)sizeof(imufTxBuf));
    memset(imufRxBuf, 0, xferLen);
    gyro->segments[0].len = xferLen;
}

// Set up gyro->segments for DMA reads; called once from imufSpiGyroInit after mpuGyroInit.
void mpuImufSetupDma(gyroDev_t *gyro) {
    gyro->dev.callbackArg        = (uint32_t)gyro;
    gyro->segments[0].u.buffers.txData = imufTxBuf;
    gyro->segments[0].u.buffers.rxData = imufRxBuf;
    gyro->segments[0].len              = gyroConfig()->imuf_mode;
    gyro->segments[0].negateCS         = true;
    gyro->segments[0].callback         = imufIntCallback;
    gyro->segments[1].len              = 0;
    gyro->segments[1].u.link.dev       = NULL;
    gyro->segments[1].u.link.segments  = NULL;
}
#endif // USE_GYRO_IMUF9001


FAST_CODE bool mpuGyroRead(gyroDev_t *gyro) {
    uint8_t data[6];
    const bool ack = busReadRegisterBuffer(&gyro->dev, MPU_RA_GYRO_XOUT_H, data, 6);
    if (!ack) {
        return false;
    }
    gyro->gyroADCRaw[X] = (int16_t)((data[0] << 8) | data[1]);
    gyro->gyroADCRaw[Y] = (int16_t)((data[2] << 8) | data[3]);
    gyro->gyroADCRaw[Z] = (int16_t)((data[4] << 8) | data[5]);
    return true;
}

FAST_CODE bool mpuGyroReadSPI(gyroDev_t *gyro)
{
#ifdef GYRO_USES_SPI
    int16_t *gyroData = (int16_t *)gyro->dev.rxBuf;
    switch (gyro->gyroModeSPI) {
    case GYRO_EXTI_INIT:
    {
        memset(gyro->dev.txBuf, 0xff, 16);
        gyro->gyroDmaMaxDuration = 5; // seed estimate in CPU cycles; updated by mpuIntCallback with actual measurements
#if defined(MPU_INT_EXTI)
        if (gyro->detectedEXTI > GYRO_EXTI_DETECT_THRESHOLD) {
            if (spiUseDMA(&gyro->dev)) {
                gyro->dev.callbackArg = (uint32_t)gyro;
                gyro->dev.txBuf[0] = gyro->accDataReg | 0x80;
                gyro->segments[0].len = gyro->gyroDataReg - gyro->accDataReg + sizeof(uint8_t) + 3 * sizeof(int16_t);
                gyro->segments[0].callback = mpuIntCallback;
                gyro->segments[0].u.buffers.txData = gyro->dev.txBuf;
                gyro->segments[0].u.buffers.rxData = &gyro->dev.rxBuf[1];
                gyro->segments[0].negateCS = true;
                gyro->segments[1].len = 0;
                gyro->segments[1].u.link.dev = NULL;
                gyro->segments[1].u.link.segments = NULL;
                gyro->gyroModeSPI = GYRO_EXTI_INT_DMA;
            } else {
                gyro->gyroModeSPI = GYRO_EXTI_INT;
            }
        } else {
            gyro->gyroModeSPI = GYRO_EXTI_NO_INT;
        }
#else
        gyro->gyroModeSPI = GYRO_EXTI_NO_INT;
#endif
        break;
    }

    case GYRO_EXTI_INT:
    case GYRO_EXTI_NO_INT:
    {
        gyro->dev.txBuf[0] = gyro->gyroDataReg | 0x80;

        busSegment_t segments[] = {
            {.u.buffers = {NULL, NULL}, 7, true, NULL},
            {.u.link = {NULL, NULL}, 0, true, NULL},
        };
        segments[0].u.buffers.txData = gyro->dev.txBuf;
        segments[0].u.buffers.rxData = &gyro->dev.rxBuf[1];

        spiSequence(&gyro->dev, &segments[0]);
        spiWait(&gyro->dev);

        gyro->gyroADCRaw[X] = __builtin_bswap16(gyroData[1]);
        gyro->gyroADCRaw[Y] = __builtin_bswap16(gyroData[2]);
        gyro->gyroADCRaw[Z] = __builtin_bswap16(gyroData[3]);
        break;
    }

    case GYRO_EXTI_INT_DMA:
    {
        // Data was read by DMA from EXTI interrupt; acc and gyro may not be contiguous
        const uint8_t gyroDataIndex = ((gyro->gyroDataReg - gyro->accDataReg) >> 1) + 1;
        gyro->gyroADCRaw[X] = __builtin_bswap16(gyroData[gyroDataIndex]);
        gyro->gyroADCRaw[Y] = __builtin_bswap16(gyroData[gyroDataIndex + 1]);
        gyro->gyroADCRaw[Z] = __builtin_bswap16(gyroData[gyroDataIndex + 2]);
        break;
    }

    default:
        break;
    }

    return true;
#else
    // I2C gyro path (e.g. CRAZYFLIE2): spiSequence not available
    static const uint8_t dataToSend[7] = {MPU_RA_GYRO_XOUT_H | 0x80, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    static uint8_t data[7];
    const bool ack = spiReadWriteBufRB(&gyro->dev, (uint8_t *)dataToSend, data, 7);
    if (!ack) {
        return false;
    }
    gyro->gyroADCRaw[X] = (int16_t)((data[1] << 8) | data[2]);
    gyro->gyroADCRaw[Y] = (int16_t)((data[3] << 8) | data[4]);
    gyro->gyroADCRaw[Z] = (int16_t)((data[5] << 8) | data[6]);
    return true;
#endif
}

#ifdef USE_SPI
static bool detectSPISensorsAndUpdateDetectionResult(gyroDev_t *gyro) {
    UNUSED(gyro); // since there are FCs which have gyro on I2C but other devices on SPI
    uint8_t sensor = MPU_NONE;
    UNUSED(sensor);
    // note, when USE_DUAL_GYRO is enabled the gyro->dev must already be initialised.
#ifdef USE_GYRO_SPI_MPU6000
#ifndef USE_DUAL_GYRO
    spiSetBusInstance(&gyro->dev, SPI_DEV_TO_CFG(MPU6000_SPI_BUS));
#endif
#ifdef MPU6000_CS_PIN
    gyro->dev.busType_u.spi.csnPin = gyro->dev.busType_u.spi.csnPin == IO_NONE ? IOGetByTag(IO_TAG(MPU6000_CS_PIN)) : gyro->dev.busType_u.spi.csnPin;
#endif
    sensor = mpu6000SpiDetect(&gyro->dev);
    if (sensor != MPU_NONE) {
        gyro->mpuDetectionResult.sensor = sensor;
        return true;
    }
#endif
#ifdef USE_GYRO_SPI_MPU6500
#ifndef USE_DUAL_GYRO
    spiSetBusInstance(&gyro->dev, SPI_DEV_TO_CFG(MPU6500_SPI_BUS));
#endif
#ifdef MPU6500_CS_PIN
    gyro->dev.busType_u.spi.csnPin = gyro->dev.busType_u.spi.csnPin == IO_NONE ? IOGetByTag(IO_TAG(MPU6500_CS_PIN)) : gyro->dev.busType_u.spi.csnPin;
#endif
    sensor = mpu6500SpiDetect(&gyro->dev);
    // some targets using MPU_9250_SPI, ICM_20608_SPI or ICM_20602_SPI state sensor is MPU_65xx_SPI
    if (sensor != MPU_NONE) {
        gyro->mpuDetectionResult.sensor = sensor;
        return true;
    }
#endif
#ifdef USE_GYRO_IMUF9001
#ifdef IMUF9001_SPI_BUS
    spiSetBusInstance(&gyro->dev, SPI_DEV_TO_CFG(IMUF9001_SPI_BUS));
#else
#error IMUF9001 is SPI only
#endif
#ifdef IMUF9001_CS_PIN
    gyro->dev.busType_u.spi.csnPin = gyro->dev.busType_u.spi.csnPin == IO_NONE ? IOGetByTag(IO_TAG(IMUF9001_CS_PIN)) : gyro->dev.busType_u.spi.csnPin;
#else
#error IMUF9001 must use a CS pin (IMUF9001_CS_PIN)
#endif
#ifdef IMUF9001_RST_PIN
    gyro->dev.busType_u.spi.rstPin = IOGetByTag(IO_TAG(IMUF9001_RST_PIN));
#else
#error IMUF9001 must use a RST pin (IMUF9001_RST_PIN)
#endif
    sensor = imuf9001SpiDetect(gyro);
    // some targets using MPU_9250_SPI, ICM_20608_SPI or ICM_20602_SPI state sensor is MPU_65xx_SPI
    if (sensor != MPU_NONE) {
        gyro->mpuDetectionResult.sensor = sensor;
        return true;
    }
#endif
#ifdef  USE_GYRO_SPI_MPU9250
#ifndef USE_DUAL_GYRO
    spiSetBusInstance(&gyro->dev, SPI_DEV_TO_CFG(MPU9250_SPI_BUS));
#endif
#ifdef MPU9250_CS_PIN
    gyro->dev.busType_u.spi.csnPin = gyro->dev.busType_u.spi.csnPin == IO_NONE ? IOGetByTag(IO_TAG(MPU9250_CS_PIN)) : gyro->dev.busType_u.spi.csnPin;
#endif
    sensor = mpu9250SpiDetect(&gyro->dev);
    if (sensor != MPU_NONE) {
        gyro->mpuDetectionResult.sensor = sensor;
        gyro->mpuConfiguration.resetFn = mpu9250SpiResetGyro;
        return true;
    }
#endif
#ifdef USE_GYRO_SPI_ICM20649
#ifndef USE_DUAL_GYRO
#ifdef ICM20649_SPI_BUS
    spiSetBusInstance(&gyro->dev, SPI_DEV_TO_CFG(ICM20649_SPI_BUS));
#endif
#endif
#ifdef ICM20649_CS_PIN
    gyro->dev.busType_u.spi.csnPin = gyro->dev.busType_u.spi.csnPin == IO_NONE ? IOGetByTag(IO_TAG(ICM20649_CS_PIN)) : gyro->dev.busType_u.spi.csnPin;
#endif
    sensor = icm20649SpiDetect(&gyro->dev);
    if (sensor != MPU_NONE) {
        gyro->mpuDetectionResult.sensor = sensor;
        return true;
    }
#endif
#ifdef USE_GYRO_SPI_ICM20689
#ifndef USE_DUAL_GYRO
    spiSetBusInstance(&gyro->dev, SPI_DEV_TO_CFG(ICM20689_SPI_BUS));
#endif
#ifdef ICM20689_CS_PIN
    gyro->dev.busType_u.spi.csnPin = gyro->dev.busType_u.spi.csnPin == IO_NONE ? IOGetByTag(IO_TAG(ICM20689_CS_PIN)) : gyro->dev.busType_u.spi.csnPin;
#endif
    sensor = icm20689SpiDetect(&gyro->dev);
    // icm20689SpiDetect detects ICM20602 and ICM20689
    if (sensor != MPU_NONE) {
        gyro->mpuDetectionResult.sensor = sensor;
        return true;
    }
#endif
#ifdef USE_GYRO_SPI_ICM42605
#ifndef USE_DUAL_GYRO
#ifdef ICM42605_SPI_BUS
    spiSetBusInstance(&gyro->dev, SPI_DEV_TO_CFG(ICM42605_SPI_BUS));
#endif
#endif
#ifdef ICM42605_CS_PIN
    gyro->dev.busType_u.spi.csnPin = gyro->dev.busType_u.spi.csnPin == IO_NONE ? IOGetByTag(IO_TAG(ICM42605_CS_PIN)) : gyro->dev.busType_u.spi.csnPin;
#endif
    sensor = icm426xxSpiDetect(&gyro->dev);
    if (sensor != MPU_NONE) {
        gyro->mpuDetectionResult.sensor = sensor;
        return true;
    }
#endif
#ifdef USE_GYRO_SPI_ICM42688P
#ifndef USE_DUAL_GYRO
#ifdef ICM42688P_SPI_BUS
    spiSetBusInstance(&gyro->dev, SPI_DEV_TO_CFG(ICM42688P_SPI_BUS));
#endif
#endif
#ifdef ICM42688P_CS_PIN
    gyro->dev.busType_u.spi.csnPin = gyro->dev.busType_u.spi.csnPin == IO_NONE ? IOGetByTag(IO_TAG(ICM42688P_CS_PIN)) : gyro->dev.busType_u.spi.csnPin;
#endif
    sensor = icm426xxSpiDetect(&gyro->dev);
    if (sensor != MPU_NONE) {
        gyro->mpuDetectionResult.sensor = sensor;
        return true;
    }
#endif
#ifdef USE_ACCGYRO_BMI160
#ifndef USE_DUAL_GYRO
    spiSetBusInstance(&gyro->dev, SPI_DEV_TO_CFG(BMI160_SPI_BUS));
#endif
#ifdef BMI160_CS_PIN
    gyro->dev.busType_u.spi.csnPin = gyro->dev.busType_u.spi.csnPin == IO_NONE ? IOGetByTag(IO_TAG(BMI160_CS_PIN)) : gyro->dev.busType_u.spi.csnPin;
#endif
    sensor = bmi160Detect(&gyro->dev);
    if (sensor != MPU_NONE) {
        gyro->mpuDetectionResult.sensor = sensor;
        return true;
    }
#endif
#ifdef USE_ACCGYRO_BMI270
#ifndef USE_DUAL_GYRO
    spiSetBusInstance(&gyro->dev, SPI_DEV_TO_CFG(BMI270_SPI_BUS));
#endif
#ifdef BMI270_CS_PIN
    gyro->dev.busType_u.spi.csnPin = gyro->dev.busType_u.spi.csnPin == IO_NONE ? IOGetByTag(IO_TAG(BMI270_CS_PIN)) : gyro->dev.busType_u.spi.csnPin;
#endif
    sensor = bmi270Detect(&gyro->dev);
    if (sensor != MPU_NONE) {
        gyro->mpuDetectionResult.sensor = sensor;
        return true;
    }
#endif
    return false;
}
#endif

void mpuDetect(gyroDev_t *gyro) {
    // MPU datasheet specifies 30ms.
    delay(35);
#if defined(USE_I2C)
    if (gyro->dev.bus == NULL || gyro->dev.bus->busType == BUS_TYPE_I2C) {
        i2cBusSetInstance(&gyro->dev, I2C_DEV_TO_CFG(MPU_I2C_INSTANCE));
        gyro->dev.busType_u.i2c.address = MPU_ADDRESS;
        uint8_t sig = 0;
        bool ack = busReadRegisterBuffer(&gyro->dev, MPU_RA_WHO_AM_I, &sig, 1);
        if (ack) {
            // If an MPU3050 is connected sig will contain 0.
            uint8_t inquiryResult;
            ack = busReadRegisterBuffer(&gyro->dev, MPU_RA_WHO_AM_I_LEGACY, &inquiryResult, 1);
            inquiryResult &= MPU_INQUIRY_MASK;
            if (ack && inquiryResult == MPUx0x0_WHO_AM_I_CONST) {
                gyro->mpuDetectionResult.sensor = MPU_3050;
                return;
            }
            sig &= MPU_INQUIRY_MASK;
            if (sig == MPUx0x0_WHO_AM_I_CONST) {
                gyro->mpuDetectionResult.sensor = MPU_60x0;
                mpu6050FindRevision(gyro);
            } else if (sig == MPU6500_WHO_AM_I_CONST) {
                gyro->mpuDetectionResult.sensor = MPU_65xx_I2C;
            }
            return;
        }
        // I2C probe failed; reset bus state so SPI detection starts clean.
        // Without this reset, i2cBusSetInstance will have written dev->busType_u.i2c.device
        // at union offset 0, aliasing dev->busType_u.spi.csnPin after field removal.
        gyro->dev.bus = NULL;
        memset(&gyro->dev.busType_u, 0, sizeof(gyro->dev.busType_u));
    }
#endif
#ifdef USE_SPI
    detectSPISensorsAndUpdateDetectionResult(gyro);
#endif
}

void mpuGyroInit(gyroDev_t *gyro) {
    gyro->accDataReg = MPU_RA_ACCEL_XOUT_H;
    gyro->gyroDataReg = MPU_RA_GYRO_XOUT_H;
#ifdef MPU_INT_EXTI
    mpuIntExtiInit(gyro);
#endif
}

uint8_t mpuGyroDLPF(gyroDev_t *gyro) {
    uint8_t ret;
    if (gyro->gyroRateKHz > GYRO_RATE_8_kHz) {
        ret = 0;  // If gyro is in 32KHz mode then the DLPF bits aren't used - set to 0
    } else {
        switch (gyro->hardware_lpf) {
        case GYRO_HARDWARE_LPF_NORMAL:
            ret = 0;
            break;
        case GYRO_HARDWARE_LPF_EXPERIMENTAL:
            ret = 7;
            break;
        case GYRO_HARDWARE_LPF_1KHZ_SAMPLE:
            ret = 1;
            break;
        default:
            ret = 0;
            break;
        }
    }
    return ret;
}

uint8_t mpuGyroFCHOICE(gyroDev_t *gyro) {
    if (gyro->gyroRateKHz > GYRO_RATE_8_kHz) {
        if (gyro->hardware_32khz_lpf == GYRO_32KHZ_HARDWARE_LPF_EXPERIMENTAL) {
            return FCB_8800_32;
        } else {
            return FCB_3600_32;
        }
    } else {
        return FCB_DISABLED;  // Not in 32KHz mode, set FCHOICE to select 8KHz sampling
    }
}

#ifdef USE_GYRO_REGISTER_DUMP
uint8_t mpuGyroReadRegister(const extDevice_t *dev, uint8_t reg) {
    uint8_t data;
    const bool ack = busReadRegisterBuffer(dev, reg, &data, 1);
    if (ack) {
        return data;
    } else {
        return 0;
    }
}
#endif
