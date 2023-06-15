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

#include "platform.h"

#ifdef USE_ACCGYRO_BMI270

#include "drivers/accgyro/accgyro.h"
#include "drivers/accgyro/accgyro_spi_bmi270.h"
#include "drivers/bus_spi.h"
#include "drivers/exti.h"
#include "drivers/io.h"
#include "drivers/io_impl.h"
#include "drivers/nvic.h"
#include "drivers/sensor.h"
#include "drivers/time.h"

// 10 MHz max SPI frequency
#define BMI270_MAX_SPI_CLK_HZ 10000000

#define BMI270_FIFO_FRAME_SIZE 6

#define BMI270_CONFIG_SIZE 8192

// Declaration for the device config (microcode) that must be uploaded to the sensor
//help
// extern const uint8_t bmi270_config_file[BMI270_CONFIG_SIZE];

#define BMI270_CHIP_ID 0x24

// BMI270 register configuration values
typedef enum {
    BMI270_VAL_CMD_G_TRIGGER = 0x02,
    BMI270_VAL_CMD_SOFTRESET = 0xB6,
    BMI270_VAL_CMD_FIFOFLUSH = 0xB0,
    BMI270_VAL_PWR_CTRL_DISABLE_ALL = 0x00,             // disable all sensors
    BMI270_VAL_PWR_CTRL = 0x0E,                         // enable gyro, acc and temp sensors
    BMI270_VAL_PWR_CTRL_ACC_ENABLE = BIT(2),            // bit 2, enable acc
    BMI270_VAL_PWR_CTRL_ACC_DISABLE = 0x00,             // bit 2, disable gyro
    BMI270_VAL_PWR_CONF_ADV_POWER_SAVE_DISABLE = 0x00,  // disable ADV PS
    BMI270_VAL_PWR_CONF_ADV_POWER_SAVE_ENABLE = BIT(0), // enable ADV PS
    BMI270_VAL_PWR_CONF = 0x02,                         // disable advanced power save, enable FIFO self-wake
    BMI270_VAL_PAGE_0 = 0x00,                           // select page 0
    BMI270_VAL_PAGE_1 = 0x01,                           // select page 1
    BMI270_VAL_ACC_CONF_ODR800 = 0x0B,                  // set acc sample rate to 800hz
    BMI270_VAL_ACC_CONF_ODR1600 = 0x0C,                 // set acc sample rate to 1600hz
    BMI270_VAL_ACC_CONF_BWP = 0x01,                     // set acc filter in osr2 mode
    BMI270_VAL_ACC_CONF_HP = 0x01,                      // set acc in high performance mode
    BMI270_VAL_ACC_RANGE_8G = 0x02,                     // set acc to 8G full scale
    BMI270_VAL_ACC_RANGE_16G = 0x03,                    // set acc to 16G full scale
    BMI270_VAL_GYRO_CONF_ODR3200 = 0x0D,                // set gyro sample rate to 3200hz
    BMI270_VAL_GYRO_CONF_BWP_OSR4 = 0x00,               // set gyro filter in OSR4 mode
    BMI270_VAL_GYRO_CONF_BWP_OSR2 = 0x01,               // set gyro filter in OSR2 mode
    BMI270_VAL_GYRO_CONF_BWP_NORM = 0x02,               // set gyro filter in normal mode
    BMI270_VAL_GYRO_CONF_NOISE_PERF = 0x01,             // set gyro in high performance noise mode
    BMI270_VAL_GYRO_CONF_FILTER_PERF = 0x01,            // set gyro in high performance filter mode

    BMI270_VAL_GYRO_RANGE_2000DPS = 0x08,               // set gyro to 2000dps full scale
                                                        // for some reason you have to enable the ois_range bit (bit 3) for 2000dps as well
                                                        // or else the gyro scale will be 250dps when in prefiltered FIFO mode (not documented in datasheet!)

    BMI270_VAL_INT_MAP_DATA_DRDY_INT1 = 0x04,           // enable the data ready interrupt pin 1
    BMI270_VAL_INT_MAP_FIFO_WM_INT1 = 0x02,             // enable the FIFO watermark interrupt pin 1
    BMI270_VAL_INT1_IO_CTRL_PINMODE = 0x0A,             // active high, push-pull, output enabled, input disabled 
    BMI270_VAL_IF_CONF_SET_INTERFACE = 0x00,            // spi 4-wire mode, disable OIS, disable AUX
    BMI270_VAL_FIFO_CONFIG_0 = 0x00,                    // don't stop when full, disable sensortime frame
    BMI270_VAL_FIFO_CONFIG_1 = 0x80,                    // only gyro data in FIFO, use headerless mode
    BMI270_VAL_FIFO_DOWNS = 0x00,                       // select unfiltered gyro data with no downsampling (6.4KHz samples)
    BMI270_VAL_FIFO_WTM_0 = 0x06,                       // set the FIFO watermark level to 1 gyro sample (6 bytes)
    BMI270_VAL_FIFO_WTM_1 = 0x00,                       // FIFO watermark MSB
    BMI270_VAL_GEN_SET_1 = 0x0200,                      // bit 9, enable self offset correction (IOC part 1)
    BMI270_VAL_OFFSET_6 = 0xC0,                         // Enable sensitivity error compensation and gyro offset compensation (IOC part2)
    BMI270_VAL_OFFSET_6_GYR_GAIN_EN_ENABLE = BIT(7),    // bit 7, enable gyro gain compensation
    BMI270_VAL_GYR_CRT_CONF_CRT_RUNNING = BIT(2),       // bit 7, enable gyro crt
    BMI270_VAL_FEATURES_1_G_TRIG_1_SELECT_CRT = 0x0100, // bit 8, CRT will be executed
    BMI270_VAL_FEATURES_1_G_TRIG_1_SELECT_GYR_BIST=0x0000, // bit 8, gyro built-in self-test will be executed
    BMI270_VAL_FEATURES_1_G_TRIG_1_BLOCK_UNLOCK = 0x0000,// bit 9, do not block further G_TRIGGER commands
    BMI270_VAL_FEATURES_1_G_TRIG_1_BLOCK_BLOCK = 0x0200, // bit 9, block further G_TRIGGER commands
} bmi270ConfigValues_e;

typedef enum {
    BMI270_MASK_FEATURES_1_GEN_SET_1 = 0x0200,
    BMI270_MASK_OFFSET_6 = 0xC0,
    BMI270_MASK_OFFSET_6_GYR_GAIN_ENABLE = 0x80,        // bit 7, enable gyro gain compensation
    BMI270_MASK_PWR_ADV_POWER_SAVE = 0x01,              // bit 0, enable advanced power save
    BMI270_MASK_PWR_CTRL_ACC_ENABLE = 0x04,             // bit 2, enable acc
    BMI270_MASK_GYR_CRT_CONF_CRT_RUNNING = 0x04,        // bit 2, gyro CRT running
    BMI270_MASK_G_TRIG_1_SELECT = 0x0100,               // bit 8, select feature that should be executed
    BMI270_MASK_G_TRIG_1_BLOCK = 0x0200,                // bit 9, block feature with next G_TRIGGER CMD
    BMI270_MASK_GYR_GAIN_STATUS_G_TRIG_STATUS = 0x38    // bit[5:3]
} bmi270ConfigMasks_e;

// BMI270 register reads are 16bits with the first byte a "dummy" value 0
// that must be ignored. The result is in the second byte.
static uint8_t bmi270RegisterRead(const busDevice_t *bus, bmi270Register_e registerId)
{
    uint8_t data[2] = { 0, 0 };

    if (spiBusReadRegisterBuffer(bus, registerId, data, 2)) {
        return data[1];
    } else {
        return 0;
    }
}

static void bmi270RegisterWrite(const busDevice_t *bus, bmi270Register_e registerId, uint8_t value, unsigned delayMs)
{
    spiBusWriteRegister(bus, registerId, value);
    if (delayMs) {
        delay(delayMs);
    }
}

// Toggle the CS to switch the device into SPI mode.
// Device switches initializes as I2C and switches to SPI on a low to high CS transition
static void bmi270EnableSPI(const busDevice_t *bus)
{
    IOLo(bus->busdev_u.spi.csnPin);
    delay(1);
    IOHi(bus->busdev_u.spi.csnPin);
    delay(10);
}

// help
// static void bmi270UploadConfig(const busDevice_t *bus)
// {
//     bmi270RegisterWrite(bus, BMI270_REG_PWR_CONF, 0, 1);
//     bmi270RegisterWrite(bus, BMI270_REG_INIT_CTRL, 0, 1);
// 
//     // Transfer the config file
//     IOLo(bus->busdev_u.spi.csnPin);
//     spiTransferByte(bus->busdev_u.spi.instance, BMI270_REG_INIT_DATA);
//     spiTransfer(bus->busdev_u.spi.instance, bmi270_config_file, NULL, sizeof(bmi270_config_file));
//     IOHi(bus->busdev_u.spi.csnPin);
// 
//     delay(10);
//     bmi270RegisterWrite(bus, BMI270_REG_INIT_CTRL, 1, 1);
// }

extiCallbackRec_t bmi270IntCallbackRec;

#if defined(USE_GYRO_EXTI) && defined(USE_MPU_DATA_READY_SIGNAL)
void bmi270ExtiHandler(extiCallbackRec_t *cb)
{
    gyroDev_t *gyro = container_of(cb, gyroDev_t, exti);
    gyro->dataReady = true;
}

static void bmi270IntExtiInit(gyroDev_t *gyro)
{
    if (gyro->mpuIntExtiTag == IO_TAG_NONE) {
        return;
    }

    IO_t mpuIntIO = IOGetByTag(gyro->mpuIntExtiTag);

    IOInit(mpuIntIO, OWNER_GYRO_EXTI, 0);
    EXTIHandlerInit(&gyro->exti, bmi270ExtiHandler);
    EXTIConfig(mpuIntIO, &gyro->exti, NVIC_PRIO_MPU_INT_EXTI, IOCFG_IN_FLOATING );
    EXTIEnable(mpuIntIO, true);
}
#endif

// help
// bool bmi270AccRead(accDev_t *acc)
// {
//     switch (acc->gyro->gyroModeSPI) {
//     case GYRO_EXTI_INT:
//     case GYRO_EXTI_NO_INT:
//     {
//         acc->gyro->dev.txBuf[0] = BMI270_REG_ACC_DATA_X_LSB | 0x80;
// 
//         busSegment_t segments[] = {
//                 {.u.buffers = {NULL, NULL}, 8, true, NULL},
//                 {.u.link = {NULL, NULL}, 0, true, NULL},
//         };
//         segments[0].u.buffers.txData = acc->gyro->dev.txBuf;
//         segments[0].u.buffers.rxData = acc->gyro->dev.rxBuf;
// 
//         spiSequence(&acc->gyro->dev, &segments[0]);
// 
//         // Wait for completion
//         spiWait(&acc->gyro->dev);
// 
//         // Fall through
//         FALLTHROUGH;
//     }
// 
//     case GYRO_EXTI_INT_DMA:
//     {
//         // If read was triggered in interrupt don't bother waiting. The worst that could happen is that we pick
//         // up an old value.
// 
//         // This data was read from the gyro, which is the same SPI device as the acc
//         int16_t *accData = (int16_t *)acc->gyro->dev.rxBuf;
//         acc->ADCRaw[X] = accData[1];
//         acc->ADCRaw[Y] = accData[2];
//         acc->ADCRaw[Z] = accData[3];
//         break;
//     }
// 
//     case GYRO_EXTI_INIT:
//     default:
//         break;
//     }
// 
//     return true;
// }

bool bmi270AccRead(accDev_t *acc)
{
    enum {
        IDX_REG = 0,
        IDX_SKIP,
        IDX_ACCEL_XOUT_L,
        IDX_ACCEL_XOUT_H,
        IDX_ACCEL_YOUT_L,
        IDX_ACCEL_YOUT_H,
        IDX_ACCEL_ZOUT_L,
        IDX_ACCEL_ZOUT_H,
        BUFFER_SIZE,
    };

    uint8_t bmi270_rx_buf[BUFFER_SIZE];
    static const uint8_t bmi270_tx_buf[BUFFER_SIZE] = {BMI270_REG_ACC_DATA_X_LSB | 0x80, 0, 0, 0, 0, 0, 0, 0};

    IOLo(acc->bus.busdev_u.spi.csnPin);
    spiTransfer(acc->bus.busdev_u.spi.instance, bmi270_tx_buf, bmi270_rx_buf, BUFFER_SIZE);   // receive response
    IOHi(acc->bus.busdev_u.spi.csnPin);

    acc->ADCRaw[X] = (int16_t)((bmi270_rx_buf[IDX_ACCEL_XOUT_H] << 8) | bmi270_rx_buf[IDX_ACCEL_XOUT_L]);
    acc->ADCRaw[Y] = (int16_t)((bmi270_rx_buf[IDX_ACCEL_YOUT_H] << 8) | bmi270_rx_buf[IDX_ACCEL_YOUT_L]);
    acc->ADCRaw[Z] = (int16_t)((bmi270_rx_buf[IDX_ACCEL_ZOUT_H] << 8) | bmi270_rx_buf[IDX_ACCEL_ZOUT_L]);

    return true;
}

bool bmi270GyroReadRegister(gyroDev_t *gyro)
{
    enum {
        IDX_REG = 0,
        IDX_SKIP,
        IDX_GYRO_XOUT_L,
        IDX_GYRO_XOUT_H,
        IDX_GYRO_YOUT_L,
        IDX_GYRO_YOUT_H,
        IDX_GYRO_ZOUT_L,
        IDX_GYRO_ZOUT_H,
        BUFFER_SIZE,
    };

    uint8_t bmi270_rx_buf[BUFFER_SIZE];
    static const uint8_t bmi270_tx_buf[BUFFER_SIZE] = {BMI270_REG_GYR_DATA_X_LSB | 0x80, 0, 0, 0, 0, 0, 0, 0};

    IOLo(gyro->bus.busdev_u.spi.csnPin);
    spiTransfer(gyro->bus.busdev_u.spi.instance, bmi270_tx_buf, bmi270_rx_buf, BUFFER_SIZE);   // receive response
    IOHi(gyro->bus.busdev_u.spi.csnPin);

    gyro->gyroADCRaw[X] = (int16_t)((bmi270_rx_buf[IDX_GYRO_XOUT_H] << 8) | bmi270_rx_buf[IDX_GYRO_XOUT_L]);
    gyro->gyroADCRaw[Y] = (int16_t)((bmi270_rx_buf[IDX_GYRO_YOUT_H] << 8) | bmi270_rx_buf[IDX_GYRO_YOUT_L]);
    gyro->gyroADCRaw[Z] = (int16_t)((bmi270_rx_buf[IDX_GYRO_ZOUT_H] << 8) | bmi270_rx_buf[IDX_GYRO_ZOUT_L]);

    return true;
}

#ifdef USE_GYRO_DLPF_EXPERIMENTAL
static bool bmi270GyroReadFifo(gyroDev_t *gyro)
{
    enum {
        IDX_REG = 0,
        IDX_SKIP,
        IDX_FIFO_LENGTH_L,
        IDX_FIFO_LENGTH_H,
        IDX_GYRO_XOUT_L,
        IDX_GYRO_XOUT_H,
        IDX_GYRO_YOUT_L,
        IDX_GYRO_YOUT_H,
        IDX_GYRO_ZOUT_L,
        IDX_GYRO_ZOUT_H,
        BUFFER_SIZE,
    };

    bool dataRead = false;
    static const uint8_t bmi270_tx_buf[BUFFER_SIZE] = {BMI270_REG_FIFO_LENGTH_LSB | 0x80, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    uint8_t bmi270_rx_buf[BUFFER_SIZE];

    // Burst read the FIFO length followed by the next 6 bytes containing the gyro axis data for
    // the first sample in the queue. It's possible for the FIFO to be empty so we need to check the
    // length before using the sample.
    IOLo(gyro->bus.busdev_u.spi.csnPin);
    spiTransfer(gyro->bus.busdev_u.spi.instance, bmi270_tx_buf, bmi270_rx_buf, BUFFER_SIZE);   // receive response
    IOHi(gyro->bus.busdev_u.spi.csnPin);

    int fifoLength = (uint16_t)((bmi270_rx_buf[IDX_FIFO_LENGTH_H] << 8) | bmi270_rx_buf[IDX_FIFO_LENGTH_L]);

    if (fifoLength >= BMI270_FIFO_FRAME_SIZE) {

        const int16_t gyroX = (int16_t)((bmi270_rx_buf[IDX_GYRO_XOUT_H] << 8) | bmi270_rx_buf[IDX_GYRO_XOUT_L]);
        const int16_t gyroY = (int16_t)((bmi270_rx_buf[IDX_GYRO_YOUT_H] << 8) | bmi270_rx_buf[IDX_GYRO_YOUT_L]);
        const int16_t gyroZ = (int16_t)((bmi270_rx_buf[IDX_GYRO_ZOUT_H] << 8) | bmi270_rx_buf[IDX_GYRO_ZOUT_L]);

        // If the FIFO data is invalid then the returned values will be 0x8000 (-32768) (pg. 43 of datasheet).
        // This shouldn't happen since we're only using the data if the FIFO length indicates
        // that data is available, but this safeguard is needed to prevent bad things in
        // case it does happen.
        if ((gyroX != INT16_MIN) || (gyroY != INT16_MIN) || (gyroZ != INT16_MIN)) {
            gyro->gyroADCRaw[X] = gyroX;
            gyro->gyroADCRaw[Y] = gyroY;
            gyro->gyroADCRaw[Z] = gyroZ;
            dataRead = true;
        }
        fifoLength -= BMI270_FIFO_FRAME_SIZE;
    }

    // If there are additional samples in the FIFO then we don't use those for now and simply
    // flush the FIFO. Under normal circumstances we only expect one sample in the FIFO since
    // the gyro loop is running at the native sample rate of 6.4KHz.
    // However the way the FIFO works in the sensor is that if a frame is partially read then
    // it remains in the queue instead of bein removed. So if we ever got into a state where there
    // was a partial frame or other unexpected data in the FIFO is may never get cleared and we
    // would end up in a lock state of always re-reading the same partial or invalid sample.
    if (fifoLength > 0) {
        // Partial or additional frames left - flush the FIFO
        bmi270RegisterWrite(&gyro->bus, BMI270_REG_CMD, BMI270_VAL_CMD_FIFOFLUSH, 0);
    }

    return dataRead;
}
#endif

bool bmi270GyroRead(gyroDev_t *gyro)
{
#ifdef USE_GYRO_DLPF_EXPERIMENTAL
    if (gyro->hardware_lpf == GYRO_HARDWARE_LPF_EXPERIMENTAL) {
        // running in 6.4KHz FIFO mode
        return bmi270GyroReadFifo(gyro);
    } else
#endif
    {
        // running in 3.2KHz register mode
        return bmi270GyroReadRegister(gyro);
    }
}

//help
// static void bmi270SpiGyroInit(gyroDev_t *gyro)
// {
//     bmi270Config(gyro);
// 
// #if defined(USE_GYRO_EXTI) && defined(USE_MPU_DATA_READY_SIGNAL)
//     bmi270IntExtiInit(gyro);
// #endif
// }

//help
// static void bmi270SpiAccInit(accDev_t *acc)
// {
//     // sensor is configured during gyro init
//     acc->acc_1G = 512 * 4;   // 16G sensor scale
// }


//help
// bool bmi270SpiGyroDetect(gyroDev_t *gyro)
// {
//     if (gyro->mpuDetectionResult.sensor != BMI_270_SPI) {
//         return false;
//     }
// 
//     gyro->initFn = bmi270SpiGyroInit;
//     gyro->readFn = bmi270GyroRead;
//     gyro->scale = GYRO_SCALE_2000DPS;
// 
//     return true;
// }

// Used to query the status register to determine what event caused the EXTI to fire.
// When in 3.2KHz mode the interrupt is mapped to the data ready state. However the data ready
// trigger will fire for both gyro and accelerometer. So it's necessary to check this register
// to determine which event caused the interrupt.
// When in 6.4KHz mode the interrupt is configured to be the FIFO watermark size of 6 bytes.
// Since in this mode we only put gyro data in the FIFO it's sufficient to check for the FIFO
// watermark reason as an idication of gyro data ready.
uint8_t bmi270InterruptStatus(gyroDev_t *gyro)
{
    return bmi270RegisterRead(&gyro->bus, BMI270_REG_INT_STATUS_1);
}
#endif // USE_ACCGYRO_BMI270
