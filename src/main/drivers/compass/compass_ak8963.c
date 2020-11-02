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

#if defined(USE_MAG_AK8963) || defined(USE_MAG_SPI_AK8963)

#include "build/debug.h"

#include "common/axis.h"
#include "common/maths.h"
#include "common/utils.h"

#include "drivers/bus.h"
#include "drivers/bus_i2c.h"
#include "drivers/bus_i2c_busdev.h"
#include "drivers/bus_spi.h"
#include "drivers/io.h"
#include "drivers/sensor.h"
#include "drivers/time.h"

#include "drivers/compass/compass.h"

#include "drivers/accgyro/accgyro.h"
#include "drivers/accgyro/accgyro_mpu.h"
#include "drivers/accgyro/accgyro_mpu6500.h"
#include "drivers/accgyro/accgyro_spi_mpu6500.h"
#include "drivers/accgyro/accgyro_spi_mpu9250.h"
#include "drivers/compass/compass_ak8963.h"

#include "scheduler/scheduler.h"

// This sensor is also available also part of the MPU-9250 connected to the secondary I2C bus.

// AK8963, mag sensor address
#define AK8963_MAG_I2C_ADDRESS          0x0C
#define AK8963_DEVICE_ID                0x48

// Registers
#define AK8963_MAG_REG_WIA              0x00
#define AK8963_MAG_REG_INFO             0x01
#define AK8963_MAG_REG_ST1              0x02
#define AK8963_MAG_REG_HXL              0x03
#define AK8963_MAG_REG_HXH              0x04
#define AK8963_MAG_REG_HYL              0x05
#define AK8963_MAG_REG_HYH              0x06
#define AK8963_MAG_REG_HZL              0x07
#define AK8963_MAG_REG_HZH              0x08
#define AK8963_MAG_REG_ST2              0x09
#define AK8963_MAG_REG_CNTL1            0x0A
#define AK8963_MAG_REG_CNTL2            0x0B
#define AK8963_MAG_REG_ASCT             0x0C // self test
#define AK8963_MAG_REG_I2CDIS           0x0F
#define AK8963_MAG_REG_ASAX             0x10 // Fuse ROM x-axis sensitivity adjustment value
#define AK8963_MAG_REG_ASAY             0x11 // Fuse ROM y-axis sensitivity adjustment value
#define AK8963_MAG_REG_ASAZ             0x12 // Fuse ROM z-axis sensitivity adjustment value

#define READ_FLAG                       0x80
#define I2C_SLV0_EN                     0x80

#define ST1_DATA_READY                  0x01
#define ST1_DATA_OVERRUN                0x02

#define ST2_MAG_SENSOR_OVERFLOW         0x08

#define CNTL1_MODE_POWER_DOWN           0x00
#define CNTL1_MODE_ONCE                 0x01
#define CNTL1_MODE_CONT1                0x02
#define CNTL1_MODE_CONT2                0x06
#define CNTL1_MODE_SELF_TEST            0x08
#define CNTL1_MODE_FUSE_ROM             0x0F
#define CNTL1_BIT_14_BIT                0x00
#define CNTL1_BIT_16_BIT                0x10

#define CNTL2_SOFT_RESET                0x01

#define I2CDIS_DISABLE_MASK             0x1D

#if defined(USE_MAG_AK8963) && (defined(USE_GYRO_SPI_MPU6500) || defined(USE_GYRO_SPI_MPU9250))

static bool ak8963SpiWriteRegisterDelay(const busDevice_t *bus, uint8_t reg, uint8_t data) {
    spiBusWriteRegister(bus, reg, data);
    delayMicroseconds(10);
    return true;
}

static bool ak8963SlaveReadRegisterBuffer(const busDevice_t *slavedev, uint8_t reg, uint8_t *buf, uint8_t len) {
    const busDevice_t *bus = slavedev->busdev_u.mpuSlave.master;
    ak8963SpiWriteRegisterDelay(bus, MPU_RA_I2C_SLV0_ADDR, slavedev->busdev_u.mpuSlave.address | READ_FLAG); // set I2C slave address for read
    ak8963SpiWriteRegisterDelay(bus, MPU_RA_I2C_SLV0_REG, reg);                             // set I2C slave register
    ak8963SpiWriteRegisterDelay(bus, MPU_RA_I2C_SLV0_CTRL, (len & 0x0F) | I2C_SLV0_EN);     // read number of bytes
    delay(4);
    __disable_irq();
    bool ack = spiBusReadRegisterBuffer(bus, MPU_RA_EXT_SENS_DATA_00, buf, len);            // read I2C
    __enable_irq();
    return ack;
}

static bool ak8963SlaveWriteRegister(const busDevice_t *slavedev, uint8_t reg, uint8_t data) {
    const busDevice_t *bus = slavedev->busdev_u.mpuSlave.master;
    ak8963SpiWriteRegisterDelay(bus, MPU_RA_I2C_SLV0_ADDR, slavedev->busdev_u.mpuSlave.address); // set I2C slave address for write
    ak8963SpiWriteRegisterDelay(bus, MPU_RA_I2C_SLV0_REG, reg);                             // set I2C slave register
    ak8963SpiWriteRegisterDelay(bus, MPU_RA_I2C_SLV0_DO, data);                             // set I2C sLave value
    ak8963SpiWriteRegisterDelay(bus, MPU_RA_I2C_SLV0_CTRL, (1 & 0x0F) | I2C_SLV0_EN);       // write 1 byte
    return true;
}

typedef struct queuedReadState_s {
    bool waiting;
    uint8_t len;
    uint32_t readStartedAt;                                                                 // time read was queued in micros.
} queuedReadState_t;

static queuedReadState_t queuedRead = { false, 0, 0};

static bool ak8963SlaveStartRead(const busDevice_t *slavedev, uint8_t reg, uint8_t len) {
    if (queuedRead.waiting) {
        return false;
    }
    const busDevice_t *bus = slavedev->busdev_u.mpuSlave.master;
    queuedRead.len = len;
    ak8963SpiWriteRegisterDelay(bus, MPU_RA_I2C_SLV0_ADDR, slavedev->busdev_u.mpuSlave.address | READ_FLAG);  // set I2C slave address for read
    ak8963SpiWriteRegisterDelay(bus, MPU_RA_I2C_SLV0_REG, reg);                             // set I2C slave register
    ak8963SpiWriteRegisterDelay(bus, MPU_RA_I2C_SLV0_CTRL, (len & 0x0F) | I2C_SLV0_EN);    // read number of bytes
    queuedRead.readStartedAt = micros();
    queuedRead.waiting = true;
    return true;
}

static uint32_t ak8963SlaveQueuedReadTimeRemaining(void) {
    if (!queuedRead.waiting) {
        return 0;
    }
    int32_t timeSinceStarted = micros() - queuedRead.readStartedAt;
    int32_t timeRemaining = 8000 - timeSinceStarted;
    if (timeRemaining < 0) {
        return 0;
    }
    return timeRemaining;
}

static bool ak8963SlaveCompleteRead(const busDevice_t *slavedev, uint8_t *buf) {
    uint32_t timeRemaining = ak8963SlaveQueuedReadTimeRemaining();
    const busDevice_t *bus = slavedev->busdev_u.mpuSlave.master;
    if (timeRemaining > 0) {
        delayMicroseconds(timeRemaining);
    }
    queuedRead.waiting = false;
    spiBusReadRegisterBuffer(bus, MPU_RA_EXT_SENS_DATA_00, buf, queuedRead.len);            // read I2C buffer
    return true;
}

static bool ak8963SlaveReadData(const busDevice_t *busdev, uint8_t *buf) {
    typedef enum {
        CHECK_STATUS = 0,
        WAITING_FOR_STATUS,
        WAITING_FOR_DATA
    } ak8963ReadState_e;
    static ak8963ReadState_e state = CHECK_STATUS;
    bool ack = false;
    // we currently need a different approach for the MPU9250 connected via SPI.
    // we cannot use the ak8963SlaveReadRegisterBuffer() method for SPI, it is to slow and blocks for far too long.
    bool retry = true;
restart:
    switch (state) {
    case CHECK_STATUS: {
        ak8963SlaveStartRead(busdev, AK8963_MAG_REG_ST1, 1);
        state = WAITING_FOR_STATUS;
        return false;
    }
    case WAITING_FOR_STATUS: {
        uint32_t timeRemaining = ak8963SlaveQueuedReadTimeRemaining();
        if (timeRemaining) {
            return false;
        }
        ack = ak8963SlaveCompleteRead(busdev, &buf[0]);
        uint8_t status = buf[0];
        if (!ack || (status & ST1_DATA_READY) == 0) {
            // too early. queue the status read again
            state = CHECK_STATUS;
            if (retry) {
                retry = false;
                goto restart;
            }
            return false;
        }
        // read the 6 bytes of data and the status2 register
        ak8963SlaveStartRead(busdev, AK8963_MAG_REG_HXL, 7);
        state = WAITING_FOR_DATA;
        return false;
    }
    case WAITING_FOR_DATA: {
        uint32_t timeRemaining = ak8963SlaveQueuedReadTimeRemaining();
        if (timeRemaining) {
            return false;
        }
        ack = ak8963SlaveCompleteRead(busdev, &buf[0]);
        state = CHECK_STATUS;
    }
    }
    return ack;
}
#endif

static bool ak8963ReadRegisterBuffer(const busDevice_t *busdev, uint8_t reg, uint8_t *buf, uint8_t len) {
#if defined(USE_MAG_AK8963) && (defined(USE_GYRO_SPI_MPU6500) || defined(USE_GYRO_SPI_MPU9250))
    if (busdev->bustype == BUSTYPE_MPU_SLAVE) {
        return ak8963SlaveReadRegisterBuffer(busdev, reg, buf, len);
    }
#endif
    return busReadRegisterBuffer(busdev, reg, buf, len);
}

static bool ak8963WriteRegister(const busDevice_t *busdev, uint8_t reg, uint8_t data) {
#if defined(USE_MAG_AK8963) && (defined(USE_GYRO_SPI_MPU6500) || defined(USE_GYRO_SPI_MPU9250))
    if (busdev->bustype == BUSTYPE_MPU_SLAVE) {
        return ak8963SlaveWriteRegister(busdev, reg, data);
    }
#endif
    return busWriteRegister(busdev, reg, data);
}

static bool ak8963DirectReadData(const busDevice_t *busdev, uint8_t *buf) {
    uint8_t status;
    bool ack = ak8963ReadRegisterBuffer(busdev, AK8963_MAG_REG_ST1, &status, 1);
    if (!ack || (status & ST1_DATA_READY) == 0) {
        return false;
    }
    return ak8963ReadRegisterBuffer(busdev, AK8963_MAG_REG_HXL, buf, 7);
}

static int16_t parseMag(uint8_t *raw, int16_t gain) {
    int ret = (int16_t)(raw[1] << 8 | raw[0]) * gain / 256;
    return constrain(ret, INT16_MIN, INT16_MAX);
}

static bool ak8963Read(magDev_t *mag, int16_t *magData) {
    bool ack = false;
    uint8_t buf[7];
    const busDevice_t *busdev = &mag->busdev;
    switch (busdev->bustype) {
#if defined(USE_MAG_SPI_AK8963) || defined(USE_MAG_AK8963)
    case BUSTYPE_I2C:
    case BUSTYPE_SPI:
        ack = ak8963DirectReadData(busdev, buf);
        break;
#endif
#if defined(USE_MAG_AK8963) && (defined(USE_GYRO_SPI_MPU6500) || defined(USE_GYRO_SPI_MPU9250))
    case BUSTYPE_MPU_SLAVE:
        ack = ak8963SlaveReadData(busdev, buf);
        break;
#endif
    default:
        break;
    }
    uint8_t status2 = buf[6];
    if (!ack) {
        return false;
    }
    ak8963WriteRegister(busdev, AK8963_MAG_REG_CNTL1, CNTL1_BIT_16_BIT | CNTL1_MODE_ONCE); // start reading again    uint8_t status2 = buf[6];
    if (status2 & ST2_MAG_SENSOR_OVERFLOW) {
        return false;
    }
    magData[X] = parseMag(buf + 0, mag->magGain[X]);
    magData[Y] = parseMag(buf + 2, mag->magGain[Y]);
    magData[Z] = parseMag(buf + 4, mag->magGain[Z]);
    return true;
}

static bool ak8963Init(magDev_t *mag) {
    uint8_t asa[3];
    uint8_t status;
    const busDevice_t *busdev = &mag->busdev;
    ak8963WriteRegister(busdev, AK8963_MAG_REG_CNTL1, CNTL1_MODE_POWER_DOWN);               // power down before entering fuse mode
    ak8963WriteRegister(busdev, AK8963_MAG_REG_CNTL1, CNTL1_MODE_FUSE_ROM);                 // Enter Fuse ROM access mode
    ak8963ReadRegisterBuffer(busdev, AK8963_MAG_REG_ASAX, asa, sizeof(asa));                // Read the x-, y-, and z-axis calibration values
    mag->magGain[X] = asa[X] + 128;
    mag->magGain[Y] = asa[Y] + 128;
    mag->magGain[Z] = asa[Z] + 128;
    ak8963WriteRegister(busdev, AK8963_MAG_REG_CNTL1, CNTL1_MODE_POWER_DOWN);               // power down after reading.
    // Clear status registers
    ak8963ReadRegisterBuffer(busdev, AK8963_MAG_REG_ST1, &status, 1);
    ak8963ReadRegisterBuffer(busdev, AK8963_MAG_REG_ST2, &status, 1);
    // Trigger first measurement
    ak8963WriteRegister(busdev, AK8963_MAG_REG_CNTL1, CNTL1_BIT_16_BIT | CNTL1_MODE_ONCE);
    return true;
}

void ak8963BusInit(const busDevice_t *busdev) {
    switch (busdev->bustype) {
#ifdef USE_MAG_AK8963
    case BUSTYPE_I2C:
        UNUSED(busdev);
        break;
#endif
#ifdef USE_MAG_SPI_AK8963
    case BUSTYPE_SPI:
        IOHi(busdev->busdev_u.spi.csnPin);                                                  // Disable
        IOInit(busdev->busdev_u.spi.csnPin, OWNER_COMPASS_CS, 0);
        IOConfigGPIO(busdev->busdev_u.spi.csnPin, IOCFG_OUT_PP);
        spiSetDivisor(busdev->busdev_u.spi.instance, SPI_CLOCK_STANDARD);
        break;
#endif
#if defined(USE_MAG_AK8963) && (defined(USE_GYRO_SPI_MPU6500) || defined(USE_GYRO_SPI_MPU9250))
    case BUSTYPE_MPU_SLAVE:
        rescheduleTask(TASK_COMPASS, TASK_PERIOD_HZ(40));
        // initialze I2C master via SPI bus
        ak8963SpiWriteRegisterDelay(busdev->busdev_u.mpuSlave.master, MPU_RA_INT_PIN_CFG, MPU6500_BIT_INT_ANYRD_2CLEAR | MPU6500_BIT_BYPASS_EN);
        ak8963SpiWriteRegisterDelay(busdev->busdev_u.mpuSlave.master, MPU_RA_I2C_MST_CTRL, 0x0D); // I2C multi-master / 400kHz
        ak8963SpiWriteRegisterDelay(busdev->busdev_u.mpuSlave.master, MPU_RA_USER_CTRL, 0x30);   // I2C master mode, SPI mode only
        break;
#endif
    default:
        break;
    }
}

void ak8963BusDeInit(const busDevice_t *busdev) {
    switch (busdev->bustype) {
#ifdef USE_MAG_AK8963
    case BUSTYPE_I2C:
        UNUSED(busdev);
        break;
#endif
#ifdef USE_MAG_SPI_AK8963
    case BUSTYPE_SPI:
        spiPreinitCsByIO(busdev->busdev_u.spi.csnPin);
        break;
#endif
#if defined(USE_MAG_AK8963) && (defined(USE_GYRO_SPI_MPU6500) || defined(USE_GYRO_SPI_MPU9250))
    case BUSTYPE_MPU_SLAVE:
        ak8963SpiWriteRegisterDelay(busdev->busdev_u.mpuSlave.master, MPU_RA_INT_PIN_CFG, MPU6500_BIT_INT_ANYRD_2CLEAR);
        break;
#endif
    default:
        break;
    }
}

bool ak8963Detect(magDev_t *mag) {
    uint8_t sig = 0;
    busDevice_t *busdev = &mag->busdev;
    if ((busdev->bustype == BUSTYPE_I2C || busdev->bustype == BUSTYPE_MPU_SLAVE) && busdev->busdev_u.mpuSlave.address == 0) {
        busdev->busdev_u.mpuSlave.address = AK8963_MAG_I2C_ADDRESS;
    }
    ak8963BusInit(busdev);
    ak8963WriteRegister(busdev, AK8963_MAG_REG_CNTL2, CNTL2_SOFT_RESET);                    // reset MAG
    delay(4);
    bool ack = ak8963ReadRegisterBuffer(busdev, AK8963_MAG_REG_WIA, &sig, 1);               // check for AK8963
    if (ack && sig == AK8963_DEVICE_ID) { // 0x48 / 01001000 / 'H'
        mag->init = ak8963Init;
        mag->read = ak8963Read;
        return true;
    }
    ak8963BusDeInit(busdev);
    return false;
}
#endif
