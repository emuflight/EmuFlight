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

#include "platform.h"

#include "common/axis.h"
#include "common/maths.h"

#include "drivers/accgyro/accgyro.h"
#include "drivers/accgyro/accgyro_mpu.h"
#include "drivers/accgyro/accgyro_spi_icm20689.h"
#include "drivers/bus_spi.h"
#include "drivers/exti.h"
#include "drivers/io.h"
#include "drivers/sensor.h"
#include "drivers/time.h"

// Register 0x37 - INT_PIN_CFG
#define ICM20689_INT_ANYRD_2CLEAR   0x10

// Register 0x68 - SIGNAL_PATH_RESET
#define ICM20689_ACCEL_RST          0x02
#define ICM20689_TEMP_RST           0x01

// Register 0x6a - USER_CTRL
#define ICM20689_I2C_IF_DIS         0x10

// Register 0x6b - PWR_MGMT_1
#define ICM20689_BIT_RESET          0x80

/* Allow CLKSEL setting time to settle when PLL is selected.
 * Testing has shown that 60us is required, so double to allow a margin.
 */
#define ICM20689_CLKSEL_SETTLE_US   120

/* MPU-6000 datasheet (section 4.28) suggests 100ms after a reset */
#define ICM20689_RESET_DELAY_MS     100

/* MPU-6000 datasheet (section 4.28) suggests 100ms after a path reset */
#define ICM20689_PATH_RESET_DELAY_MS 100

static void icm20689SpiInit(const extDevice_t *dev) {
    static bool hardwareInitialised = false;
    if (hardwareInitialised) {
        return;
    }
#ifndef USE_DUAL_GYRO
    IOInit(dev->busType_u.spi.csnPin, OWNER_MPU_CS, 0);
    IOConfigGPIO(dev->busType_u.spi.csnPin, SPI_IO_CS_CFG);
    IOHi(dev->busType_u.spi.csnPin);
#endif
    spiSetDivisor(dev->bus->busType_u.spi.instance, SPI_CLOCK_STANDARD);
    hardwareInitialised = true;
}

uint8_t icm20689SpiDetect(const extDevice_t *dev) {
    icm20689SpiInit(dev);
    spiSetDivisor(dev->bus->busType_u.spi.instance, SPI_CLOCK_INITIALIZATION);

    // Note that the following reset is being done repeatedly by each MPU6000
    // compatible device being probed
    spiWriteReg(dev, MPU_RA_PWR_MGMT_1, ICM20689_BIT_RESET);

    uint8_t icmDetected = MPU_NONE;
    uint8_t attemptsRemaining = 20;
    do {
        delay(ICM20689_RESET_DELAY_MS);
        const uint8_t whoAmI = spiReadRegMsk(dev, MPU_RA_WHO_AM_I);
        switch (whoAmI) {
        case ICM20601_WHO_AM_I_CONST:
            icmDetected = ICM_20601_SPI;
            break;
        case ICM20602_WHO_AM_I_CONST:
            icmDetected = ICM_20602_SPI;
            break;
        case ICM20608G_WHO_AM_I_CONST:
            icmDetected = ICM_20608_SPI;
            break;
        case ICM20689_WHO_AM_I_CONST:
            icmDetected = ICM_20689_SPI;
            break;
        default:
            icmDetected = MPU_NONE;
            break;
        }
        if (icmDetected != MPU_NONE) {
            break;
        }
        if (!attemptsRemaining) {
            return MPU_NONE;
        }
    } while (attemptsRemaining--);

    // We now know the device is recognised so it's safe to perform device-specific register accesses
    spiSetDivisor(dev->bus->busType_u.spi.instance, SPI_CLOCK_STANDARD);

    // Disable Primary I2C Interface
    spiWriteReg(dev, MPU_RA_USER_CTRL, ICM20689_I2C_IF_DIS);

    // Reset the device signal paths
    spiWriteReg(dev, MPU_RA_SIGNAL_PATH_RESET, ICM20689_ACCEL_RST | ICM20689_TEMP_RST);
    delay(ICM20689_PATH_RESET_DELAY_MS);

    return icmDetected;
}

void icm20689AccInit(accDev_t *acc) {
    acc->acc_1G = 512 * 4;
}

bool icm20689SpiAccDetect(accDev_t *acc) {
    switch (acc->mpuDetectionResult.sensor) {
    case ICM_20602_SPI:
    case ICM_20689_SPI:
        break;
    default:
        return false;
    }
    acc->initFn = icm20689AccInit;
    acc->readFn = mpuAccRead;
    return true;
}

void icm20689GyroInit(gyroDev_t *gyro) {
    mpuGyroInit(gyro);

    // Device was already reset during detection so proceed with configuration
    spiWriteReg(&gyro->dev, MPU_RA_PWR_MGMT_1, INV_CLK_PLL);
    delayMicroseconds(ICM20689_CLKSEL_SETTLE_US);
    spiWriteReg(&gyro->dev, MPU_RA_GYRO_CONFIG, INV_FSR_2000DPS << 3 | mpuGyroFCHOICE(gyro));
    spiWriteReg(&gyro->dev, MPU_RA_ACCEL_CONFIG, INV_FSR_16G << 3);
    spiWriteReg(&gyro->dev, MPU_RA_CONFIG, mpuGyroDLPF(gyro));
    spiWriteReg(&gyro->dev, MPU_RA_SMPLRT_DIV, gyro->mpuDividerDrops);

    // Data ready interrupt configuration
    spiWriteReg(&gyro->dev, MPU_RA_INT_PIN_CFG, ICM20689_INT_ANYRD_2CLEAR);

#ifdef USE_MPU_DATA_READY_SIGNAL
    spiWriteReg(&gyro->dev, MPU_RA_INT_ENABLE, MPU_RF_DATA_RDY_EN);
#endif

    spiSetDivisor(gyro->dev.bus->busType_u.spi.instance, SPI_CLOCK_STANDARD);
}

bool icm20689SpiGyroDetect(gyroDev_t *gyro) {
    switch (gyro->mpuDetectionResult.sensor) {
    case ICM_20601_SPI:
    case ICM_20602_SPI:
    case ICM_20608_SPI:
    case ICM_20689_SPI:
        break;
    default:
        return false;
    }
    gyro->initFn = icm20689GyroInit;
    gyro->readFn = mpuGyroReadSPI;
    // 16.4 dps/lsb scalefactor
    gyro->scale = 1.0f / 16.4f;
    return true;
}
