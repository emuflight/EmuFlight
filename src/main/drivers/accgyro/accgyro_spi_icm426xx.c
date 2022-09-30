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
#include "drivers/accgyro/accgyro_spi_icm426xx.h"
#include "drivers/bus_spi.h"
#include "drivers/exti.h"
#include "drivers/io.h"
#include "drivers/sensor.h"
#include "drivers/time.h"


// 24 MHz max SPI frequency
#define ICM426XX_MAX_SPI_CLK_HZ 24000000

#define ICM426XX_RA_PWR_MGMT0                       0x4E

#define ICM426XX_PWR_MGMT0_ACCEL_MODE_LN            (3 << 0)
#define ICM426XX_PWR_MGMT0_GYRO_MODE_LN             (3 << 2)
#define ICM426XX_PWR_MGMT0_TEMP_DISABLE_OFF         (0 << 5)
#define ICM426XX_PWR_MGMT0_TEMP_DISABLE_ON          (1 << 5)

#define ICM426XX_RA_GYRO_CONFIG0                    0x4F
#define ICM426XX_RA_ACCEL_CONFIG0                   0x50

#define ICM426XX_RA_GYRO_ACCEL_CONFIG0              0x52

#define ICM426XX_ACCEL_UI_FILT_BW_LOW_LATENCY       (14 << 4)
#define ICM426XX_GYRO_UI_FILT_BW_LOW_LATENCY        (14 << 0)

#define ICM426XX_RA_GYRO_DATA_X1                    0x25
#define ICM426XX_RA_ACCEL_DATA_X1                   0x1F

#define ICM426XX_RA_INT_CONFIG                      0x14
#define ICM426XX_INT1_MODE_PULSED                   (0 << 2)
#define ICM426XX_INT1_MODE_LATCHED                  (1 << 2)
#define ICM426XX_INT1_DRIVE_CIRCUIT_OD              (0 << 1)
#define ICM426XX_INT1_DRIVE_CIRCUIT_PP              (1 << 1)
#define ICM426XX_INT1_POLARITY_ACTIVE_LOW           (0 << 0)
#define ICM426XX_INT1_POLARITY_ACTIVE_HIGH          (1 << 0)

#define ICM426XX_RA_INT_CONFIG0                     0x63
#define ICM426XX_UI_DRDY_INT_CLEAR_ON_SBR           ((0 << 5) || (0 << 4))
#define ICM426XX_UI_DRDY_INT_CLEAR_ON_SBR_DUPLICATE ((0 << 5) || (0 << 4)) // duplicate settings in datasheet, Rev 1.2.
#define ICM426XX_UI_DRDY_INT_CLEAR_ON_F1BR          ((1 << 5) || (0 << 4))
#define ICM426XX_UI_DRDY_INT_CLEAR_ON_SBR_AND_F1BR  ((1 << 5) || (1 << 4))

#define ICM426XX_RA_INT_CONFIG1                     0x64
#define ICM426XX_INT_ASYNC_RESET_BIT                4
#define ICM426XX_INT_TDEASSERT_DISABLE_BIT          5
#define ICM426XX_INT_TDEASSERT_ENABLED              (0 << ICM426XX_INT_TDEASSERT_DISABLE_BIT)
#define ICM426XX_INT_TDEASSERT_DISABLED             (1 << ICM426XX_INT_TDEASSERT_DISABLE_BIT)
#define ICM426XX_INT_TPULSE_DURATION_BIT            6
#define ICM426XX_INT_TPULSE_DURATION_100            (0 << ICM426XX_INT_TPULSE_DURATION_BIT)
#define ICM426XX_INT_TPULSE_DURATION_8              (1 << ICM426XX_INT_TPULSE_DURATION_BIT)


#define ICM426XX_RA_INT_SOURCE0                     0x65
#define ICM426XX_UI_DRDY_INT1_EN_DISABLED           (0 << 3)
#define ICM426XX_UI_DRDY_INT1_EN_ENABLED            (1 << 3)


static void icm426xxSpiInit(const busDevice_t *bus) {
    static bool hardwareInitialised = false;
    if (hardwareInitialised) {
        return;
    }
#ifndef USE_DUAL_GYRO
    IOInit(bus->busdev_u.spi.csnPin, OWNER_MPU_CS, 0);
    IOConfigGPIO(bus->busdev_u.spi.csnPin, SPI_IO_CS_CFG);
    IOHi(bus->busdev_u.spi.csnPin);
#endif
    spiSetDivisor(bus->busdev_u.spi.instance, SPI_CLOCK_STANDARD);
    hardwareInitialised = true;
}

uint8_t icm426xxSpiDetect(const busDevice_t *bus) {
    icm426xxSpiInit(bus);
    spiSetDivisor(bus->busdev_u.spi.instance, SPI_CLOCK_INITIALIZATION); //low speed
    spiBusWriteRegister(bus, MPU_RA_PWR_MGMT_1, ICM426xx_BIT_RESET);
    uint8_t icmDetected = MPU_NONE;
    uint8_t attemptsRemaining = 20;
    do {
        delay(150);
        const uint8_t whoAmI = spiBusReadRegister(bus, MPU_RA_WHO_AM_I);
        switch (whoAmI) {
        case ICM42605_WHO_AM_I_CONST:
            icmDetected = ICM_42605_SPI;
            break;
        case ICM42688P_WHO_AM_I_CONST:
            icmDetected = ICM_42688P_SPI;
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
    spiSetDivisor(bus->busdev_u.spi.instance, SPI_CLOCK_STANDARD);
    return icmDetected;
}

void icm426xxAccInit(accDev_t *acc) {
    acc->acc_1G = 512 * 4;
}

bool icm426xxSpiAccDetect(accDev_t *acc) {
    switch (acc->mpuDetectionResult.sensor) {
    case ICM_42605_SPI:
    case ICM_42688P_SPI:
        break;
    default:
        return false;
    }
    acc->initFn = icm426xxAccInit;
    acc->readFn = mpuAccRead;
    return true;
}

void icm426xxGyroInit(gyroDev_t *gyro) {
    mpuGyroInit(gyro);
    spiSetDivisor(gyro->bus.busdev_u.spi.instance, SPI_CLOCK_INITIALIZATION);
    spiBusWriteRegister(&gyro->bus, MPU_RA_PWR_MGMT_1, ICM426xx_BIT_RESET);
    delay(100);
    spiBusWriteRegister(&gyro->bus, MPU_RA_SIGNAL_PATH_RESET, 0x03);
    delay(100);
//    spiBusWriteRegister(&gyro->bus, MPU_RA_PWR_MGMT_1, 0);
//    delay(100);
    spiBusWriteRegister(&gyro->bus, MPU_RA_PWR_MGMT_1, INV_CLK_PLL);
    delay(15);
    spiBusWriteRegister(&gyro->bus, MPU_RA_GYRO_CONFIG,  (3 - INV_FSR_2000DPS) << 5  | mpuGyroFCHOICE(gyro));
    delay(15);
    spiBusWriteRegister(&gyro->bus, MPU_RA_ACCEL_CONFIG, (3 - INV_FSR_16G) << 5);
    delay(15);
    spiBusWriteRegister(&gyro->bus, MPU_RA_CONFIG, mpuGyroDLPF(gyro));
    delay(15);
    spiBusWriteRegister(&gyro->bus, MPU_RA_SMPLRT_DIV, gyro->mpuDividerDrops); // Get Divider Drops
    delay(100);
    // Data ready interrupt configuration
//    spiBusWriteRegister(&gyro->bus, MPU_RA_INT_PIN_CFG, 0 << 7 | 0 << 6 | 0 << 5 | 1 << 4 | 0 << 3 | 0 << 2 | 0 << 1 | 0 << 0);  // INT_ANYRD_2CLEAR, BYPASS_EN
    spiBusWriteRegister(&gyro->bus, MPU_RA_INT_PIN_CFG, 0x10);  // INT_ANYRD_2CLEAR, BYPASS_EN
    delay(15);
#ifdef USE_MPU_DATA_READY_SIGNAL
    spiBusWriteRegister(&gyro->bus, MPU_RA_INT_ENABLE, 0x01); // RAW_RDY_EN interrupt enable
#endif
    spiSetDivisor(gyro->bus.busdev_u.spi.instance, SPI_CLOCK_STANDARD);
}

bool icm426xxSpiGyroDetect(gyroDev_t *gyro) {
    switch (gyro->mpuDetectionResult.sensor) {
    case ICM_42605_SPI:
    case ICM_42688P_SPI:
        break;
    default:
        return false;
    }
    gyro->initFn = icm426xxGyroInit;
    gyro->readFn = mpuGyroReadSPI;
    // 16.4 dps/lsb scalefactor
    gyro->scale = 1.0f / 16.4f;
    return true;
}
