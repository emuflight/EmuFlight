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
#include "drivers/accgyro/accgyro_spi_icm20649.h"
#include "drivers/bus_spi.h"
#include "drivers/exti.h"
#include "drivers/io.h"
#include "drivers/sensor.h"
#include "drivers/time.h"


static void icm20649SpiInit(const busDevice_t *bus) {
    static bool hardwareInitialised = false;
    if (hardwareInitialised) {
        return;
    }
#ifndef USE_DUAL_GYRO
    IOInit(bus->busdev_u.spi.csnPin, OWNER_MPU_CS, 0);
    IOConfigGPIO(bus->busdev_u.spi.csnPin, SPI_IO_CS_CFG);
    IOHi(bus->busdev_u.spi.csnPin);
#endif
    // all registers can be read/written at full speed (7MHz +-10%)
    // TODO verify that this works at 9MHz and 10MHz on non F7
    spiSetDivisor(bus->busdev_u.spi.instance, SPI_CLOCK_STANDARD);
    hardwareInitialised = true;
}

uint8_t icm20649SpiDetect(const busDevice_t *bus) {
    icm20649SpiInit(bus);
    spiSetDivisor(bus->busdev_u.spi.instance, SPI_CLOCK_STANDARD);
    spiBusWriteRegister(bus, ICM20649_RA_REG_BANK_SEL, 0 << 4); // select bank 0 just to be safe
    delay(15);
    spiBusWriteRegister(bus, ICM20649_RA_PWR_MGMT_1, ICM20649_BIT_RESET);
    uint8_t icmDetected = MPU_NONE;
    uint8_t attemptsRemaining = 20;
    do {
        delay(150);
        const uint8_t whoAmI = spiBusReadRegister(bus, ICM20649_RA_WHO_AM_I);
        if (whoAmI == ICM20649_WHO_AM_I_CONST) {
            icmDetected = ICM_20649_SPI;
        } else {
            icmDetected = MPU_NONE;
        }
        if (icmDetected != MPU_NONE) {
            break;
        }
        if (!attemptsRemaining) {
            return MPU_NONE;
        }
    } while (attemptsRemaining--);
    return icmDetected;
}

void icm20649AccInit(accDev_t *acc) {
    // 2,048 LSB/g 16g
    // 1,024 LSB/g 30g
    acc->acc_1G = acc->acc_high_fsr ? 1024 : 2048;
    spiSetDivisor(acc->bus.busdev_u.spi.instance, SPI_CLOCK_STANDARD);
    spiBusWriteRegister(&acc->bus, ICM20649_RA_REG_BANK_SEL, 2 << 4); // config in bank 2
    delay(15);
    const uint8_t acc_fsr = acc->acc_high_fsr ? ICM20649_FSR_30G : ICM20649_FSR_16G;
    spiBusWriteRegister(&acc->bus, ICM20649_RA_ACCEL_CONFIG, acc_fsr << 1);
    delay(15);
    spiBusWriteRegister(&acc->bus, ICM20649_RA_REG_BANK_SEL, 0 << 4); // back to bank 0
    delay(15);
}

bool icm20649SpiAccDetect(accDev_t *acc) {
    if (acc->mpuDetectionResult.sensor != ICM_20649_SPI) {
        return false;
    }
    acc->initFn = icm20649AccInit;
    acc->readFn = icm20649AccRead;
    return true;
}


void icm20649GyroInit(gyroDev_t *gyro) {
    mpuGyroInit(gyro);
    spiSetDivisor(gyro->bus.busdev_u.spi.instance, SPI_CLOCK_STANDARD); // ensure proper speed
    spiBusWriteRegister(&gyro->bus, ICM20649_RA_REG_BANK_SEL, 0 << 4); // select bank 0 just to be safe
    delay(15);
    spiBusWriteRegister(&gyro->bus, ICM20649_RA_PWR_MGMT_1, ICM20649_BIT_RESET);
    delay(100);
    spiBusWriteRegister(&gyro->bus, ICM20649_RA_PWR_MGMT_1, INV_CLK_PLL);
    delay(15);
    spiBusWriteRegister(&gyro->bus, ICM20649_RA_REG_BANK_SEL, 2 << 4); // config in bank 2
    delay(15);
    const uint8_t gyro_fsr = gyro->gyro_high_fsr ? ICM20649_FSR_4000DPS : ICM20649_FSR_2000DPS;
    // If hardware_lpf is either GYRO_HARDWARE_LPF_NORMAL or GYRO_HARDWARE_LPF_EXPERIMENTAL then the
    // gyro is running in 9KHz sample mode and GYRO_FCHOICE should be 0, otherwise we're in 1.1KHz sample
    // mode and GYRO_FCHOICE = 1.  When in 1.1KHz mode select the 196.6Hz DLPF (GYRO_DLPFCFG = 0)
    // Unfortunately we can't configure any difference in DLPF based on NORMAL vs. EXPERIMENTAL because
    // the ICM20649 only has a single 9KHz DLPF cutoff.
    uint8_t raGyroConfigData = gyro->gyroRateKHz > GYRO_RATE_1100_Hz ? 0 : 1; // deactivate GYRO_FCHOICE for sample rates over 1kHz (opposite of other invensense chips)
    raGyroConfigData |= gyro_fsr << 1;
    spiBusWriteRegister(&gyro->bus, ICM20649_RA_GYRO_CONFIG_1, raGyroConfigData);
    delay(15);
    spiBusWriteRegister(&gyro->bus, ICM20649_RA_GYRO_SMPLRT_DIV, gyro->mpuDividerDrops); // Get Divider Drops
    delay(100);
    // Data ready interrupt configuration
    // back to bank 0
    spiBusWriteRegister(&gyro->bus, ICM20649_RA_REG_BANK_SEL, 0 << 4);
    delay(15);
    spiBusWriteRegister(&gyro->bus, ICM20649_RA_INT_PIN_CFG, 0x11);  // INT_ANYRD_2CLEAR, BYPASS_EN
    delay(15);
#ifdef USE_MPU_DATA_READY_SIGNAL
    spiBusWriteRegister(&gyro->bus, ICM20649_RA_INT_ENABLE_1, 0x01);
#endif
}

bool icm20649SpiGyroDetect(gyroDev_t *gyro) {
    if (gyro->mpuDetectionResult.sensor != ICM_20649_SPI)
        return false;
    gyro->initFn = icm20649GyroInit;
    gyro->readFn = icm20649GyroReadSPI;
    // 16.4 dps/lsb 2kDps
    //  8.2 dps/lsb 4kDps
    gyro->scale = 1.0f / (gyro->gyro_high_fsr ? 8.2f : 16.4f);
    return true;
}

bool icm20649GyroReadSPI(gyroDev_t *gyro) {
    static const uint8_t dataToSend[7] = {ICM20649_RA_GYRO_XOUT_H | 0x80, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    uint8_t data[7];
    const bool ack = spiBusTransfer(&gyro->bus, dataToSend, data, 7);
    if (!ack) {
        return false;
    }
    gyro->gyroADCRaw[X] = (int16_t)((data[1] << 8) | data[2]);
    gyro->gyroADCRaw[Y] = (int16_t)((data[3] << 8) | data[4]);
    gyro->gyroADCRaw[Z] = (int16_t)((data[5] << 8) | data[6]);
    return true;
}

bool icm20649AccRead(accDev_t *acc) {
    uint8_t data[6];
    const bool ack = spiBusReadRegisterBuffer(&acc->bus, ICM20649_RA_ACCEL_XOUT_H, data, 6);
    if (!ack) {
        return false;
    }
    acc->ADCRaw[X] = (int16_t)((data[0] << 8) | data[1]);
    acc->ADCRaw[Y] = (int16_t)((data[2] << 8) | data[3]);
    acc->ADCRaw[Z] = (int16_t)((data[4] << 8) | data[5]);
    return true;
}
