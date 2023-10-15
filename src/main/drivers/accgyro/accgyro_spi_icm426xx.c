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

#if defined(USE_GYRO_SPI_ICM42605) || defined(USE_GYRO_SPI_ICM42688P)

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

#define ICM426XX_RA_REG_BANK_SEL                    0x76
#define ICM426XX_BANK_SELECT0                       0x00
#define ICM426XX_BANK_SELECT1                       0x01
#define ICM426XX_BANK_SELECT2                       0x02
#define ICM426XX_BANK_SELECT3                       0x03
#define ICM426XX_BANK_SELECT4                       0x04

#define ICM426XX_RA_PWR_MGMT0                       0x4E  // User Bank 0
#define ICM426XX_PWR_MGMT0_ACCEL_MODE_LN            (3 << 0)
#define ICM426XX_PWR_MGMT0_GYRO_MODE_LN             (3 << 2)
#define ICM426XX_PWR_MGMT0_GYRO_ACCEL_MODE_OFF      ((0 << 0) | (0 << 2))
#define ICM426XX_PWR_MGMT0_TEMP_DISABLE_OFF         (0 << 5)
#define ICM426XX_PWR_MGMT0_TEMP_DISABLE_ON          (1 << 5)

#define ICM426XX_RA_GYRO_CONFIG0                    0x4F
#define ICM426XX_RA_ACCEL_CONFIG0                   0x50

// --- Registers for gyro and acc Anti-Alias Filter ---------
#define ICM426XX_RA_GYRO_CONFIG_STATIC3             0x0C  // User Bank 1
#define ICM426XX_RA_GYRO_CONFIG_STATIC4             0x0D  // User Bank 1
#define ICM426XX_RA_GYRO_CONFIG_STATIC5             0x0E  // User Bank 1
#define ICM426XX_RA_ACCEL_CONFIG_STATIC2            0x03  // User Bank 2
#define ICM426XX_RA_ACCEL_CONFIG_STATIC3            0x04  // User Bank 2
#define ICM426XX_RA_ACCEL_CONFIG_STATIC4            0x05  // User Bank 2
// --- Register & setting for gyro and acc UI Filter --------
#define ICM426XX_RA_GYRO_ACCEL_CONFIG0              0x52  // User Bank 0
#define ICM426XX_ACCEL_UI_FILT_BW_LOW_LATENCY       (15 << 4)
#define ICM426XX_GYRO_UI_FILT_BW_LOW_LATENCY        (15 << 0)
// ----------------------------------------------------------

#define ICM426XX_RA_GYRO_DATA_X1                    0x25  // User Bank 0
#define ICM426XX_RA_ACCEL_DATA_X1                   0x1F  // User Bank 0

#define ICM426XX_RA_INT_CONFIG                      0x14  // User Bank 0
#define ICM426XX_INT1_MODE_PULSED                   (0 << 2)
#define ICM426XX_INT1_MODE_LATCHED                  (1 << 2)
#define ICM426XX_INT1_DRIVE_CIRCUIT_OD              (0 << 1)
#define ICM426XX_INT1_DRIVE_CIRCUIT_PP              (1 << 1)
#define ICM426XX_INT1_POLARITY_ACTIVE_LOW           (0 << 0)
#define ICM426XX_INT1_POLARITY_ACTIVE_HIGH          (1 << 0)

#define ICM426XX_RA_INT_CONFIG0                     0x63  // User Bank 0
#define ICM426XX_UI_DRDY_INT_CLEAR_ON_SBR           ((0 << 5) || (0 << 4))
#define ICM426XX_UI_DRDY_INT_CLEAR_ON_SBR_DUPLICATE ((0 << 5) || (0 << 4)) // duplicate settings in datasheet, Rev 1.2.
#define ICM426XX_UI_DRDY_INT_CLEAR_ON_F1BR          ((1 << 5) || (0 << 4))
#define ICM426XX_UI_DRDY_INT_CLEAR_ON_SBR_AND_F1BR  ((1 << 5) || (1 << 4))

#define ICM426XX_RA_INT_CONFIG1                     0x64   // User Bank 0
#define ICM426XX_INT_ASYNC_RESET_BIT                4
#define ICM426XX_INT_TDEASSERT_DISABLE_BIT          5
#define ICM426XX_INT_TDEASSERT_ENABLED              (0 << ICM426XX_INT_TDEASSERT_DISABLE_BIT)
#define ICM426XX_INT_TDEASSERT_DISABLED             (1 << ICM426XX_INT_TDEASSERT_DISABLE_BIT)
#define ICM426XX_INT_TPULSE_DURATION_BIT            6
#define ICM426XX_INT_TPULSE_DURATION_100            (0 << ICM426XX_INT_TPULSE_DURATION_BIT)
#define ICM426XX_INT_TPULSE_DURATION_8              (1 << ICM426XX_INT_TPULSE_DURATION_BIT)

#define ICM426XX_RA_INT_SOURCE0                     0x65  // User Bank 0
#define ICM426XX_UI_DRDY_INT1_EN_DISABLED           (0 << 3)
#define ICM426XX_UI_DRDY_INT1_EN_ENABLED            (1 << 3)

typedef enum {
    ODR_CONFIG_8K = 0,
    ODR_CONFIG_4K,
    ODR_CONFIG_2K,
    ODR_CONFIG_1K,
    ODR_CONFIG_COUNT
} odrConfig_e;

typedef enum {
    AAF_CONFIG_258HZ = 0,
    AAF_CONFIG_536HZ,
    AAF_CONFIG_997HZ,
    AAF_CONFIG_1962HZ,
    AAF_CONFIG_COUNT
} aafConfig_e;

typedef struct aafConfig_s {
    uint8_t delt;
    uint16_t deltSqr;
    uint8_t bitshift;
} aafConfig_t;

// Possible output data rates (ODRs)
static uint8_t odrLUT[ODR_CONFIG_COUNT] = {  // see GYRO_ODR in section 5.6
    [ODR_CONFIG_8K] = 3,
    [ODR_CONFIG_4K] = 4,
    [ODR_CONFIG_2K] = 5,
    [ODR_CONFIG_1K] = 6,
};

// Possible gyro Anti-Alias Filter (AAF) cutoffs for ICM-42688P
static aafConfig_t aafLUT42688[AAF_CONFIG_COUNT] = {  // see table in section 5.3
    [AAF_CONFIG_258HZ]  = {  6,   36, 10 },
    [AAF_CONFIG_536HZ]  = { 12,  144,  8 },
    [AAF_CONFIG_997HZ]  = { 21,  440,  6 },
    [AAF_CONFIG_1962HZ] = { 37, 1376,  4 },
};

// Possible gyro Anti-Alias Filter (AAF) cutoffs for ICM-42688P
// actual cutoff differs slightly from those of the 42688P
static aafConfig_t aafLUT42605[AAF_CONFIG_COUNT] = {  // see table in section 5.3
    [AAF_CONFIG_258HZ]  = { 21,  440,  6 }, // actually 249 Hz
    [AAF_CONFIG_536HZ]  = { 39, 1536,  4 }, // actually 524 Hz
    [AAF_CONFIG_997HZ]  = { 63, 3968,  3 }, // actually 995 Hz
    [AAF_CONFIG_1962HZ] = { 63, 3968,  3 }, // 995 Hz is the max cutoff on the 42605
};

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

bool icm426xxAccRead(accDev_t *acc)
{
    uint8_t data[6];

    const bool ack = busReadRegisterBuffer(&acc->bus, ICM426XX_RA_ACCEL_DATA_X1, data, 6);
    if (!ack) {
        return false;
    }

    acc->ADCRaw[X] = (int16_t)((data[0] << 8) | data[1]);
    acc->ADCRaw[Y] = (int16_t)((data[2] << 8) | data[3]);
    acc->ADCRaw[Z] = (int16_t)((data[4] << 8) | data[5]);

    return true;
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
    acc->readFn = icm426xxAccRead;
    return true;
}

static aafConfig_t getGyroAafConfig(const mpuSensor_e, const aafConfig_e);

static void turnGyroAccOff(const gyroDev_t *gyro)
{
    spiBusWriteRegister(&gyro->bus, ICM426XX_RA_PWR_MGMT0, ICM426XX_PWR_MGMT0_GYRO_ACCEL_MODE_OFF);
}

// Turn on gyro and acc on in Low Noise mode
static void turnGyroAccOn(const gyroDev_t *gyro)
{
    spiBusWriteRegister(&gyro->bus, ICM426XX_RA_PWR_MGMT0, ICM426XX_PWR_MGMT0_TEMP_DISABLE_OFF | ICM426XX_PWR_MGMT0_ACCEL_MODE_LN | ICM426XX_PWR_MGMT0_GYRO_MODE_LN);
    delay(1);
}

static void setUserBank(const gyroDev_t *gyro, const uint8_t user_bank)
{
    spiBusWriteRegister(&gyro->bus, ICM426XX_RA_REG_BANK_SEL, user_bank & 7);
}

void icm426xxGyroInit(gyroDev_t *gyro)
{
    mpuGyroInit(gyro);

    spiSetDivisor(gyro->bus.busdev_u.spi.instance, spiCalculateDivider(ICM426XX_MAX_SPI_CLK_HZ));

    gyro->accDataReg = ICM426XX_RA_ACCEL_DATA_X1;
    gyro->gyroDataReg = ICM426XX_RA_GYRO_DATA_X1;

    // Turn off ACC and GYRO so they can be configured
    // See section 12.9 in ICM-42688-P datasheet v1.7
    setUserBank(gyro, ICM426XX_BANK_SELECT0);
    turnGyroAccOff(gyro);

    // Configure gyro Anti-Alias Filter (see section 5.3 "ANTI-ALIAS FILTER")
    const mpuSensor_e gyroModel = gyro->mpuDetectionResult.sensor;
    aafConfig_t aafConfig = getGyroAafConfig(gyroModel, gyroConfig()->gyro_hardware_lpf);
    setUserBank(gyro, ICM426XX_BANK_SELECT1);
    spiBusWriteRegister(&gyro->bus, ICM426XX_RA_GYRO_CONFIG_STATIC3, aafConfig.delt);
    spiBusWriteRegister(&gyro->bus, ICM426XX_RA_GYRO_CONFIG_STATIC4, aafConfig.deltSqr & 0xFF);
    spiBusWriteRegister(&gyro->bus, ICM426XX_RA_GYRO_CONFIG_STATIC5, (aafConfig.deltSqr >> 8) | (aafConfig.bitshift << 4));

    // Configure acc Anti-Alias Filter for 1kHz sample rate (see tasks.c)
    aafConfig = getGyroAafConfig(gyroModel, AAF_CONFIG_258HZ);
    setUserBank(gyro, ICM426XX_BANK_SELECT2);
    spiBusWriteRegister(&gyro->bus, ICM426XX_RA_ACCEL_CONFIG_STATIC2, aafConfig.delt << 1);
    spiBusWriteRegister(&gyro->bus, ICM426XX_RA_ACCEL_CONFIG_STATIC3, aafConfig.deltSqr & 0xFF);
    spiBusWriteRegister(&gyro->bus, ICM426XX_RA_ACCEL_CONFIG_STATIC4, (aafConfig.deltSqr >> 8) | (aafConfig.bitshift << 4));

    // Configure gyro and acc UI Filters
    setUserBank(gyro, ICM426XX_BANK_SELECT0);
    spiBusWriteRegister(&gyro->bus, ICM426XX_RA_GYRO_ACCEL_CONFIG0, ICM426XX_ACCEL_UI_FILT_BW_LOW_LATENCY | ICM426XX_GYRO_UI_FILT_BW_LOW_LATENCY);

    // Configure interrupt pin
    spiBusWriteRegister(&gyro->bus, ICM426XX_RA_INT_CONFIG, ICM426XX_INT1_MODE_PULSED | ICM426XX_INT1_DRIVE_CIRCUIT_PP | ICM426XX_INT1_POLARITY_ACTIVE_HIGH);
    spiBusWriteRegister(&gyro->bus, ICM426XX_RA_INT_CONFIG0, ICM426XX_UI_DRDY_INT_CLEAR_ON_SBR);

    spiBusWriteRegister(&gyro->bus, ICM426XX_RA_INT_SOURCE0, ICM426XX_UI_DRDY_INT1_EN_ENABLED);

    uint8_t intConfig1Value = spiReadRegMsk(&gyro->bus, ICM426XX_RA_INT_CONFIG1);
    // Datasheet says: "User should change setting to 0 from default setting of 1, for proper INT1 and INT2 pin operation"
    intConfig1Value &= ~(1 << ICM426XX_INT_ASYNC_RESET_BIT);
    intConfig1Value |= (ICM426XX_INT_TPULSE_DURATION_8 | ICM426XX_INT_TDEASSERT_DISABLED);

    spiBusWriteRegister(&gyro->bus, ICM426XX_RA_INT_CONFIG1, intConfig1Value);

    // Turn on gyro and acc on again so ODR and FSR can be configured
    turnGyroAccOn(gyro);

    // Get desired output data rate
    uint8_t odrConfig;
    const unsigned decim = llog2(gyro->mpuDividerDrops + 1);
    if (gyro->gyroRateKHz && decim < ODR_CONFIG_COUNT) {
        odrConfig = odrLUT[decim];
    } else {
        odrConfig = odrLUT[ODR_CONFIG_1K];
        gyro->gyroRateKHz = GYRO_RATE_1_kHz;
    }

    STATIC_ASSERT(INV_FSR_2000DPS == 3, INV_FSR_2000DPS_must_be_3_to_generate_correct_value);
    spiBusWriteRegister(&gyro->bus, ICM426XX_RA_GYRO_CONFIG0, (3 - INV_FSR_2000DPS) << 5 | (odrConfig & 0x0F));
    delay(15);

    STATIC_ASSERT(INV_FSR_16G == 3, INV_FSR_16G_must_be_3_to_generate_correct_value);
    spiBusWriteRegister(&gyro->bus, ICM426XX_RA_ACCEL_CONFIG0, (3 - INV_FSR_16G) << 5 | (odrConfig & 0x0F));
    delay(15);
}

// MIGHT NOT work on STM32F7
#define STATIC_DMA_DATA_AUTO static
//FAST_CODE bool mpuGyroReadSPI(gyroDev_t *gyro)
bool icm426xxGyroReadSPI(gyroDev_t *gyro)
{
    //static const uint8_t dataToSend[7] = {MPU_RA_GYRO_XOUT_H | 0x80, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    STATIC_DMA_DATA_AUTO uint8_t dataToSend[7] = {ICM426XX_RA_GYRO_DATA_X1 | 0x80, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    //uint8_t data[7];
    STATIC_DMA_DATA_AUTO uint8_t data[7];

    const bool ack = spiBusTransfer(&gyro->bus, dataToSend, data, 7);
    if (!ack) {
        return false;
    }

    gyro->gyroADCRaw[X] = (int16_t)((data[1] << 8) | data[2]);
    gyro->gyroADCRaw[Y] = (int16_t)((data[3] << 8) | data[4]);
    gyro->gyroADCRaw[Z] = (int16_t)((data[5] << 8) | data[6]);

    return true;
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
    gyro->readFn = icm426xxGyroReadSPI;

    gyro->scale = (2000.0f / (1 << 15));   // 16.384 dps/lsb scalefactor for 2000dps sensors

    return true;
}

static aafConfig_t getGyroAafConfig(const mpuSensor_e gyroModel, const aafConfig_e config)
{
    switch (gyroModel){
    case ICM_42605_SPI:
        switch (config) {
        case GYRO_HARDWARE_LPF_NORMAL:
            return aafLUT42605[AAF_CONFIG_258HZ];
        case GYRO_HARDWARE_LPF_OPTION_1:
            return aafLUT42605[AAF_CONFIG_536HZ];
        case GYRO_HARDWARE_LPF_OPTION_2:
            return aafLUT42605[AAF_CONFIG_997HZ];
        default:
            return aafLUT42605[AAF_CONFIG_258HZ];
        }

    case ICM_42688P_SPI:
    default:
        switch (config) {
        case GYRO_HARDWARE_LPF_NORMAL:
            return aafLUT42688[AAF_CONFIG_258HZ];
        case GYRO_HARDWARE_LPF_OPTION_1:
            return aafLUT42688[AAF_CONFIG_536HZ];
        case GYRO_HARDWARE_LPF_OPTION_2:
            return aafLUT42688[AAF_CONFIG_997HZ];
#ifdef USE_GYRO_DLPF_EXPERIMENTAL
        case GYRO_HARDWARE_LPF_EXPERIMENTAL:
            return aafLUT42688[AAF_CONFIG_1962HZ];
#endif
        default:
            return aafLUT42688[AAF_CONFIG_258HZ];
        }
    }
}

#endif // USE_GYRO_SPI_ICM42605 || USE_GYRO_SPI_ICM42688P
