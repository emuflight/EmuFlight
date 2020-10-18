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

#if defined(USE_BARO) && (defined(USE_BARO_MS5611) || defined(USE_BARO_SPI_MS5611))

#include "build/build_config.h"

#include "barometer.h"
#include "barometer_ms5611.h"

#include "drivers/bus_i2c.h"
#include "drivers/bus_i2c_busdev.h"
#include "drivers/bus_spi.h"
#include "drivers/io.h"
#include "drivers/time.h"

#define CMD_RESET               0x1E // ADC reset command
#define CMD_ADC_READ            0x00 // ADC read command
#define CMD_ADC_CONV            0x40 // ADC conversion command
#define CMD_ADC_D1              0x00 // ADC D1 conversion
#define CMD_ADC_D2              0x10 // ADC D2 conversion
#define CMD_ADC_256             0x00 // ADC OSR=256
#define CMD_ADC_512             0x02 // ADC OSR=512
#define CMD_ADC_1024            0x04 // ADC OSR=1024
#define CMD_ADC_2048            0x06 // ADC OSR=2048
#define CMD_ADC_4096            0x08 // ADC OSR=4096
#define CMD_PROM_RD             0xA0 // Prom read command
#define PROM_NB                 8

static void ms5611_reset(busDevice_t *busdev);
static uint16_t ms5611_prom(busDevice_t *busdev, int8_t coef_num);
STATIC_UNIT_TESTED int8_t ms5611_crc(uint16_t *prom);
static uint32_t ms5611_read_adc(busDevice_t *busdev);
static void ms5611_start_ut(baroDev_t *baro);
static void ms5611_get_ut(baroDev_t *baro);
static void ms5611_start_up(baroDev_t *baro);
static void ms5611_get_up(baroDev_t *baro);
STATIC_UNIT_TESTED void ms5611_calculate(int32_t *pressure, int32_t *temperature);

STATIC_UNIT_TESTED uint32_t ms5611_ut;  // static result of temperature measurement
STATIC_UNIT_TESTED uint32_t ms5611_up;  // static result of pressure measurement
STATIC_UNIT_TESTED uint16_t ms5611_c[PROM_NB];  // on-chip ROM
static uint8_t ms5611_osr = CMD_ADC_4096;

void ms5611BusInit(busDevice_t *busdev) {
#ifdef USE_BARO_SPI_MS5611
    if (busdev->bustype == BUSTYPE_SPI) {
        IOHi(busdev->busdev_u.spi.csnPin); // Disable
        IOInit(busdev->busdev_u.spi.csnPin, OWNER_BARO_CS, 0);
        IOConfigGPIO(busdev->busdev_u.spi.csnPin, IOCFG_OUT_PP);
        spiSetDivisor(busdev->busdev_u.spi.instance, SPI_CLOCK_STANDARD); // XXX
    }
#else
    UNUSED(busdev);
#endif
}

void ms5611BusDeinit(busDevice_t *busdev) {
#ifdef USE_BARO_SPI_MS5611
    if (busdev->bustype == BUSTYPE_SPI) {
        spiPreinitCsByIO(busdev->busdev_u.spi.csnPin);
    }
#else
    UNUSED(busdev);
#endif
}

bool ms5611Detect(baroDev_t *baro) {
    uint8_t sig;
    int i;
    bool defaultAddressApplied = false;
    delay(10); // No idea how long the chip takes to power-up, but let's make it 10ms
    busDevice_t *busdev = &baro->busdev;
    ms5611BusInit(busdev);
    if ((busdev->bustype == BUSTYPE_I2C) && (busdev->busdev_u.i2c.address == 0)) {
        // Default address for MS5611
        busdev->busdev_u.i2c.address = MS5611_I2C_ADDR;
        defaultAddressApplied = true;
    }
    if (!busReadRegisterBuffer(busdev, CMD_PROM_RD, &sig, 1) || sig == 0xFF) {
        goto fail;
    }
    ms5611_reset(busdev);
    // read all coefficients
    for (i = 0; i < PROM_NB; i++)
        ms5611_c[i] = ms5611_prom(busdev, i);
    // check crc, bail out if wrong - we are probably talking to BMP085 w/o XCLR line!
    if (ms5611_crc(ms5611_c) != 0) {
        goto fail;
    }
    // TODO prom + CRC
    baro->ut_delay = 10000;
    baro->up_delay = 10000;
    baro->start_ut = ms5611_start_ut;
    baro->get_ut = ms5611_get_ut;
    baro->start_up = ms5611_start_up;
    baro->get_up = ms5611_get_up;
    baro->calculate = ms5611_calculate;
    return true;
fail:
    ;
    ms5611BusDeinit(busdev);
    if (defaultAddressApplied) {
        busdev->busdev_u.i2c.address = 0;
    }
    return false;
}

static void ms5611_reset(busDevice_t *busdev) {
    busWriteRegister(busdev, CMD_RESET, 1);
    delayMicroseconds(2800);
}

static uint16_t ms5611_prom(busDevice_t *busdev, int8_t coef_num) {
    uint8_t rxbuf[2] = { 0, 0 };
    busReadRegisterBuffer(busdev, CMD_PROM_RD + coef_num * 2, rxbuf, 2); // send PROM READ command
    return rxbuf[0] << 8 | rxbuf[1];
}

STATIC_UNIT_TESTED int8_t ms5611_crc(uint16_t *prom) {
    int32_t i, j;
    uint32_t res = 0;
    uint8_t crc = prom[7] & 0xF;
    prom[7] &= 0xFF00;
    bool blankEeprom = true;
    for (i = 0; i < 16; i++) {
        if (prom[i >> 1]) {
            blankEeprom = false;
        }
        if (i & 1)
            res ^= ((prom[i >> 1]) & 0x00FF);
        else
            res ^= (prom[i >> 1] >> 8);
        for (j = 8; j > 0; j--) {
            if (res & 0x8000)
                res ^= 0x1800;
            res <<= 1;
        }
    }
    prom[7] |= crc;
    if (!blankEeprom && crc == ((res >> 12) & 0xF))
        return 0;
    return -1;
}

static uint32_t ms5611_read_adc(busDevice_t *busdev) {
    uint8_t rxbuf[3];
    busReadRegisterBuffer(busdev, CMD_ADC_READ, rxbuf, 3); // read ADC
    return (rxbuf[0] << 16) | (rxbuf[1] << 8) | rxbuf[2];
}

static void ms5611_start_ut(baroDev_t *baro) {
    busWriteRegister(&baro->busdev, CMD_ADC_CONV + CMD_ADC_D2 + ms5611_osr, 1); // D2 (temperature) conversion start!
}

static void ms5611_get_ut(baroDev_t *baro) {
    ms5611_ut = ms5611_read_adc(&baro->busdev);
}

static void ms5611_start_up(baroDev_t *baro) {
    busWriteRegister(&baro->busdev, CMD_ADC_CONV + CMD_ADC_D1 + ms5611_osr, 1); // D1 (pressure) conversion start!
}

static void ms5611_get_up(baroDev_t *baro) {
    ms5611_up = ms5611_read_adc(&baro->busdev);
}

STATIC_UNIT_TESTED void ms5611_calculate(int32_t *pressure, int32_t *temperature) {
    uint32_t press;
    int64_t temp;
    int64_t delt;
    int64_t dT = (int64_t)ms5611_ut - ((uint64_t)ms5611_c[5] * 256);
    int64_t off = ((int64_t)ms5611_c[2] << 16) + (((int64_t)ms5611_c[4] * dT) >> 7);
    int64_t sens = ((int64_t)ms5611_c[1] << 15) + (((int64_t)ms5611_c[3] * dT) >> 8);
    temp = 2000 + ((dT * (int64_t)ms5611_c[6]) >> 23);
    if (temp < 2000) { // temperature lower than 20degC
        delt = temp - 2000;
        delt = 5 * delt * delt;
        off -= delt >> 1;
        sens -= delt >> 2;
        if (temp < -1500) { // temperature lower than -15degC
            delt = temp + 1500;
            delt = delt * delt;
            off -= 7 * delt;
            sens -= (11 * delt) >> 1;
        }
        temp -= ((dT * dT) >> 31);
    }
    press = ((((int64_t)ms5611_up * sens) >> 21) - off) >> 15;
    if (pressure)
        *pressure = press;
    if (temperature)
        *temperature = temp;
}
#endif
