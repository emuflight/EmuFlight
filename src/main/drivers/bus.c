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

#include "drivers/bus.h"
#include "drivers/bus_i2c_busdev.h"
#include "drivers/bus_spi.h"

bool busWriteRegister(const busDevice_t *busdev, uint8_t reg, uint8_t data) {
#ifdef USE_DMA_SPI_DEVICE
    return spiBusWriteRegister(busdev, reg & 0x7f, data);
#else
#if !defined(USE_SPI) && !defined(USE_I2C)
    UNUSED(reg);
    UNUSED(data);
#endif
    switch (busdev->bustype) {
#ifdef USE_SPI
    case BUSTYPE_SPI:
        return spiBusWriteRegister(busdev, reg & 0x7f, data);
#endif
#ifdef USE_I2C
    case BUSTYPE_I2C:
        return i2cBusWriteRegister(busdev, reg, data);
#endif
    default:
        return false;
    }
#endif
}

bool busReadRegisterBuffer(const busDevice_t *busdev, uint8_t reg, uint8_t *data, uint8_t length) {
#ifdef USE_DMA_SPI_DEVICE
    return spiBusReadRegisterBuffer(busdev, reg | 0x80, data, length);
#else
#if !defined(USE_SPI) && !defined(USE_I2C)
    UNUSED(reg);
    UNUSED(data);
    UNUSED(length);
#endif
    switch (busdev->bustype) {
#ifdef USE_SPI
    case BUSTYPE_SPI:
        return spiBusReadRegisterBuffer(busdev, reg | 0x80, data, length);
#endif
#ifdef USE_I2C
    case BUSTYPE_I2C:
        return i2cBusReadRegisterBuffer(busdev, reg, data, length);
#endif
    default:
        return false;
    }
#endif
}

uint8_t busReadRegister(const busDevice_t *busdev, uint8_t reg) {
#ifdef USE_DMA_SPI_DEVICE
    uint8_t data;
    busReadRegisterBuffer(busdev, reg, &data, 1);
    return data;
#else
#if !defined(USE_SPI) && !defined(USE_I2C)
    UNUSED(busdev);
    UNUSED(reg);
    return false;
#else
    uint8_t data;
    busReadRegisterBuffer(busdev, reg, &data, 1);
    return data;
#endif
#endif
}
