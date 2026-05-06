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

// Raw register access (register number passed as-is, no masking).
// For SPI: uses the RB variant so it returns false if the bus is busy.
bool busRawWriteRegister(const extDevice_t *dev, uint8_t reg, uint8_t data) {
#ifdef USE_SPI
    if (dev->bus->busType == BUS_TYPE_SPI) {
        return spiWriteRegRB(dev, reg, data);
    }
#endif
    return busWriteRegister(dev, reg, data);
}

bool busRawWriteRegisterStart(const extDevice_t *dev, uint8_t reg, uint8_t data) {
#ifdef USE_SPI
    if (dev->bus->busType == BUS_TYPE_SPI) {
        return spiWriteRegRB(dev, reg, data);
    }
#endif
    return busWriteRegisterStart(dev, reg, data);
}

bool busRawReadRegisterBuffer(const extDevice_t *dev, uint8_t reg, uint8_t *data, uint8_t length) {
#ifdef USE_SPI
    if (dev->bus->busType == BUS_TYPE_SPI) {
        return spiReadRegBufRB(dev, reg, data, length);
    }
#endif
    return busReadRegisterBuffer(dev, reg, data, length);
}

bool busRawReadRegisterBufferStart(const extDevice_t *dev, uint8_t reg, uint8_t *data, uint8_t length) {
#ifdef USE_SPI
    if (dev->bus->busType == BUS_TYPE_SPI) {
        return spiReadRegBufRB(dev, reg, data, length);
    }
#endif
    return busReadRegisterBufferStart(dev, reg, data, length);
}

// Write register, masking register address with 0x7f.
bool busWriteRegister(const extDevice_t *dev, uint8_t reg, uint8_t data) {
#if !defined(USE_SPI) && !defined(USE_I2C)
    UNUSED(reg);
    UNUSED(data);
#endif
    switch (dev->bus->busType) {
#ifdef USE_SPI
    case BUS_TYPE_SPI:
        return spiWriteRegRB(dev, reg & 0x7f, data);
#endif
#ifdef USE_I2C
    case BUS_TYPE_I2C:
        return i2cBusWriteRegister(dev, reg, data);
#endif
    default:
        return false;
    }
}

bool busWriteRegisterStart(const extDevice_t *dev, uint8_t reg, uint8_t data) {
    return busWriteRegister(dev, reg, data);
}

// Read register buffer, OR'ing register address with 0x80 for SPI read convention.
bool busReadRegisterBuffer(const extDevice_t *dev, uint8_t reg, uint8_t *data, uint8_t length) {
#if !defined(USE_SPI) && !defined(USE_I2C)
    UNUSED(reg);
    UNUSED(data);
    UNUSED(length);
#endif
    switch (dev->bus->busType) {
#ifdef USE_SPI
    case BUS_TYPE_SPI:
        return spiReadRegBufRB(dev, reg | 0x80, data, length);
#endif
#ifdef USE_I2C
    case BUS_TYPE_I2C:
        return i2cBusReadRegisterBuffer(dev, reg, data, length);
#endif
    default:
        return false;
    }
}

bool busReadRegisterBufferStart(const extDevice_t *dev, uint8_t reg, uint8_t *data, uint8_t length) {
    return busReadRegisterBuffer(dev, reg, data, length);
}

uint8_t busReadRegister(const extDevice_t *dev, uint8_t reg) {
#if !defined(USE_SPI) && !defined(USE_I2C)
    UNUSED(dev);
    UNUSED(reg);
    return 0;
#else
    uint8_t data;
    busReadRegisterBuffer(dev, reg, &data, 1);
    return data;
#endif
}

// busBusy is for I2C async operation polling. SPI transfers always complete before returning
// (via spiWait), so from the bus abstraction perspective SPI is never busy.
bool busBusy(const extDevice_t *dev, bool *error) {
    UNUSED(error);
#ifdef USE_SPI
    if (dev->bus->busType == BUS_TYPE_SPI) {
        return false;
    }
#else
    UNUSED(dev);
#endif
    return false;
}

void busDeviceRegister(const extDevice_t *dev) {
#ifdef USE_SPI
    if (dev->bus->busType == BUS_TYPE_SPI) {
        spiBusDeviceRegister(dev);
    }
#else
    UNUSED(dev);
#endif
}
