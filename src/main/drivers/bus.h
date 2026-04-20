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

#pragma once

#include "platform.h"

#include "drivers/bus_i2c.h"
#include "drivers/io_types.h"

typedef enum {
    BUS_TYPE_NONE = 0,
    BUS_TYPE_I2C,
    BUS_TYPE_SPI,
    BUS_TYPE_MPU_SLAVE // Slave I2C on SPI master
} busType_e;

// Shared bus-resource struct. Represents an SPI or I2C peripheral; one
// instance per physical bus. Does NOT carry per-device addressing (CS pin,
// I2C slave address). Matches Betaflight 4.5-maintenance layout minus the
// DMA/segment fields which are a later phase.
typedef struct busDevice_s {
    busType_e busType;
    union {
        struct busSpi_s {
            SPI_TypeDef *instance;
#if defined(USE_HAL_DRIVER)
            SPI_HandleTypeDef* handle; // cached here for efficiency
#endif
        } spi;
        struct busI2C_s {
            I2CDevice device;
        } i2c;
    } busType_u;
} busDevice_t;

// Per-device struct. Carries a back-pointer to the shared busDevice_t
// (populated by spiBusSetInstance for SPI devices; NULL for I2C and
// MPU-slave devices until their bus-resource split lands). The inline
// instance/device fields are still present and authoritative in this
// sub-commit; Stage I.4 migrates access sites to prefer dev->bus.
typedef struct extDevice_s {
    busDevice_t *bus;
    busType_e busType;
    union {
        struct extSpi_s {
            SPI_TypeDef *instance;
#if defined(USE_HAL_DRIVER)
            SPI_HandleTypeDef* handle; // cached here for efficiency
#endif
            IO_t csnPin;

#if defined(USE_GYRO_IMUF9001)
            IO_t rstPin;
#endif
        } spi;
        struct extI2C_s {
            I2CDevice device;
            uint8_t address;
        } i2c;
        struct extMpuSlave_s {
            const struct extDevice_s *master;
            uint8_t address;
        } mpuSlave;
    } busType_u;
} extDevice_t;

#ifdef TARGET_BUS_INIT
void targetBusInit(void);
#endif

bool busWriteRegister(const extDevice_t *dev, uint8_t reg, uint8_t data);
bool busReadRegisterBuffer(const extDevice_t *dev, uint8_t reg, uint8_t *data, uint8_t length);
uint8_t busReadRegister(const extDevice_t *dev, uint8_t reg);
