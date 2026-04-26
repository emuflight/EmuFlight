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
#ifndef UNIT_TEST
#include "drivers/dma.h"
#endif

typedef enum {
    BUS_TYPE_NONE = 0,
    BUS_TYPE_I2C,
    BUS_TYPE_SPI,
    BUS_TYPE_MPU_SLAVE, // Slave I2C on SPI master
    BUS_TYPE_GYRO_AUTO, // Only used by acc/gyro bus auto detection code
} busType_e;

typedef enum {
    BUS_READY,
    BUS_BUSY,
    BUS_ABORT
} busStatus_e;

struct extDevice_s;

struct busSegment_s;

// Shared bus-resource struct. Represents an SPI, I2C, or MPU-slave peripheral;
// one instance per physical bus. Does NOT carry per-device addressing (CS pin,
// I2C slave address). Matches Betaflight 4.5-maintenance layout.
typedef struct busDevice_s {
    busType_e busType;
    union {
        struct busSpi_s {
            SPI_TypeDef *instance;
#if defined(USE_HAL_DRIVER)
            SPI_HandleTypeDef* handle; // cached here for efficiency
#endif
            uint16_t speed;
            bool leadingEdge;
        } spi;
        struct busI2C_s {
            I2CDevice device;
        } i2c;
        struct busMpuSlave_s {
            const struct extDevice_s *master;
        } mpuSlave;
    } busType_u;
    bool useDMA;
    uint8_t deviceCount;
#ifndef UNIT_TEST
    // dmaChannelDescriptor_t requires dma.h which is not available in unit test builds
    dmaChannelDescriptor_t *dmaTx;
    dmaChannelDescriptor_t *dmaRx;
#if defined(STM32F7)
    LL_DMA_InitTypeDef *initTx;
    LL_DMA_InitTypeDef *initRx;
#else
    DMA_InitTypeDef    *initTx;
    DMA_InitTypeDef    *initRx;
#endif
#endif // UNIT_TEST
    volatile struct busSegment_s *volatile curSegment;
    bool initSegment;
} busDevice_t;

// Per-device struct. Carries a back-pointer to the shared busDevice_t
// (populated by spiSetBusInstance for SPI devices). Bus type and instance
// are accessed via the bus back-pointer (dev->bus->busType / bus->busType_u.spi.instance).
typedef struct extDevice_s {
    busDevice_t *bus;
    union {
        struct extSpi_s {
            IO_t csnPin;
            uint16_t speed;
            bool leadingEdge;
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
#ifndef UNIT_TEST
    // Per-device DMA init cache — reduces inter-segment setup time (BF 4.5-m pattern).
    // Pointer stored in bus->initTx/initRx so the bus driver always references the
    // current device's cache.
#if defined(STM32F7)
    LL_DMA_InitTypeDef initTx;
    LL_DMA_InitTypeDef initRx;
#else
    DMA_InitTypeDef    initTx;
    DMA_InitTypeDef    initRx;
#endif
#endif // UNIT_TEST
    bool useDMA;
    uint8_t *txBuf, *rxBuf;
    uint32_t callbackArg;
} extDevice_t;

/* Each SPI access may comprise multiple segments (e.g. reg-write then data-read),
 * each described by one busSegment_t entry terminated by a sentinel with len == 0.
 * negateCS controls whether CS is deasserted between segments.
 * callback (if non-NULL) is called after each segment completes; its return value
 * may request BUS_BUSY (repeat), BUS_ABORT (skip remaining), or BUS_READY (advance).
 * Stage M.1 sync path advances unconditionally; BUS_ABORT/BUS_BUSY only honored
 * by the async DMA path (Stage M.3).
 */
typedef struct busSegment_s {
    union {
        struct {
            uint8_t *txData;
            uint8_t *rxData;
        } buffers;
        struct {
            const struct extDevice_s *dev;
            volatile struct busSegment_s *segments;
        } link;
    } u;
    int len;
    bool negateCS;
    busStatus_e (*callback)(uint32_t arg);
} busSegment_t;

#ifdef TARGET_BUS_INIT
void targetBusInit(void);
#endif

// Register access with register number passed as-is (no masking)
bool busRawWriteRegister(const extDevice_t *dev, uint8_t reg, uint8_t data);
bool busRawWriteRegisterStart(const extDevice_t *dev, uint8_t reg, uint8_t data);
bool busRawReadRegisterBuffer(const extDevice_t *dev, uint8_t reg, uint8_t *data, uint8_t length);
bool busRawReadRegisterBufferStart(const extDevice_t *dev, uint8_t reg, uint8_t *data, uint8_t length);

// Write: register masked with 0x7f; Read: register OR'd with 0x80
bool busWriteRegister(const extDevice_t *dev, uint8_t reg, uint8_t data);
bool busWriteRegisterStart(const extDevice_t *dev, uint8_t reg, uint8_t data);
bool busReadRegisterBuffer(const extDevice_t *dev, uint8_t reg, uint8_t *data, uint8_t length);
bool busReadRegisterBufferStart(const extDevice_t *dev, uint8_t reg, uint8_t *data, uint8_t length);
uint8_t busReadRegister(const extDevice_t *dev, uint8_t reg);

bool busBusy(const extDevice_t *dev, bool *error);
void busDeviceRegister(const extDevice_t *dev);
