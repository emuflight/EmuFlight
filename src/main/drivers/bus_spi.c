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
#include <string.h>

#include "platform.h"

#ifdef USE_SPI

#include "drivers/bus.h"
#include "drivers/bus_spi.h"
#include "drivers/bus_spi_impl.h"
#include "drivers/exti.h"
#include "drivers/io.h"
#include "drivers/rcc.h"
#ifdef USE_DMA_SPI_DEVICE
#ifndef GYRO_READ_TIMEOUT
#define GYRO_READ_TIMEOUT 20
#endif //GYRO_READ_TIMEOUT
#include "drivers/dma_spi.h"
#include "drivers/time.h"
#endif //USE_DMA_SPI_DEVICE

FAST_RAM_ZERO_INIT spiDevice_t spiDevice[SPIDEV_COUNT];

// Bus-abstraction layer: one busDevice_t per SPI peripheral. Shared by every
// extDevice_t that uses this bus. Populated by spiInit() on successful init
// of each SPI peripheral. Not statically initialised — zero-init is fine
// because unused slots stay BUS_TYPE_NONE.
FAST_RAM_ZERO_INIT static busDevice_t spiBusDevice[SPIDEV_COUNT];

busDevice_t *spiBusByDevice(SPIDevice device) {
    if (device == SPIINVALID || device >= SPIDEV_COUNT) {
        return NULL;
    }
    return &spiBusDevice[device];
}

SPIDevice spiDeviceByInstance(SPI_TypeDef *instance) {
#ifdef USE_SPI_DEVICE_1
    if (instance == SPI1)
        return SPIDEV_1;
#endif
#ifdef USE_SPI_DEVICE_2
    if (instance == SPI2)
        return SPIDEV_2;
#endif
#ifdef USE_SPI_DEVICE_3
    if (instance == SPI3)
        return SPIDEV_3;
#endif
#ifdef USE_SPI_DEVICE_4
    if (instance == SPI4)
        return SPIDEV_4;
#endif
    return SPIINVALID;
}

SPI_TypeDef *spiInstanceByDevice(SPIDevice device) {
    if (device >= SPIDEV_COUNT) {
        return NULL;
    }
    return spiDevice[device].dev;
}

bool spiInit(SPIDevice device) {
    bool ok = false;
    switch (device) {
    case SPIINVALID:
        return false;
    case SPIDEV_1:
#ifdef USE_SPI_DEVICE_1
        spiInitDevice(device);
        ok = true;
#endif
        break;
    case SPIDEV_2:
#ifdef USE_SPI_DEVICE_2
        spiInitDevice(device);
        ok = true;
#endif
        break;
    case SPIDEV_3:
#if defined(USE_SPI_DEVICE_3) && !defined(STM32F1)
        spiInitDevice(device);
        ok = true;
#endif
        break;
    case SPIDEV_4:
#if defined(USE_SPI_DEVICE_4)
        spiInitDevice(device);
        ok = true;
#endif
        break;
    }
    if (ok) {
        // Populate bus-abstraction resource for this peripheral. Later stages
        // migrate extDevice_t to dereference dev->bus->busType_u.spi.instance
        // instead of the per-device inline copy. Route through spiBusByDevice()
        // so the write shares the read path's bounds check.
        busDevice_t *bus = spiBusByDevice(device);
        if (bus) {
            bus->busType = BUS_TYPE_SPI;
            bus->busType_u.spi.instance = spiDevice[device].dev;
#if defined(USE_HAL_DRIVER)
            bus->busType_u.spi.handle = &spiDevice[device].hspi;
#endif
        }
    }
    return ok;
}

uint32_t spiTimeoutUserCallback(SPI_TypeDef *instance) {
    SPIDevice device = spiDeviceByInstance(instance);
    if (device == SPIINVALID) {
        return -1;
    }
    spiDevice[device].errorCount++;
    return spiDevice[device].errorCount;
}


FAST_CODE bool spiReadWriteBuf(const extDevice_t *dev, const uint8_t *txData, uint8_t *rxData, int length) {
#ifdef USE_DMA_SPI_DEVICE
    if(USE_DMA_SPI_DEVICE == dev->busType_u.spi.instance) {
        uint32_t timeoutCheck = millis();
        memcpy(dmaTxBuffer, (uint8_t *)txData, length);
        dmaSpiTransmitReceive(dmaTxBuffer, dmaRxBuffer, length, 1);
        while(dmaSpiReadStatus != DMA_SPI_READ_DONE) {
            if(millis() - timeoutCheck > GYRO_READ_TIMEOUT) {
                //GYRO_READ_TIMEOUT ms max, read failed, cleanup spi and return 0
                IOHi(dev->busType_u.spi.csnPin);
                return false;
            }
        }
        memcpy((uint8_t *)rxData, dmaRxBuffer, length);
    } else {
        IOLo(dev->busType_u.spi.csnPin);
        spiTransfer(dev->busType_u.spi.instance, txData, rxData, length);
        IOHi(dev->busType_u.spi.csnPin);
    }
#else
    IOLo(dev->busType_u.spi.csnPin);
    spiTransfer(dev->busType_u.spi.instance, txData, rxData, length);
    IOHi(dev->busType_u.spi.csnPin);
#endif
    return true;
}

uint16_t spiGetErrorCounter(SPI_TypeDef *instance) {
    SPIDevice device = spiDeviceByInstance(instance);
    if (device == SPIINVALID) {
        return 0;
    }
    return spiDevice[device].errorCount;
}

void spiResetErrorCounter(SPI_TypeDef *instance) {
    SPIDevice device = spiDeviceByInstance(instance);
    if (device != SPIINVALID) {
        spiDevice[device].errorCount = 0;
    }
}

FAST_CODE bool spiWriteReg(const extDevice_t *dev, uint8_t reg, uint8_t data) {
#ifdef USE_DMA_SPI_DEVICE
    if(USE_DMA_SPI_DEVICE == dev->busType_u.spi.instance) {
        uint32_t timeoutCheck = millis();
        dmaTxBuffer[0] = reg;
        dmaTxBuffer[1] = data;
        dmaSpiTransmitReceive(dmaTxBuffer, dmaRxBuffer, 2, 1);
        while(dmaSpiReadStatus != DMA_SPI_READ_DONE) {
            if(millis() - timeoutCheck > GYRO_READ_TIMEOUT) {
                //GYRO_READ_TIMEOUT ms max, read failed, cleanup spi and return 0
                IOHi(dev->busType_u.spi.csnPin);
                return false;
            }
        }
    } else {
        IOLo(dev->busType_u.spi.csnPin);
        spiTransferByte(dev->busType_u.spi.instance, reg);
        spiTransferByte(dev->busType_u.spi.instance, data);
        IOHi(dev->busType_u.spi.csnPin);
    }
#else
    IOLo(dev->busType_u.spi.csnPin);
    spiTransferByte(dev->busType_u.spi.instance, reg);
    spiTransferByte(dev->busType_u.spi.instance, data);
    IOHi(dev->busType_u.spi.csnPin);
#endif
    return true;
}

FAST_CODE bool spiReadRegBuf(const extDevice_t *dev, uint8_t reg, uint8_t *data, uint8_t length) {
#ifdef USE_DMA_SPI_DEVICE
    if(USE_DMA_SPI_DEVICE == dev->busType_u.spi.instance) {
        uint32_t timeoutCheck = millis();
        dmaTxBuffer[0] = reg | 0x80;
        dmaSpiTransmitReceive(dmaTxBuffer, dmaRxBuffer, length + 1, 1);
        while(dmaSpiReadStatus != DMA_SPI_READ_DONE) {
            if(millis() - timeoutCheck > GYRO_READ_TIMEOUT) {
                //GYRO_READ_TIMEOUT ms max, read failed, cleanup spi and return 0
                IOHi(dev->busType_u.spi.csnPin);
                return false;
            }
        }
        memcpy(data, dmaRxBuffer + 1, length);
    } else {
        IOLo(dev->busType_u.spi.csnPin);
        spiTransferByte(dev->busType_u.spi.instance, reg | 0x80); // read transaction
        spiTransfer(dev->busType_u.spi.instance, NULL, data, length);
        IOHi(dev->busType_u.spi.csnPin);
    }
#else
    IOLo(dev->busType_u.spi.csnPin);
    spiTransferByte(dev->busType_u.spi.instance, reg | 0x80); // read transaction
    spiTransfer(dev->busType_u.spi.instance, NULL, data, length);
    IOHi(dev->busType_u.spi.csnPin);
#endif
    return true;
}

FAST_CODE uint8_t spiReadReg(const extDevice_t *dev, uint8_t reg) {
#ifdef USE_DMA_SPI_DEVICE
    if(USE_DMA_SPI_DEVICE == dev->busType_u.spi.instance) {
        uint32_t timeoutCheck = millis();
        dmaTxBuffer[0] = reg | 0x80;
        dmaSpiTransmitReceive(dmaTxBuffer, dmaRxBuffer, 2, 1);
        while(dmaSpiReadStatus != DMA_SPI_READ_DONE) {
            if(millis() - timeoutCheck > GYRO_READ_TIMEOUT) {
                //GYRO_READ_TIMEOUT ms max, read failed, cleanup spi and return 0
                IOHi(dev->busType_u.spi.csnPin);
                return 0;
            }
        }
        return dmaRxBuffer[1];
    } else {
        uint8_t data;
        IOLo(dev->busType_u.spi.csnPin);
        spiTransferByte(dev->busType_u.spi.instance, reg | 0x80); // read transaction
        spiTransfer(dev->busType_u.spi.instance, NULL, &data, 1);
        IOHi(dev->busType_u.spi.csnPin);
        return data;
    }
#else
    uint8_t data;
    IOLo(dev->busType_u.spi.csnPin);
    spiTransferByte(dev->busType_u.spi.instance, reg | 0x80); // read transaction
    spiTransfer(dev->busType_u.spi.instance, NULL, &data, 1);
    IOHi(dev->busType_u.spi.csnPin);
    return data;
#endif
}

void spiBusSetInstance(extDevice_t *dev, SPI_TypeDef *instance) {
    dev->busType = BUS_TYPE_SPI;
    dev->busType_u.spi.instance = instance;
    // Wire the bus-abstraction back-pointer. spiInit() must have already
    // populated spiBusDevice[] for this peripheral; if the caller passes an
    // instance that does not map to a known SPI device, bus stays NULL and
    // callers that rely on dev->bus will hit a clear null deref at first
    // use rather than a silent wrong-peripheral access.
    dev->bus = spiBusByDevice(spiDeviceByInstance(instance));
}

// icm42688p and bmi270 porting
uint16_t spiCalculateDivider(uint32_t freq)
{
#if defined(STM32F4) || defined(STM32G4) || defined(STM32F7)
    uint32_t spiClk = SystemCoreClock / 2;
#elif defined(STM32H7)
    uint32_t spiClk = 100000000;
#elif defined(AT32F4)
    if(freq > 36000000){
        freq = 36000000;
    }
    uint32_t spiClk = system_core_clock / 2;
#else
#error "Base SPI clock not defined for this architecture"
#endif
    uint16_t divisor = 2;
    spiClk >>= 1;
    for (; (spiClk > freq) && (divisor < 256); divisor <<= 1, spiClk >>= 1);
    return divisor;
}

// Wait for bus to become free, then read a byte of data where the register is bitwise OR'ed with 0x80
// EmuFlight codebase is old.  Bitwise or 0x80 is redundant here as spiReadReg already contains such.
uint8_t spiReadRegMsk(const extDevice_t *dev, uint8_t reg)
{
    return spiReadReg(dev, reg | 0x80);
}

#endif
