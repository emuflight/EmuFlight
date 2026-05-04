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

#include "build/atomic.h"

#ifdef USE_SPI

// Transmit-only DMA path: used when a bus has a Tx DMA channel but no Rx channel.
// Applicable to F4/F7; not needed on H7/G4 where full-duplex DMA is always available.
#if defined(STM32F4) || defined(STM32F7)
#define USE_TX_IRQ_HANDLER
#endif

// F7 cache-line constants for DMA buffer maintenance (32-byte L1D cache lines on Cortex-M7).
#ifdef STM32F7
#define CACHE_LINE_SIZE  32
#define CACHE_LINE_MASK  (CACHE_LINE_SIZE - 1)
#endif

#include "drivers/bus.h"
#include "drivers/bus_spi.h"
#include "drivers/bus_spi_impl.h"
#include "drivers/dma.h"
#include "drivers/dma_reqmap.h"
#include "drivers/nvic.h"
#include "drivers/resource.h"
#include "drivers/exti.h"
#include "drivers/io.h"
#include "drivers/rcc.h"
#ifdef USE_DMA_SPI_DEVICE
#ifndef GYRO_READ_TIMEOUT
#define GYRO_READ_TIMEOUT 20
#endif
#include "drivers/dma_spi.h"
#include "drivers/time.h"
#endif

static uint8_t spiRegisteredDeviceCount = 0;

FAST_RAM_ZERO_INIT spiDevice_t spiDevice[SPIDEV_COUNT];

// One busDevice_t per SPI peripheral. Shared by all extDevice_t instances on that bus.
// Populated by spiInit(); zero-init is correct (unused slots stay BUS_TYPE_NONE).
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
        busDevice_t *bus = spiBusByDevice(device);
        if (bus) {
            bus->busType = BUS_TYPE_SPI;
            bus->busType_u.spi.instance = spiDevice[device].dev;
#if defined(USE_HAL_DRIVER)
            bus->busType_u.spi.handle = &spiDevice[device].hspi;
#endif
            bus->curSegment = (busSegment_t *)BUS_SPI_FREE;
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

// Mark this bus as SPI using a 1-based CLI device id (BF 4.5-maintenance convention).
// Returns false if device is 0 (disabled), out of range, or the peripheral is absent.
bool spiSetBusInstance(extDevice_t *dev, uint32_t device) {
    if ((device == 0) || (device > SPIDEV_COUNT)) {
        return false;
    }
    SPI_TypeDef *instance = spiInstanceByDevice(SPI_CFG_TO_DEV(device));
    if (instance == NULL) {
        return false;
    }

    dev->bus = spiBusByDevice(SPI_CFG_TO_DEV(device));

    // By default each device uses DMA if the bus supports it (bus->useDMA starts false
    // until spiInitBusDMA enables it when channels are allocated).
    dev->useDMA = true;

    if (dev->bus->busType == BUS_TYPE_SPI) {
        // Bus already initialised by a prior call — increment device count.
        dev->bus->deviceCount++;
    } else {
        busDevice_t *bus = dev->bus;
        bus->busType = BUS_TYPE_SPI;
        bus->busType_u.spi.instance = instance;
#if defined(USE_HAL_DRIVER)
        bus->busType_u.spi.handle = &spiDevice[SPI_CFG_TO_DEV(device)].hspi;
#endif
        bus->useDMA = false;
        bus->deviceCount = 1;
#ifndef UNIT_TEST
        bus->initTx = &dev->initTx;
        bus->initRx = &dev->initRx;
#endif
        bus->curSegment = (busSegment_t *)BUS_SPI_FREE;
    }

    return true;
}

// Shared segment-completion handler called from spiRxIrqHandler / spiTxIrqHandler.
// Advances the segment list, starts the next DMA transfer, or marks the bus free.
FAST_CODE static void spiIrqHandler(const extDevice_t *dev)
{
    busDevice_t *bus = dev->bus;
    busSegment_t *nextSegment;

    if (bus->curSegment->callback) {
        switch (bus->curSegment->callback(dev->callbackArg)) {
        case BUS_BUSY:
            bus->curSegment--;
            spiInternalInitStream(dev, true);
            break;
        case BUS_ABORT:
            nextSegment = (busSegment_t *)bus->curSegment + 1;
            while (nextSegment->len != 0) {
                bus->curSegment = nextSegment;
                nextSegment = (busSegment_t *)bus->curSegment + 1;
            }
            break;
        case BUS_READY:
        default:
            break;
        }
    }

    nextSegment = (busSegment_t *)bus->curSegment + 1;

    if (nextSegment->len == 0) {
        if (nextSegment->u.link.dev) {
            const extDevice_t *nextDev = nextSegment->u.link.dev;
            busSegment_t *nextSegments = (busSegment_t *)nextSegment->u.link.segments;
            bus->curSegment = nextSegments;
            nextSegment->u.link.dev = NULL;
            nextSegment->u.link.segments = NULL;
            spiSequenceStart(nextDev);
        } else {
            bus->curSegment = (busSegment_t *)BUS_SPI_FREE;
        }
    } else {
        bool negateCS = bus->curSegment->negateCS;
        bus->curSegment = nextSegment;

        if (bus->initSegment) {
            spiInternalInitStream(dev, false);
            bus->initSegment = false;
        }

        if (negateCS) {
            IOLo(dev->busType_u.spi.csnPin);
        }

        spiInternalStartDMA(dev);
        spiInternalInitStream(dev, true);
    }
}

// DMA Rx TC IRQ handler — fires when SPI Rx DMA transfer completes.
// Negates CS, stops DMA, invalidates cache on F7, then advances the segment list.
FAST_CODE static void spiRxIrqHandler(dmaChannelDescriptor_t *descriptor)
{
    const extDevice_t *dev = (const extDevice_t *)descriptor->userParam;

    if (!dev) {
        return;
    }

    busDevice_t *bus = dev->bus;

    if (bus->curSegment->negateCS) {
        IOHi(dev->busType_u.spi.csnPin);
    }

    spiInternalStopDMA(dev);

#ifdef STM32F7
    if (bus->curSegment->u.buffers.rxData) {
        SCB_InvalidateDCache_by_Addr(
            (uint32_t *)((uint32_t)bus->curSegment->u.buffers.rxData & ~CACHE_LINE_MASK),
            (((uint32_t)bus->curSegment->u.buffers.rxData & CACHE_LINE_MASK) +
              bus->curSegment->len - 1 + CACHE_LINE_SIZE) & ~CACHE_LINE_MASK);
    }
#endif

    spiIrqHandler(dev);
}

#ifdef USE_TX_IRQ_HANDLER
// DMA Tx TC IRQ handler — used for Tx-only DMA buses (no Rx channel).
FAST_CODE static void spiTxIrqHandler(dmaChannelDescriptor_t *descriptor)
{
    const extDevice_t *dev = (const extDevice_t *)descriptor->userParam;

    if (!dev) {
        return;
    }

    busDevice_t *bus = dev->bus;

    spiInternalStopDMA(dev);

    if (bus->curSegment->negateCS) {
        IOHi(dev->busType_u.spi.csnPin);
    }

    spiIrqHandler(dev);
}
#endif

void spiInitBusDMA(void)
{
#if (defined(STM32F4) || defined(STM32F7)) && defined(USE_SPI)
    for (uint32_t device = 0; device < SPIDEV_COUNT; device++) {
        busDevice_t *bus = &spiBusDevice[device];

        if (bus->busType != BUS_TYPE_SPI) {
            continue;
        }

        dmaIdentifier_e dmaTxIdentifier = DMA_NONE;
        dmaIdentifier_e dmaRxIdentifier = DMA_NONE;

        for (uint8_t opt = 0; opt < MAX_PERIPHERAL_DMA_OPTIONS; opt++) {
            const dmaChannelSpec_t *dmaTxChannelSpec = dmaGetChannelSpecByPeripheral(DMA_PERIPH_SPI_SDO, device, opt);
            if (dmaTxChannelSpec) {
                dmaTxIdentifier = dmaGetIdentifier((DMA_Stream_TypeDef *)dmaTxChannelSpec->ref);
                if (!dmaAllocate(dmaTxIdentifier, OWNER_SPI_SDO, device + 1)) {
                    dmaTxIdentifier = DMA_NONE;
                    continue;
                }
                bus->dmaTx = dmaGetDescriptorByIdentifier(dmaTxIdentifier);
                bus->dmaTx->stream  = DMA_DEVICE_INDEX(dmaTxIdentifier);
                bus->dmaTx->channel = dmaTxChannelSpec->channel;
                dmaEnable(dmaTxIdentifier);
                break;
            }
        }

        for (uint8_t opt = 0; opt < MAX_PERIPHERAL_DMA_OPTIONS; opt++) {
            const dmaChannelSpec_t *dmaRxChannelSpec = dmaGetChannelSpecByPeripheral(DMA_PERIPH_SPI_SDI, device, opt);
            if (dmaRxChannelSpec) {
                dmaRxIdentifier = dmaGetIdentifier((DMA_Stream_TypeDef *)dmaRxChannelSpec->ref);
                if (!dmaAllocate(dmaRxIdentifier, OWNER_SPI_SDI, device + 1)) {
                    dmaRxIdentifier = DMA_NONE;
                    continue;
                }
                bus->dmaRx = dmaGetDescriptorByIdentifier(dmaRxIdentifier);
                bus->dmaRx->stream  = DMA_DEVICE_INDEX(dmaRxIdentifier);
                bus->dmaRx->channel = dmaRxChannelSpec->channel;
                dmaEnable(dmaRxIdentifier);
                break;
            }
        }

        if (dmaTxIdentifier && dmaRxIdentifier) {
            spiInternalResetStream(bus->dmaRx);
            spiInternalResetStream(bus->dmaTx);
            spiInternalResetDescriptors(bus);
            dmaSetHandler(dmaRxIdentifier, spiRxIrqHandler, NVIC_PRIO_SPI_DMA, 0);
            bus->useDMA = true;
#ifdef USE_TX_IRQ_HANDLER
        } else if (dmaTxIdentifier) {
            bus->dmaRx = (dmaChannelDescriptor_t *)NULL;
            spiInternalResetStream(bus->dmaTx);
            spiInternalResetDescriptors(bus);
            dmaSetHandler(dmaTxIdentifier, spiTxIrqHandler, NVIC_PRIO_SPI_DMA, 0);
            bus->useDMA = true;
#endif
        } else {
            bus->dmaRx = NULL;
            bus->dmaTx = NULL;
        }
    }
#endif
}

bool spiIsBusy(const extDevice_t *dev) {
    return (dev->bus->curSegment != (busSegment_t *)BUS_SPI_FREE);
}

void spiWait(const extDevice_t *dev) {
    while (spiIsBusy(dev));
}

void spiRelease(const extDevice_t *dev) {
    IOHi(dev->busType_u.spi.csnPin);
}

void spiDmaEnable(const extDevice_t *dev, bool enable) {
    ((extDevice_t *)dev)->useDMA = enable;
}

// Store per-device clock divisor; hardware applied at spiSequenceStart time.
void spiSetClkDivisor(const extDevice_t *dev, uint16_t divisor) {
    ((extDevice_t *)dev)->busType_u.spi.speed = divisor;
}

// Store per-device clock phase/polarity flag.
// HW re-configuration at transfer-start deferred to Stage M.3.
void spiSetClkPhasePolarity(const extDevice_t *dev, bool leadingEdge) {
    ((extDevice_t *)dev)->busType_u.spi.leadingEdge = leadingEdge;
}

bool spiUseDMA(const extDevice_t *dev) {
    if (!dev->bus->useDMA || !dev->useDMA) {
        return false;
    }
#ifndef UNIT_TEST
    return dev->bus->dmaRx != NULL;
#else
    return false;
#endif
}

bool spiUseSDO_DMA(const extDevice_t *dev) {
    return dev->bus->useDMA && dev->useDMA;
}

void spiBusDeviceRegister(const extDevice_t *dev) {
    UNUSED(dev);
    spiRegisteredDeviceCount++;
}

uint8_t spiGetRegisteredDeviceCount(void) {
    return spiRegisteredDeviceCount;
}

uint8_t spiGetExtDeviceCount(const extDevice_t *dev) {
    return dev->bus->deviceCount;
}

void spiLinkSegments(const extDevice_t *dev, busSegment_t *firstSegment, busSegment_t *secondSegment) {
    busSegment_t *endSegment;
    for (endSegment = firstSegment; endSegment->len; endSegment++);
    endSegment->u.link.dev = dev;
    endSegment->u.link.segments = secondSegment;
}

void spiSequence(const extDevice_t *dev, busSegment_t *segments) {
    busDevice_t *bus = dev->bus;

#ifdef USE_DMA_SPI_DEVICE
    if (dev->bus->busType_u.spi.instance == USE_DMA_SPI_DEVICE) {
        // IMUF9001 blocking custom DMA — incompatible with non-blocking segment queuing.
        spiWait(dev);
        bus->curSegment = segments;
        if (segments[0].len > 0 && segments[1].len == 0) {
            uint8_t *txData = segments[0].u.buffers.txData;
            uint8_t *rxData = segments[0].u.buffers.rxData;
            int len = segments[0].len;
            uint32_t timeoutCheck = millis();
            if (txData) {
                memcpy(dmaTxBuffer, txData, len);
            } else {
                memset(dmaTxBuffer, 0xFF, len);
            }
            dmaSpiTransmitReceive(dmaTxBuffer, dmaRxBuffer, len, 1);
            while (dmaSpiReadStatus != DMA_SPI_READ_DONE) {
                if (millis() - timeoutCheck > GYRO_READ_TIMEOUT) {
                    IOHi(dev->busType_u.spi.csnPin);
                    break;
                }
            }
            if (rxData) {
                memcpy(rxData, dmaRxBuffer, len);
            }
            if (segments[0].callback) {
                segments[0].callback(dev->callbackArg);
            }
            bus->curSegment = (busSegment_t *)BUS_SPI_FREE;
            return;
        }
        spiSequenceStart(dev);
        return;
    }
#endif

    ATOMIC_BLOCK(NVIC_PRIO_MAX) {
        if (spiIsBusy(dev)) {
            busSegment_t *endSegment;

            // Find the last segment of the new transfer
            for (endSegment = segments; endSegment->len; endSegment++);

            // Safe to discard the volatile qualifier as we're in an atomic block
            busSegment_t *endCmpSegment = (busSegment_t *)bus->curSegment;

            if (endCmpSegment) {
                while (true) {
                    // Find the last segment of the current transfer
                    for (; endCmpSegment->len; endCmpSegment++);

                    if (endCmpSegment == endSegment) {
                        // Same segment list queued twice; abort.
                        return;
                    }

                    if (endCmpSegment->u.link.dev == NULL) {
                        break;
                    } else {
                        endCmpSegment = (busSegment_t *)endCmpSegment->u.link.segments;
                    }
                }

                endCmpSegment->u.link.dev = dev;
                endCmpSegment->u.link.segments = segments;
            }

            return;
        } else {
            bus->curSegment = segments;
        }
    }

    spiSequenceStart(dev);
}

void spiReadWriteBuf(const extDevice_t *dev, uint8_t *txData, uint8_t *rxData, int len) {
    busSegment_t segments[] = {
        {.u.buffers = {txData, rxData}, len, true, NULL},
        {.u.link = {NULL, NULL}, 0, true, NULL},
    };
    spiSequence(dev, &segments[0]);
    spiWait(dev);
}

bool spiReadWriteBufRB(const extDevice_t *dev, uint8_t *txData, uint8_t *rxData, int length) {
    if (spiIsBusy(dev)) {
        return false;
    }
    spiReadWriteBuf(dev, txData, rxData, length);
    return true;
}

uint8_t spiReadWrite(const extDevice_t *dev, uint8_t data) {
    uint8_t retval;
    busSegment_t segments[] = {
        {.u.buffers = {&data, &retval}, sizeof(data), true, NULL},
        {.u.link = {NULL, NULL}, 0, true, NULL},
    };
    spiSequence(dev, &segments[0]);
    spiWait(dev);
    return retval;
}

uint8_t spiReadWriteReg(const extDevice_t *dev, uint8_t reg, uint8_t data) {
    uint8_t retval;
    busSegment_t segments[] = {
        {.u.buffers = {&reg, NULL}, sizeof(reg), false, NULL},
        {.u.buffers = {&data, &retval}, sizeof(data), true, NULL},
        {.u.link = {NULL, NULL}, 0, true, NULL},
    };
    spiSequence(dev, &segments[0]);
    spiWait(dev);
    return retval;
}

void spiWrite(const extDevice_t *dev, uint8_t data) {
    busSegment_t segments[] = {
        {.u.buffers = {&data, NULL}, sizeof(data), true, NULL},
        {.u.link = {NULL, NULL}, 0, true, NULL},
    };
    spiSequence(dev, &segments[0]);
    spiWait(dev);
}

void spiWriteReg(const extDevice_t *dev, uint8_t reg, uint8_t data) {
    busSegment_t segments[] = {
        {.u.buffers = {&reg, NULL}, sizeof(reg), false, NULL},
        {.u.buffers = {&data, NULL}, sizeof(data), true, NULL},
        {.u.link = {NULL, NULL}, 0, true, NULL},
    };
    spiSequence(dev, &segments[0]);
    spiWait(dev);
}

bool spiWriteRegRB(const extDevice_t *dev, uint8_t reg, uint8_t data) {
    if (spiIsBusy(dev)) {
        return false;
    }
    spiWriteReg(dev, reg, data);
    return true;
}

void spiWriteRegBuf(const extDevice_t *dev, uint8_t reg, uint8_t *data, uint32_t length) {
    busSegment_t segments[] = {
        {.u.buffers = {&reg, NULL}, sizeof(reg), false, NULL},
        {.u.buffers = {data, NULL}, length, true, NULL},
        {.u.link = {NULL, NULL}, 0, true, NULL},
    };
    spiSequence(dev, &segments[0]);
    spiWait(dev);
}

void spiReadRegBuf(const extDevice_t *dev, uint8_t reg, uint8_t *data, uint8_t length) {
    busSegment_t segments[] = {
        {.u.buffers = {&reg, NULL}, sizeof(reg), false, NULL},
        {.u.buffers = {NULL, data}, length, true, NULL},
        {.u.link = {NULL, NULL}, 0, true, NULL},
    };
    spiSequence(dev, &segments[0]);
    spiWait(dev);
}

bool spiReadRegBufRB(const extDevice_t *dev, uint8_t reg, uint8_t *data, uint8_t length) {
    if (spiIsBusy(dev)) {
        return false;
    }
    spiReadRegBuf(dev, reg, data, length);
    return true;
}

bool spiReadRegMskBufRB(const extDevice_t *dev, uint8_t reg, uint8_t *data, uint8_t length) {
    return spiReadRegBufRB(dev, reg | 0x80, data, length);
}

uint8_t spiReadReg(const extDevice_t *dev, uint8_t reg) {
    uint8_t data;
    busSegment_t segments[] = {
        {.u.buffers = {&reg, NULL}, sizeof(reg), false, NULL},
        {.u.buffers = {NULL, &data}, sizeof(data), true, NULL},
        {.u.link = {NULL, NULL}, 0, true, NULL},
    };
    spiSequence(dev, &segments[0]);
    spiWait(dev);
    return data;
}

uint8_t spiReadRegMsk(const extDevice_t *dev, uint8_t reg) {
    return spiReadReg(dev, reg | 0x80);
}

uint16_t spiCalculateDivider(uint32_t freq) {
#if defined(STM32F4) || defined(STM32G4) || defined(STM32F7)
    uint32_t spiClk = SystemCoreClock / 2;
#elif defined(STM32H7)
    uint32_t spiClk = 100000000;
#elif defined(AT32F4)
    if (freq > 36000000) {
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

uint32_t spiCalculateClock(uint16_t spiClkDivisor) {
#if defined(STM32F4) || defined(STM32G4) || defined(STM32F7)
    uint32_t spiClk = SystemCoreClock / 2;
#elif defined(STM32H7)
    uint32_t spiClk = 100000000;
#elif defined(AT32F4)
    uint32_t spiClk = system_core_clock / 2;
    if ((spiClk / spiClkDivisor) > 36000000) {
        return 36000000;
    }
#else
#error "Base SPI clock not defined for this architecture"
#endif
    return spiClk / spiClkDivisor;
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

#endif
