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

#if defined(USE_SPI)

#include "common/utils.h"

#include "drivers/bus.h"
#include "drivers/bus_spi.h"
#include "drivers/bus_spi_impl.h"
#include "drivers/dma.h"
#include "drivers/io.h"
#include "drivers/nvic.h"
#include "drivers/rcc.h"

spiDevice_t spiDevice[SPIDEV_COUNT];

#ifndef SPI2_SCK_PIN
#define SPI2_NSS_PIN    PB12
#define SPI2_SCK_PIN    PB13
#define SPI2_MISO_PIN   PB14
#define SPI2_MOSI_PIN   PB15
#endif

#ifndef SPI3_SCK_PIN
#define SPI3_NSS_PIN    PA15
#define SPI3_SCK_PIN    PB3
#define SPI3_MISO_PIN   PB4
#define SPI3_MOSI_PIN   PB5
#endif

#ifndef SPI4_SCK_PIN
#define SPI4_NSS_PIN    PA15
#define SPI4_SCK_PIN    PB3
#define SPI4_MISO_PIN   PB4
#define SPI4_MOSI_PIN   PB5
#endif

#ifndef SPI1_NSS_PIN
#define SPI1_NSS_PIN NONE
#endif
#ifndef SPI2_NSS_PIN
#define SPI2_NSS_PIN NONE
#endif
#ifndef SPI3_NSS_PIN
#define SPI3_NSS_PIN NONE
#endif
#ifndef SPI4_NSS_PIN
#define SPI4_NSS_PIN NONE
#endif

#define SPI_DEFAULT_TIMEOUT 10
#define SPI_DMA_THRESHOLD 8

#ifdef STM32F7
#define IS_DTCM(p) (((uint32_t)(p) & 0xffff0000) == 0x20000000)
#endif

static uint32_t spiDivisorToBRbits(SPI_TypeDef *instance, uint16_t divisor)
{
#if !(defined(STM32F1) || defined(STM32F3))
    if (instance == SPI2 || instance == SPI3) {
        divisor /= 2;
    }
#else
    UNUSED(instance);
#endif
    // divisor | 0x100 ensures non-zero; ffs maps power-of-2 divisor to prescaler index.
    return (ffs(divisor | 0x100) - 2) << SPI_CR1_BR_Pos;
}

void spiInitDevice(SPIDevice device) {
    spiDevice_t *spi = &(spiDevice[device]);
#ifdef SDCARD_SPI_INSTANCE
    if (spi->dev == SDCARD_SPI_INSTANCE) {
        spi->leadingEdge = true;
    }
#endif
#ifdef RX_SPI_INSTANCE
    if (spi->dev == RX_SPI_INSTANCE) {
        spi->leadingEdge = true;
    }
#endif
    // Enable SPI clock
    RCC_ClockCmd(spi->rcc, ENABLE);
    RCC_ResetCmd(spi->rcc, ENABLE);
    IOInit(IOGetByTag(spi->sck),  OWNER_SPI_SCK,  RESOURCE_INDEX(device));
    IOInit(IOGetByTag(spi->miso), OWNER_SPI_MISO, RESOURCE_INDEX(device));
    IOInit(IOGetByTag(spi->mosi), OWNER_SPI_MOSI, RESOURCE_INDEX(device));
    if (spi->leadingEdge == true)
        IOConfigGPIOAF(IOGetByTag(spi->sck), SPI_IO_AF_SCK_CFG_LOW, spi->sckAF);
    else
        IOConfigGPIOAF(IOGetByTag(spi->sck), SPI_IO_AF_SCK_CFG_HIGH, spi->sckAF);
    IOConfigGPIOAF(IOGetByTag(spi->miso), SPI_IO_AF_MISO_CFG, spi->misoAF);
    IOConfigGPIOAF(IOGetByTag(spi->mosi), SPI_IO_AF_CFG, spi->mosiAF);
    LL_SPI_Disable(spi->dev);
    LL_SPI_DeInit(spi->dev);
    LL_SPI_InitTypeDef init = {
        .TransferDirection = SPI_DIRECTION_2LINES,
        .Mode = SPI_MODE_MASTER,
        .DataWidth = SPI_DATASIZE_8BIT,
        .ClockPolarity = spi->leadingEdge ? SPI_POLARITY_LOW : SPI_POLARITY_HIGH,
        .ClockPhase = spi->leadingEdge ? SPI_PHASE_1EDGE : SPI_PHASE_2EDGE,
        .NSS = SPI_NSS_SOFT,
        .BaudRate = SPI_BAUDRATEPRESCALER_8,
        .BitOrder = SPI_FIRSTBIT_MSB,
        .CRCPoly = 7,
        .CRCCalculation = SPI_CRCCALCULATION_DISABLE,
    };
    LL_SPI_SetRxFIFOThreshold(spi->dev, SPI_RXFIFO_THRESHOLD_QF);
    LL_SPI_Init(spi->dev, &init);
    LL_SPI_Enable(spi->dev);
}

uint8_t spiTransferByte(SPI_TypeDef *instance, uint8_t txByte) {
    uint16_t spiTimeout = 1000;
    while (!LL_SPI_IsActiveFlag_TXE(instance))
        if ((spiTimeout--) == 0)
            return spiTimeoutUserCallback(instance);
    LL_SPI_TransmitData8(instance, txByte);
    spiTimeout = 1000;
    while (!LL_SPI_IsActiveFlag_RXNE(instance))
        if ((spiTimeout--) == 0)
            return spiTimeoutUserCallback(instance);
    return (uint8_t)LL_SPI_ReceiveData8(instance);
}

/**
 * Return true if the bus is currently in the middle of a transmission.
 */
bool spiIsBusBusy(SPI_TypeDef *instance) {
    return LL_SPI_GetTxFIFOLevel(instance) != LL_SPI_TX_FIFO_EMPTY
           || LL_SPI_IsActiveFlag_BSY(instance);
}

bool spiTransfer(SPI_TypeDef *instance, const uint8_t *txData, uint8_t *rxData, int len) {
    // set 16-bit transfer
    CLEAR_BIT(instance->CR2, SPI_RXFIFO_THRESHOLD);
    while (len > 1) {
        int spiTimeout = 1000;
        while (!LL_SPI_IsActiveFlag_TXE(instance)) {
            if ((spiTimeout--) == 0) {
                return spiTimeoutUserCallback(instance);
            }
        }
        uint16_t w;
        if (txData) {
            w = *((uint16_t *)txData);
            txData += 2;
        } else {
            w = 0xFFFF;
        }
        LL_SPI_TransmitData16(instance, w);
        spiTimeout = 1000;
        while (!LL_SPI_IsActiveFlag_RXNE(instance)) {
            if ((spiTimeout--) == 0) {
                return spiTimeoutUserCallback(instance);
            }
        }
        w = LL_SPI_ReceiveData16(instance);
        if (rxData) {
            *((uint16_t *)rxData) = w;
            rxData += 2;
        }
        len -= 2;
    }
    // set 8-bit transfer
    SET_BIT(instance->CR2, SPI_RXFIFO_THRESHOLD);
    if (len) {
        int spiTimeout = 1000;
        while (!LL_SPI_IsActiveFlag_TXE(instance)) {
            if ((spiTimeout--) == 0) {
                return spiTimeoutUserCallback(instance);
            }
        }
        uint8_t b = txData ? *(txData++) : 0xFF;
        LL_SPI_TransmitData8(instance, b);
        spiTimeout = 1000;
        while (!LL_SPI_IsActiveFlag_RXNE(instance)) {
            if ((spiTimeout--) == 0) {
                return spiTimeoutUserCallback(instance);
            }
        }
        b = LL_SPI_ReceiveData8(instance);
        if (rxData) {
            *(rxData++) = b;
        }
        --len;
    }
    return true;
}

// DMA transfer setup and start (BF 4.5-maintenance parity).
// Handles per-device speed and polarity switching, then dispatches to DMA path
// (when bus->useDMA is enabled by spiInitBusDMA, Stage M.3.d) or polled path.
FAST_CODE void spiSequenceStart(const extDevice_t *dev)
{
    busDevice_t *bus = dev->bus;
    SPI_TypeDef *instance = bus->busType_u.spi.instance;
    spiDevice_t *spi = &spiDevice[spiDeviceByInstance(instance)];
    bool dmaSafe = dev->useDMA;
    uint32_t xferLen = 0;
    uint32_t segmentCount = 0;

    bus->initSegment = true;

    LL_SPI_Disable(instance);

    if (dev->busType_u.spi.speed != bus->busType_u.spi.speed) {
        LL_SPI_SetBaudRatePrescaler(instance, spiDivisorToBRbits(instance, dev->busType_u.spi.speed));
        bus->busType_u.spi.speed = dev->busType_u.spi.speed;
    }

    if (dev->busType_u.spi.leadingEdge != bus->busType_u.spi.leadingEdge) {
        if (dev->busType_u.spi.leadingEdge) {
            IOConfigGPIOAF(IOGetByTag(spi->sck), SPI_IO_AF_SCK_CFG_LOW, spi->sckAF);
            LL_SPI_SetClockPhase(instance, LL_SPI_PHASE_1EDGE);
            LL_SPI_SetClockPolarity(instance, LL_SPI_POLARITY_LOW);
        } else {
            IOConfigGPIOAF(IOGetByTag(spi->sck), SPI_IO_AF_SCK_CFG_HIGH, spi->sckAF);
            LL_SPI_SetClockPhase(instance, LL_SPI_PHASE_2EDGE);
            LL_SPI_SetClockPolarity(instance, LL_SPI_POLARITY_HIGH);
        }
        bus->busType_u.spi.leadingEdge = dev->busType_u.spi.leadingEdge;
    }

    LL_SPI_Enable(instance);

    // Scan the segment list for DMA safety (cache alignment, DTCM region).
    for (busSegment_t *checkSegment = (busSegment_t *)bus->curSegment; checkSegment->len; checkSegment++) {
        if ((checkSegment->u.buffers.rxData) && (bus->dmaRx == (dmaChannelDescriptor_t *)NULL)) {
            dmaSafe = false;
            break;
        }
#ifdef STM32F7
        // Non-DTCM Rx buffers must be cache-line aligned for DMA on F7.
        if ((checkSegment->u.buffers.rxData) && !IS_DTCM(checkSegment->u.buffers.rxData) &&
            (((uint32_t)checkSegment->u.buffers.rxData & 31) || (checkSegment->len & 31))) {
            dmaSafe = false;
            break;
        }
#endif
        segmentCount++;
        xferLen += checkSegment->len;
    }

    // Use DMA if available and safe (dead path until spiInitBusDMA sets bus->useDMA, Stage M.3.d).
    if (bus->useDMA && dmaSafe && ((segmentCount > 1) ||
                                    (xferLen >= SPI_DMA_THRESHOLD) ||
                                    !bus->curSegment[segmentCount].negateCS)) {
        spiInternalInitStream(dev, false);
        IOLo(dev->busType_u.spi.csnPin);
        spiInternalStartDMA(dev);
    } else {
        busSegment_t *lastSegment = NULL;
        bool segmentComplete;

        while (bus->curSegment->len) {
            if (!lastSegment || lastSegment->negateCS) {
                IOLo(dev->busType_u.spi.csnPin);
            }

            spiTransfer(instance,
                        bus->curSegment->u.buffers.txData,
                        bus->curSegment->u.buffers.rxData,
                        bus->curSegment->len);

            if (bus->curSegment->negateCS) {
                IOHi(dev->busType_u.spi.csnPin);
            }

            segmentComplete = true;
            if (bus->curSegment->callback) {
                switch (bus->curSegment->callback(dev->callbackArg)) {
                case BUS_BUSY:
                    segmentComplete = false;
                    break;
                case BUS_ABORT:
                    bus->curSegment = (busSegment_t *)BUS_SPI_FREE;
                    return;
                case BUS_READY:
                default:
                    break;
                }
            }
            if (segmentComplete) {
                lastSegment = (busSegment_t *)bus->curSegment;
                bus->curSegment++;
            }
        }

        if (bus->curSegment->u.link.dev) {
            busSegment_t *endSegment = (busSegment_t *)bus->curSegment;
            const extDevice_t *nextDev = endSegment->u.link.dev;
            busSegment_t *nextSegments = (busSegment_t *)endSegment->u.link.segments;
            bus->curSegment = nextSegments;
            endSegment->u.link.dev = NULL;
            endSegment->u.link.segments = NULL;
            spiSequenceStart(nextDev);
        } else {
            bus->curSegment = (busSegment_t *)BUS_SPI_FREE;
        }
    }
}

// Platform-internal DMA functions — stubs until Stage M.3.d (dma_reqmap + spiInitBusDMA).
void spiInternalInitStream(const extDevice_t *dev, bool preInit) { UNUSED(dev); UNUSED(preInit); }
void spiInternalStartDMA(const extDevice_t *dev) { UNUSED(dev); }
void spiInternalStopDMA(const extDevice_t *dev) { UNUSED(dev); }
void spiInternalResetStream(dmaChannelDescriptor_t *descriptor) { UNUSED(descriptor); }
void spiInternalResetDescriptors(busDevice_t *bus) { UNUSED(bus); }

void spiSetDivisor(SPI_TypeDef *instance, uint16_t divisor) {
#if !(defined(STM32F1) || defined(STM32F3))
    // SPI2 and SPI3 are on APB1/AHB1 which PCLK is half that of APB2/AHB2.
    if (instance == SPI2 || instance == SPI3) {
        divisor /= 2; // Safe for divisor == 0 or 1
    }
#endif
    LL_SPI_Disable(instance);
    LL_SPI_SetBaudRatePrescaler(instance, divisor ? (ffs(divisor | 0x100) - 2) << SPI_CR1_BR_Pos : 0);
    LL_SPI_Enable(instance);
}
#endif
