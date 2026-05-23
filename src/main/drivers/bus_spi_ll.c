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

#if defined(STM32H7)
// H7 DTCM is 128 KB (0x20000000–0x2001FFFF); DMA1/2 cannot access it
#define IS_DTCM(p) (((uint32_t)(p) & 0xfffe0000) == 0x20000000)
// Linker-exported write-through DMA RAM region; buffers here need no cache maintenance
extern uint8_t _dmaram_start__;
extern uint8_t _dmaram_end__;
#define IS_DMARAM(p) ((uint8_t *)(p) >= &_dmaram_start__ && (uint8_t *)(p) < &_dmaram_end__)
#elif defined(STM32F7)
// F7 DTCM is 64 KB (0x20000000–0x2000FFFF)
#define IS_DTCM(p) (((uint32_t)(p) & 0xffff0000) == 0x20000000)
#endif

#if defined(STM32H7)
#define SPI_CR1_BR_Pos                      SPI_CFG1_MBR_Pos
#define SPI_RXFIFO_THRESHOLD_QF             LL_SPI_FIFO_TH_01DATA
#define LL_SPI_SetRxFIFOThreshold(spi, t)   LL_SPI_SetFIFOThreshold(spi, t)
#define LL_SPI_IsActiveFlag_TXE(x)          LL_SPI_IsActiveFlag_TXP(x)
#define LL_SPI_IsActiveFlag_RXNE(x)         LL_SPI_IsActiveFlag_RXP(x)
#define LL_SPI_IsActiveFlag_BSY(x)          (!LL_SPI_IsActiveFlag_TXC(x))
#define LL_SPI_GetTxFIFOLevel(x)            (LL_SPI_IsActiveFlag_TXP(x) ? 0 : 1)
#define LL_SPI_TX_FIFO_EMPTY                0
#endif

static uint32_t spiDivisorToBRbits(SPI_TypeDef *instance, uint16_t divisor)
{
#if defined(STM32H7)
    // On H7 all SPI buses are derived from the same kernel clock — no halving for SPI2/SPI3.
    UNUSED(instance);
#elif !(defined(STM32F1) || defined(STM32F3))
    // On F4/F7 SPI2/SPI3 are on APB1 (half the APB2 rate), so halve divisor to compensate.
    if (instance == SPI2 || instance == SPI3) {
        divisor /= 2;
    }
#else
    UNUSED(instance);
#endif
    // BF parity: clamp to valid range — divisor=1 after /=2 gives prescaler 256 via ffs(-1)<<3
    divisor = (divisor < 2) ? 2 : divisor;
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
    RCC_ResetCmd(spi->rcc, DISABLE);
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
#if defined(STM32H7)
    LL_SPI_EnableGPIOControl(spi->dev);
#endif
    LL_SPI_Init(spi->dev, &init);
#if !defined(STM32H7)
    // H7: leave SPI disabled after init; Enable/StartMasterTransfer happen per-transfer in spiTransfer.
    LL_SPI_Enable(spi->dev);
#endif
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
#if defined(STM32H7)
    LL_SPI_SetTransferSize(instance, len);
    LL_SPI_Enable(instance);
    LL_SPI_StartMasterTransfer(instance);
    int spiTimeout;
    while (len) {
        spiTimeout = 1000;
        while (!LL_SPI_IsActiveFlag_TXP(instance)) {
            if ((spiTimeout--) == 0) {
                return spiTimeoutUserCallback(instance);
            }
        }
        uint8_t b = txData ? *(txData++) : 0xFF;
        LL_SPI_TransmitData8(instance, b);
        spiTimeout = 1000; // reuse declared above
        while (!LL_SPI_IsActiveFlag_RXP(instance)) {
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
    spiTimeout = 1000;
    while (!LL_SPI_IsActiveFlag_EOT(instance)) {
        if ((spiTimeout--) == 0) {
            return spiTimeoutUserCallback(instance);
        }
    }
    LL_SPI_ClearFlag_TXTF(instance);
    LL_SPI_Disable(instance);
#else
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
#endif
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

#if !defined(STM32H7)
    LL_SPI_Disable(instance);
#endif

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

#if !defined(STM32H7)
    LL_SPI_Enable(instance);
#endif

    // Scan the segment list for DMA safety (cache alignment, DTCM region).
    for (busSegment_t *checkSegment = (busSegment_t *)bus->curSegment; checkSegment->len; checkSegment++) {
        if ((checkSegment->u.buffers.rxData) && (bus->dmaRx == (dmaChannelDescriptor_t *)NULL)) {
            dmaSafe = false;
            break;
        }
#if defined(STM32H7)
        // DMA1/2 cannot access DTCM on H7 — fall back to polled if buffer is there.
        if ((checkSegment->u.buffers.rxData) && IS_DTCM(checkSegment->u.buffers.rxData)) {
            dmaSafe = false;
            break;
        }
        if ((checkSegment->u.buffers.txData) && IS_DTCM(checkSegment->u.buffers.txData)) {
            dmaSafe = false;
            break;
        }
        // Rx buffers outside write-through dmaram must be 32-byte cache-line aligned.
        if ((checkSegment->u.buffers.rxData) && !IS_DMARAM(checkSegment->u.buffers.rxData) &&
            (((uint32_t)checkSegment->u.buffers.rxData & 31) || (checkSegment->len & 31))) {
            dmaSafe = false;
            break;
        }
#elif defined(STM32F7)
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

#if defined(STM32F7) || defined(STM32H7)
#define CACHE_LINE_SIZE  32
#define CACHE_LINE_MASK  (CACHE_LINE_SIZE - 1)
#endif

void spiInternalResetDescriptors(busDevice_t *bus)
{
    LL_DMA_InitTypeDef *initTx = bus->initTx;

    LL_DMA_StructInit(initTx);
#if defined(STM32H7)
    initTx->PeriphRequest = bus->dmaTx->channel;
#else
    initTx->Channel = bus->dmaTx->channel;
#endif
    initTx->Mode = LL_DMA_MODE_NORMAL;
    initTx->Direction = LL_DMA_DIRECTION_MEMORY_TO_PERIPH;
#if defined(STM32H7)
    initTx->PeriphOrM2MSrcAddress = (uint32_t)&bus->busType_u.spi.instance->TXDR;
#else
    initTx->PeriphOrM2MSrcAddress = (uint32_t)&bus->busType_u.spi.instance->DR;
#endif
    initTx->Priority = LL_DMA_PRIORITY_LOW;
    initTx->PeriphOrM2MSrcIncMode = LL_DMA_PERIPH_NOINCREMENT;
    initTx->PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_BYTE;
    initTx->MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_BYTE;

    if (bus->dmaRx) {
        LL_DMA_InitTypeDef *initRx = bus->initRx;

        LL_DMA_StructInit(initRx);
#if defined(STM32H7)
        initRx->PeriphRequest = bus->dmaRx->channel;
#else
        initRx->Channel = bus->dmaRx->channel;
#endif
        initRx->Mode = LL_DMA_MODE_NORMAL;
        initRx->Direction = LL_DMA_DIRECTION_PERIPH_TO_MEMORY;
#if defined(STM32H7)
        initRx->PeriphOrM2MSrcAddress = (uint32_t)&bus->busType_u.spi.instance->RXDR;
#else
        initRx->PeriphOrM2MSrcAddress = (uint32_t)&bus->busType_u.spi.instance->DR;
#endif
        initRx->Priority = LL_DMA_PRIORITY_LOW;
        initRx->PeriphOrM2MSrcIncMode = LL_DMA_PERIPH_NOINCREMENT;
        initRx->PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_BYTE;
    }
}

void spiInternalResetStream(dmaChannelDescriptor_t *descriptor)
{
    LL_DMA_DisableStream(descriptor->dma, descriptor->stream);
    while (LL_DMA_IsEnabledStream(descriptor->dma, descriptor->stream));
    DMA_CLEAR_FLAG(descriptor, DMA_IT_HTIF | DMA_IT_TEIF | DMA_IT_TCIF);
}

void spiInternalInitStream(const extDevice_t *dev, bool preInit)
{
    static DMA_DATA uint8_t dummyTxByte = 0xff;
    static DMA_DATA_ZERO_INIT uint8_t dummyRxByte;
    busDevice_t *bus = dev->bus;

    busSegment_t *segment = (busSegment_t *)bus->curSegment;

    if (preInit) {
        segment++;
        if (segment->len == 0) {
            return;
        }
    }

    int len = segment->len;
    uint8_t *txData = segment->u.buffers.txData;
    LL_DMA_InitTypeDef *initTx = bus->initTx;

    if (txData) {
#ifdef __DCACHE_PRESENT
#if defined(STM32H7)
        if (!IS_DMARAM(txData)) {
#else
        if (!IS_DTCM(txData)) {
#endif
            SCB_CleanDCache_by_Addr(
                (uint32_t *)((uint32_t)txData & ~CACHE_LINE_MASK),
                (((uint32_t)txData & CACHE_LINE_MASK) + len - 1 + CACHE_LINE_SIZE) & ~CACHE_LINE_MASK);
        }
#endif
        initTx->MemoryOrM2MDstAddress = (uint32_t)txData;
        initTx->MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT;
    } else {
        initTx->MemoryOrM2MDstAddress = (uint32_t)&dummyTxByte;
        initTx->MemoryOrM2MDstIncMode = LL_DMA_MEMORY_NOINCREMENT;
    }
    initTx->NbData = len;

    if (bus->dmaRx) {
        uint8_t *rxData = segment->u.buffers.rxData;
        LL_DMA_InitTypeDef *initRx = bus->initRx;

        if (rxData) {
#ifdef __DCACHE_PRESENT
#if defined(STM32H7)
            if (!IS_DMARAM(rxData)) {
#else
            if (!IS_DTCM(rxData)) {
#endif
                SCB_CleanInvalidateDCache_by_Addr(
                    (uint32_t *)((uint32_t)rxData & ~CACHE_LINE_MASK),
                    (((uint32_t)rxData & CACHE_LINE_MASK) + len - 1 + CACHE_LINE_SIZE) & ~CACHE_LINE_MASK);
            }
#endif
            initRx->MemoryOrM2MDstAddress = (uint32_t)rxData;
            initRx->MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT;
        } else {
            initRx->MemoryOrM2MDstAddress = (uint32_t)&dummyRxByte;
            initRx->MemoryOrM2MDstIncMode = LL_DMA_MEMORY_NOINCREMENT;
        }
        initRx->NbData = len;
    }
}

void spiInternalStartDMA(const extDevice_t *dev)
{
    busDevice_t *bus = dev->bus;
    dmaChannelDescriptor_t *dmaTx = bus->dmaTx;
    dmaChannelDescriptor_t *dmaRx = bus->dmaRx;

    if (dmaRx) {
        dmaRx->userParam = (uint32_t)dev;

        DMA_CLEAR_FLAG(dmaTx, DMA_IT_HTIF | DMA_IT_TEIF | DMA_IT_TCIF);
        DMA_CLEAR_FLAG(dmaRx, DMA_IT_HTIF | DMA_IT_TEIF | DMA_IT_TCIF);

        DMA_Stream_TypeDef *streamRegsTx = (DMA_Stream_TypeDef *)dmaTx->ref;
        DMA_Stream_TypeDef *streamRegsRx = (DMA_Stream_TypeDef *)dmaRx->ref;

        LL_DMA_WriteReg(streamRegsTx, CR, 0U);
        LL_DMA_WriteReg(streamRegsRx, CR, 0U);

        /* Use Rx TC interrupt — fires after SPI operation completes, unlike Tx TC
         * which fires when Tx FIFO empties while the SPI operation is still in progress.
         */
        LL_EX_DMA_EnableIT_TC(streamRegsRx);

        LL_DMA_Init(dmaTx->dma, dmaTx->stream, bus->initTx);
        LL_DMA_Init(dmaRx->dma, dmaRx->stream, bus->initRx);

#if defined(STM32H7)
        LL_SPI_SetTransferSize(dev->bus->busType_u.spi.instance, bus->curSegment->len);
        LL_DMA_EnableStream(dmaTx->dma, dmaTx->stream);
        LL_DMA_EnableStream(dmaRx->dma, dmaRx->stream);
        SET_BIT(dev->bus->busType_u.spi.instance->CFG1, SPI_CFG1_RXDMAEN | SPI_CFG1_TXDMAEN);
        LL_SPI_Enable(dev->bus->busType_u.spi.instance);
        LL_SPI_StartMasterTransfer(dev->bus->busType_u.spi.instance);
#else
        LL_DMA_EnableStream(dmaTx->dma, dmaTx->stream);
        LL_DMA_EnableStream(dmaRx->dma, dmaRx->stream);
        SET_BIT(dev->bus->busType_u.spi.instance->CR2, SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN);
#endif
    } else {
        DMA_Stream_TypeDef *streamRegsTx = (DMA_Stream_TypeDef *)dmaTx->ref;

        dmaTx->userParam = (uint32_t)dev;
        DMA_CLEAR_FLAG(dmaTx, DMA_IT_HTIF | DMA_IT_TEIF | DMA_IT_TCIF);
        LL_DMA_WriteReg(streamRegsTx, CR, 0U);
        LL_EX_DMA_EnableIT_TC(streamRegsTx);
        LL_DMA_Init(dmaTx->dma, dmaTx->stream, bus->initTx);
#if defined(STM32H7)
        LL_SPI_SetTransferSize(dev->bus->busType_u.spi.instance, bus->curSegment->len);
#endif
        LL_DMA_EnableStream(dmaTx->dma, dmaTx->stream);
#if defined(STM32H7)
        SET_BIT(dev->bus->busType_u.spi.instance->CFG1, SPI_CFG1_TXDMAEN);
        LL_SPI_Enable(dev->bus->busType_u.spi.instance);
        LL_SPI_StartMasterTransfer(dev->bus->busType_u.spi.instance);
#else
        SET_BIT(dev->bus->busType_u.spi.instance->CR2, SPI_CR2_TXDMAEN);
#endif
    }
}

void spiInternalStopDMA(const extDevice_t *dev)
{
    busDevice_t *bus = dev->bus;
    dmaChannelDescriptor_t *dmaTx = bus->dmaTx;
    dmaChannelDescriptor_t *dmaRx = bus->dmaRx;
    SPI_TypeDef *instance = bus->busType_u.spi.instance;

    if (dmaRx) {
        LL_DMA_DisableStream(dmaRx->dma, dmaRx->stream);
        LL_DMA_DisableStream(dmaTx->dma, dmaTx->stream);
        DMA_CLEAR_FLAG(dmaRx, DMA_IT_HTIF | DMA_IT_TEIF | DMA_IT_TCIF);
        LL_SPI_DisableDMAReq_TX(instance);
        LL_SPI_DisableDMAReq_RX(instance);
#if defined(STM32H7)
        LL_SPI_ClearFlag_TXTF(instance);
        LL_SPI_Disable(instance);
#endif
    } else {
        while (LL_SPI_IsActiveFlag_BSY(instance));
        while (LL_SPI_IsActiveFlag_RXNE(instance)) {
#if defined(STM32H7)
            (void)instance->RXDR;
#else
            (void)instance->DR;
#endif
        }
        LL_DMA_DisableStream(dmaTx->dma, dmaTx->stream);
        DMA_CLEAR_FLAG(dmaTx, DMA_IT_HTIF | DMA_IT_TEIF | DMA_IT_TCIF);
        LL_SPI_DisableDMAReq_TX(instance);
#if defined(STM32H7)
        LL_SPI_ClearFlag_TXTF(instance);
        LL_SPI_Disable(instance);
#endif
    }
}

void spiSetDivisor(SPI_TypeDef *instance, uint16_t divisor) {
    LL_SPI_Disable(instance);
    LL_SPI_SetBaudRatePrescaler(instance, spiDivisorToBRbits(instance, divisor));
    LL_SPI_Enable(instance);
}
#endif
