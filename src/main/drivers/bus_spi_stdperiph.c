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

// STM32F405 CCM SRAM (0x10000000–0x1000FFFF) is inaccessible by DMA1/2.
#define IS_CCM(p) (((uint32_t)(p) & 0xffff0000) == 0x10000000)

#include "drivers/bus.h"
#include "drivers/bus_spi.h"
#include "drivers/bus_spi_impl.h"
#include "drivers/dma.h"
#include "drivers/exti.h"
#include "drivers/io.h"
#include "drivers/rcc.h"

spiDevice_t spiDevice[SPIDEV_COUNT];

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
#if defined(STM32F3) || defined(STM32F4)
    IOConfigGPIOAF(IOGetByTag(spi->sck),  SPI_IO_AF_CFG, spi->af);
    IOConfigGPIOAF(IOGetByTag(spi->miso), SPI_IO_AF_CFG, spi->af);
    IOConfigGPIOAF(IOGetByTag(spi->mosi), SPI_IO_AF_CFG, spi->af);
#elif defined(STM32F10X)
    IOConfigGPIO(IOGetByTag(spi->sck), SPI_IO_AF_SCK_CFG);
    IOConfigGPIO(IOGetByTag(spi->miso), SPI_IO_AF_MISO_CFG);
    IOConfigGPIO(IOGetByTag(spi->mosi), SPI_IO_AF_MOSI_CFG);
#else
#error Undefined MCU architecture
#endif
    // Init SPI hardware
    SPI_I2S_DeInit(spi->dev);
    SPI_InitTypeDef spiInit;
    spiInit.SPI_Mode = SPI_Mode_Master;
    spiInit.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    spiInit.SPI_DataSize = SPI_DataSize_8b;
    spiInit.SPI_NSS = SPI_NSS_Soft;
    spiInit.SPI_FirstBit = SPI_FirstBit_MSB;
    spiInit.SPI_CRCPolynomial = 7;
    spiInit.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
    if (spi->leadingEdge) {
        spiInit.SPI_CPOL = SPI_CPOL_Low;
        spiInit.SPI_CPHA = SPI_CPHA_1Edge;
    } else {
        spiInit.SPI_CPOL = SPI_CPOL_High;
        spiInit.SPI_CPHA = SPI_CPHA_2Edge;
    }
#ifdef STM32F303xC
    // Configure for 8-bit reads.
    SPI_RxFIFOThresholdConfig(spi->dev, SPI_RxFIFOThreshold_QF);
#endif
    SPI_Init(spi->dev, &spiInit);
    SPI_Cmd(spi->dev, ENABLE);
}

// return uint8_t value or -1 when failure
uint8_t spiTransferByte(SPI_TypeDef *instance, uint8_t txByte) {
    uint16_t spiTimeout = 1000;
    while (SPI_I2S_GetFlagStatus(instance, SPI_I2S_FLAG_TXE) == RESET)
        if ((spiTimeout--) == 0)
            return spiTimeoutUserCallback(instance);
#ifdef STM32F303xC
    SPI_SendData8(instance, txByte);
#else
    SPI_I2S_SendData(instance, txByte);
#endif
    spiTimeout = 1000;
    while (SPI_I2S_GetFlagStatus(instance, SPI_I2S_FLAG_RXNE) == RESET)
        if ((spiTimeout--) == 0)
            return spiTimeoutUserCallback(instance);
#ifdef STM32F303xC
    return ((uint8_t)SPI_ReceiveData8(instance));
#else
    return ((uint8_t)SPI_I2S_ReceiveData(instance));
#endif
}

/**
 * Return true if the bus is currently in the middle of a transmission.
 */
bool spiIsBusBusy(SPI_TypeDef *instance) {
#ifdef STM32F303xC
    return SPI_GetTransmissionFIFOStatus(instance) != SPI_TransmissionFIFOStatus_Empty || SPI_I2S_GetFlagStatus(instance, SPI_I2S_FLAG_BSY) == SET;
#else
    return SPI_I2S_GetFlagStatus(instance, SPI_I2S_FLAG_TXE) == RESET || SPI_I2S_GetFlagStatus(instance, SPI_I2S_FLAG_BSY) == SET;
#endif
}

bool spiTransfer(SPI_TypeDef *instance, const uint8_t *txData, uint8_t *rxData, int len) {
    uint16_t spiTimeout = 1000;
    uint8_t b;
    instance->DR;
    while (len--) {
        b = txData ? *(txData++) : 0xFF;
        while (SPI_I2S_GetFlagStatus(instance, SPI_I2S_FLAG_TXE) == RESET) {
            if ((spiTimeout--) == 0)
                return spiTimeoutUserCallback(instance);
        }
#ifdef STM32F303xC
        SPI_SendData8(instance, b);
#else
        SPI_I2S_SendData(instance, b);
#endif
        spiTimeout = 1000;
        while (SPI_I2S_GetFlagStatus(instance, SPI_I2S_FLAG_RXNE) == RESET) {
            if ((spiTimeout--) == 0)
                return spiTimeoutUserCallback(instance);
        }
#ifdef STM32F303xC
        b = SPI_ReceiveData8(instance);
#else
        b = SPI_I2S_ReceiveData(instance);
#endif
        if (rxData)
            *(rxData++) = b;
    }
    return true;
}

#define SPI_DMA_THRESHOLD 8

// DMA transfer setup and start (BF 4.5-maintenance parity, stdperiph API).
// Handles per-device speed and polarity switching, then dispatches to DMA path
// (when bus->useDMA is enabled by spiInitBusDMA, Stage M.3.d) or polled path.
FAST_CODE void spiSequenceStart(const extDevice_t *dev)
{
    busDevice_t *bus = dev->bus;
    SPI_TypeDef *instance = bus->busType_u.spi.instance;
    bool dmaSafe = dev->useDMA;
    uint32_t xferLen = 0;
    uint32_t segmentCount = 0;

    bus->initSegment = true;

    SPI_Cmd(instance, DISABLE);

    if (dev->busType_u.spi.speed != bus->busType_u.spi.speed) {
#define BR_BITS ((BIT(5) | BIT(4) | BIT(3)))
        uint16_t divisor = dev->busType_u.spi.speed;
#if !(defined(STM32F1) || defined(STM32F3))
        if (instance == SPI2 || instance == SPI3) {
            divisor /= 2;
        }
#endif
        instance->CR1 = (instance->CR1 & ~BR_BITS) | (divisor ? ((ffs(divisor | 0x100) - 2) << 3) : 0);
#undef BR_BITS
        bus->busType_u.spi.speed = dev->busType_u.spi.speed;
    }

    if (dev->busType_u.spi.leadingEdge != bus->busType_u.spi.leadingEdge) {
        // F3/F4 have no SCK_CFG_LOW/HIGH GPIO variant — only CR1 CPOL/CPHA updated.
        instance->CR1 &= ~(SPI_CPOL_High | SPI_CPHA_2Edge);
        if (!dev->busType_u.spi.leadingEdge) {
            instance->CR1 |= SPI_CPOL_High | SPI_CPHA_2Edge;
        }
        bus->busType_u.spi.leadingEdge = dev->busType_u.spi.leadingEdge;
    }

    SPI_Cmd(instance, ENABLE);

    // Scan the segment list for DMA safety.
    // CCM SRAM is inaccessible by DMA on F4 — reject any buffer in that region.
    for (busSegment_t *checkSegment = (busSegment_t *)bus->curSegment; checkSegment->len; checkSegment++) {
        if (((checkSegment->u.buffers.rxData) && (IS_CCM(checkSegment->u.buffers.rxData) || (bus->dmaRx == (dmaChannelDescriptor_t *)NULL))) ||
            ((checkSegment->u.buffers.txData) && IS_CCM(checkSegment->u.buffers.txData))) {
            dmaSafe = false;
            break;
        }
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

#if defined(STM32F4)
void spiInternalResetDescriptors(busDevice_t *bus)
{
    DMA_InitTypeDef *initTx = bus->initTx;

    DMA_StructInit(initTx);
    initTx->DMA_Channel = bus->dmaTx->channel;
    initTx->DMA_DIR = DMA_DIR_MemoryToPeripheral;
    initTx->DMA_Mode = DMA_Mode_Normal;
    initTx->DMA_PeripheralBaseAddr = (uint32_t)&bus->busType_u.spi.instance->DR;
    initTx->DMA_Priority = DMA_Priority_Low;
    initTx->DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    initTx->DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    initTx->DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;

    if (bus->dmaRx) {
        DMA_InitTypeDef *initRx = bus->initRx;

        DMA_StructInit(initRx);
        initRx->DMA_Channel = bus->dmaRx->channel;
        initRx->DMA_DIR = DMA_DIR_PeripheralToMemory;
        initRx->DMA_Mode = DMA_Mode_Normal;
        initRx->DMA_PeripheralBaseAddr = (uint32_t)&bus->busType_u.spi.instance->DR;
        initRx->DMA_Priority = DMA_Priority_Low;
        initRx->DMA_PeripheralInc = DMA_PeripheralInc_Disable;
        initRx->DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    }
}

void spiInternalResetStream(dmaChannelDescriptor_t *descriptor)
{
    DMA_Stream_TypeDef *streamRegs = (DMA_Stream_TypeDef *)descriptor->ref;
    streamRegs->CR = 0U;
    DMA_CLEAR_FLAG(descriptor, DMA_IT_HTIF | DMA_IT_TEIF | DMA_IT_TCIF);
}

void spiInternalInitStream(const extDevice_t *dev, bool preInit)
{
    static uint8_t dummyTxByte = 0xff;
    static uint8_t dummyRxByte;
    busDevice_t *bus = dev->bus;

    volatile busSegment_t *segment = bus->curSegment;

    if (preInit) {
        segment++;
        if (segment->len == 0) {
            return;
        }
    }

    int len = segment->len;
    uint8_t *txData = segment->u.buffers.txData;
    DMA_InitTypeDef *initTx = bus->initTx;

    if (txData) {
        initTx->DMA_Memory0BaseAddr = (uint32_t)txData;
        initTx->DMA_MemoryInc = DMA_MemoryInc_Enable;
    } else {
        dummyTxByte = 0xff;
        initTx->DMA_Memory0BaseAddr = (uint32_t)&dummyTxByte;
        initTx->DMA_MemoryInc = DMA_MemoryInc_Disable;
    }
    initTx->DMA_BufferSize = len;

    if (bus->dmaRx) {
        uint8_t *rxData = segment->u.buffers.rxData;
        DMA_InitTypeDef *initRx = bus->initRx;

        if (rxData) {
            initRx->DMA_Memory0BaseAddr = (uint32_t)rxData;
            initRx->DMA_MemoryInc = DMA_MemoryInc_Enable;
        } else {
            initRx->DMA_Memory0BaseAddr = (uint32_t)&dummyRxByte;
            initRx->DMA_MemoryInc = DMA_MemoryInc_Disable;
        }
        /* Use 16-bit memory writes where possible to prevent atomic access issues on gyro data */
        if ((initRx->DMA_Memory0BaseAddr & 0x1) || (len & 0x1)) {
            initRx->DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
        } else {
            initRx->DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
        }
        initRx->DMA_BufferSize = len;
    }
}

void spiInternalStartDMA(const extDevice_t *dev)
{
    dmaChannelDescriptor_t *dmaTx = dev->bus->dmaTx;
    dmaChannelDescriptor_t *dmaRx = dev->bus->dmaRx;
    DMA_Stream_TypeDef *streamRegsTx = (DMA_Stream_TypeDef *)dmaTx->ref;

    if (dmaRx) {
        DMA_Stream_TypeDef *streamRegsRx = (DMA_Stream_TypeDef *)dmaRx->ref;

        dmaRx->userParam = (uint32_t)dev;

        DMA_CLEAR_FLAG(dmaTx, DMA_IT_HTIF | DMA_IT_TEIF | DMA_IT_TCIF);
        DMA_CLEAR_FLAG(dmaRx, DMA_IT_HTIF | DMA_IT_TEIF | DMA_IT_TCIF);

        streamRegsTx->CR = 0U;
        streamRegsRx->CR = 0U;

        /* Use Rx TC interrupt — fires after SPI operation completes, unlike Tx TC
         * which fires when Tx FIFO empties while the SPI operation is still in progress.
         */
        DMA_ITConfig(streamRegsRx, DMA_IT_TC, ENABLE);

        DMA_Init(streamRegsTx, dev->bus->initTx);
        DMA_Init(streamRegsRx, dev->bus->initRx);

        DMA_Cmd(streamRegsTx, ENABLE);
        DMA_Cmd(streamRegsRx, ENABLE);

        SPI_I2S_DMACmd(dev->bus->busType_u.spi.instance, SPI_I2S_DMAReq_Tx | SPI_I2S_DMAReq_Rx, ENABLE);
    } else {
        dmaTx->userParam = (uint32_t)dev;

        DMA_CLEAR_FLAG(dmaTx, DMA_IT_HTIF | DMA_IT_TEIF | DMA_IT_TCIF);

        streamRegsTx->CR = 0U;
        DMA_ITConfig(streamRegsTx, DMA_IT_TC, ENABLE);
        DMA_Init(streamRegsTx, dev->bus->initTx);
        DMA_Cmd(streamRegsTx, ENABLE);

        SPI_I2S_DMACmd(dev->bus->busType_u.spi.instance, SPI_I2S_DMAReq_Tx, ENABLE);
    }
}

void spiInternalStopDMA(const extDevice_t *dev)
{
    dmaChannelDescriptor_t *dmaTx = dev->bus->dmaTx;
    dmaChannelDescriptor_t *dmaRx = dev->bus->dmaRx;
    SPI_TypeDef *instance = dev->bus->busType_u.spi.instance;
    DMA_Stream_TypeDef *streamRegsTx = (DMA_Stream_TypeDef *)dmaTx->ref;

    if (dmaRx) {
        DMA_Stream_TypeDef *streamRegsRx = (DMA_Stream_TypeDef *)dmaRx->ref;

        streamRegsTx->CR = 0U;
        streamRegsRx->CR = 0U;

        SPI_I2S_DMACmd(instance, SPI_I2S_DMAReq_Tx | SPI_I2S_DMAReq_Rx, DISABLE);
    } else {
        while (SPI_I2S_GetFlagStatus(instance, SPI_I2S_FLAG_BSY));
        while (SPI_I2S_GetFlagStatus(instance, SPI_I2S_FLAG_RXNE)) {
            instance->DR;
        }
        streamRegsTx->CR = 0U;
        SPI_I2S_DMACmd(instance, SPI_I2S_DMAReq_Tx, DISABLE);
    }
}
#endif // STM32F4

void spiSetDivisor(SPI_TypeDef *instance, uint16_t divisor) {
#define BR_BITS ((BIT(5) | BIT(4) | BIT(3)))
#if !(defined(STM32F1) || defined(STM32F3))
    // SPI2 and SPI3 are on APB1/AHB1 which PCLK is half that of APB2/AHB2.
    if (instance == SPI2 || instance == SPI3) {
        divisor /= 2; // Safe for divisor == 0 or 1
    }
#endif
    SPI_Cmd(instance, DISABLE);
    const uint16_t tempRegister = (instance->CR1 & ~BR_BITS);
    instance->CR1 = tempRegister | (divisor ? ((ffs(divisor | 0x100) - 2) << 3) : 0);
    SPI_Cmd(instance, ENABLE);
#undef BR_BITS
}
#endif
