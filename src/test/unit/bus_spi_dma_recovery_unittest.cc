/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */


#include <cstring>

extern "C" {

#include "platform.h"

#include "drivers/bus.h"
#include "drivers/bus_spi.h"
#include "drivers/bus_spi_impl.h"
#include "drivers/dma.h"
#include "drivers/dma_reqmap.h"
#include "drivers/nvic.h"
#include "drivers/io.h"

#include "pg/bus_spi.h"
#include "pg/pg_ids.h"

PG_REGISTER_ARRAY(spiPinConfig_t, SPIDEV_COUNT, spiPinConfig, PG_SPI_PIN_CONFIG, 2);

static int abortDmaCalls;
static int sequenceStartCalls;
static const extDevice_t *lastStartedDev;
static IO_t lastIoHi;
static dmaCallbackHandlerFuncPtr rxHandler;
static dmaCallbackHandlerFuncPtr txHandler;
static dmaIdentifier_e rxHandlerId;
static dmaIdentifier_e txHandlerId;

}

#include "unittest_macros.h"
#include "gtest/gtest.h"

namespace {

const IO_t gyroCs = (IO_t)1;
const IO_t otherCs = (IO_t)2;

SPI_TypeDef fakeSpi1;
DMA_TypeDef fakeDma;
int txMarker;
int rxMarker;
dmaChannelDescriptor_t txDescriptor;
dmaChannelDescriptor_t rxDescriptor;

class SpiDmaRecoveryTest : public ::testing::Test {
protected:
    extDevice_t dev;
    busSegment_t segments[2];
    extDevice_t other;
    uint8_t txBuf[16];
    uint8_t rxBuf[16];
    busSegment_t otherSegments[2];

    void SetUp() override
    {
        abortDmaCalls = 0;
        sequenceStartCalls = 0;
        lastStartedDev = nullptr;
        lastIoHi = nullptr;
        rxHandler = nullptr;
        txHandler = nullptr;

        memset(&fakeSpi1, 0, sizeof(fakeSpi1));
        memset(&fakeDma, 0, sizeof(fakeDma));
        memset(&txDescriptor, 0, sizeof(txDescriptor));
        memset(&rxDescriptor, 0, sizeof(rxDescriptor));
        txDescriptor.dma = &fakeDma;
        rxDescriptor.dma = &fakeDma;
        for (int device = 0; device < SPIDEV_COUNT; device++) {
            busDevice_t *bus = spiBusByDevice(static_cast<SPIDevice>(device));
            memset(bus, 0, sizeof(*bus));
            memset(&spiDevice[device], 0, sizeof(spiDevice[device]));
        }
        spiDevice[SPIDEV_1].dev = &fakeSpi1;

        memset(&dev, 0, sizeof(dev));
        ASSERT_TRUE(spiSetBusInstance(&dev, SPI_DEV_TO_CFG(SPIDEV_1)));
        dev.busType_u.spi.csnPin = gyroCs;
        segments[0] = {.u = {.buffers = {txBuf, rxBuf}}, .len = 15, .negateCS = true, .callback = nullptr};
        segments[1] = {.u = {.link = {nullptr, nullptr}}, .len = 0, .negateCS = true, .callback = nullptr};

        memset(&other, 0, sizeof(other));
        other.bus = dev.bus;
        other.busType_u.spi.csnPin = otherCs;
        otherSegments[0] = {.u = {.buffers = {txBuf, nullptr}}, .len = 256, .negateCS = true, .callback = nullptr};
        otherSegments[1] = {.u = {.link = {nullptr, nullptr}}, .len = 0, .negateCS = true, .callback = nullptr};
    }

    bool inFlight() const
    {
        return dev.bus->curSegment == segments;
    }
};

TEST_F(SpiDmaRecoveryTest, AbortOnIdleBusDoesNothing)
{
    spiAbortTransfer(&dev);

    EXPECT_EQ(0, abortDmaCalls);
    EXPECT_EQ(nullptr, lastIoHi);
    EXPECT_EQ(0, spiGetErrorCounter(SPI1));
}

TEST_F(SpiDmaRecoveryTest, AbortReleasesBusNegatesChipSelectAndCountsError)
{
    spiSequence(&dev, segments);
    ASSERT_TRUE(inFlight());

    spiAbortTransfer(&dev);

    EXPECT_EQ(1, abortDmaCalls);
    EXPECT_EQ(gyroCs, lastIoHi);
    EXPECT_FALSE(spiIsBusy(&dev));
    EXPECT_EQ(1, spiGetErrorCounter(SPI1));
}

TEST_F(SpiDmaRecoveryTest, AbortStartsTransferQueuedBehindIt)
{
    spiSequence(&dev, segments);
    spiSequence(&other, otherSegments);
    ASSERT_EQ(1, sequenceStartCalls);

    spiAbortTransfer(&dev);

    EXPECT_EQ(2, sequenceStartCalls);
    EXPECT_EQ(&other, lastStartedDev);
    EXPECT_EQ(otherSegments, (busSegment_t *)dev.bus->curSegment);
    EXPECT_EQ(nullptr, segments[1].u.link.dev);
    EXPECT_EQ(nullptr, segments[1].u.link.segments);
}

TEST_F(SpiDmaRecoveryTest, RxHandlerWithTransferErrorAbortsInsteadOfCompleting)
{
    spiInitBusDMA();
    ASSERT_NE(rxHandler, nullptr);
    rxDescriptor.userParam = (uintptr_t)&dev;
    spiSequence(&dev, segments);
    ASSERT_TRUE(inFlight());

    fakeDma.LISR = DMA_IT_TEIF << rxDescriptor.flagsShift;
    rxHandler(&rxDescriptor);

    EXPECT_EQ(1, abortDmaCalls);
    EXPECT_FALSE(spiIsBusy(&dev));
    EXPECT_EQ(1, spiGetErrorCounter(SPI1));
}

TEST_F(SpiDmaRecoveryTest, TxErrorHandlerAbortsOnlyWithTransferErrorFlag)
{
    spiInitBusDMA();
    ASSERT_NE(txHandler, nullptr);
    txDescriptor.userParam = (uintptr_t)&dev;
    spiSequence(&dev, segments);

    txHandler(&txDescriptor); // flag clear: ignored
    EXPECT_EQ(0, abortDmaCalls);
    EXPECT_TRUE(inFlight());

    fakeDma.LISR = DMA_IT_TEIF << txDescriptor.flagsShift;
    txHandler(&txDescriptor);
    EXPECT_EQ(1, abortDmaCalls);
    EXPECT_FALSE(spiIsBusy(&dev));
}

} // namespace

extern "C" {

void spiSequenceStart(const extDevice_t *dev) { sequenceStartCalls++; lastStartedDev = dev; }
void spiInternalAbortDMA(const extDevice_t *) { abortDmaCalls++; }
void spiInitDevice(SPIDevice) {}
void spiInternalInitStream(const extDevice_t *, bool) {}
void spiInternalStartDMA(const extDevice_t *) {}
void spiInternalStopDMA(const extDevice_t *) {}
void spiInternalResetStream(dmaChannelDescriptor_t *) {}
void spiInternalResetDescriptors(busDevice_t *) {}

const dmaChannelSpec_t *dmaGetChannelSpecByPeripheral(dmaPeripheral_e device, uint8_t, int8_t opt)
{
    static dmaChannelSpec_t txSpec = { 0, (dmaResource_t *)&txMarker, 0 };
    static dmaChannelSpec_t rxSpec = { 0, (dmaResource_t *)&rxMarker, 0 };
    if (opt != 0) {
        return NULL;
    }
    return device == DMA_PERIPH_SPI_SDO ? &txSpec : (device == DMA_PERIPH_SPI_SDI ? &rxSpec : NULL);
}

dmaIdentifier_e dmaGetIdentifier(const DMA_Stream_TypeDef *stream)
{
    if ((const void *)stream == (const void *)&txMarker) {
        return DMA1_ST0_HANDLER;
    }
    return (const void *)stream == (const void *)&rxMarker ? DMA1_ST1_HANDLER : DMA_NONE;
}

bool dmaAllocate(dmaIdentifier_e, resourceOwner_e, uint8_t) { return true; }

dmaChannelDescriptor_t *dmaGetDescriptorByIdentifier(const dmaIdentifier_e identifier)
{
    return identifier == DMA1_ST0_HANDLER ? &txDescriptor : &rxDescriptor;
}

void dmaEnable(dmaIdentifier_e) {}

void dmaSetHandler(dmaIdentifier_e identifier, dmaCallbackHandlerFuncPtr callback, uint32_t, uintptr_t)
{
    if (identifier == DMA1_ST1_HANDLER) {
        rxHandler = callback;
        rxHandlerId = identifier;
    } else {
        txHandler = callback;
        txHandlerId = identifier;
    }
}

void IOHi(IO_t io) { lastIoHi = io; }
void IOLo(IO_t) {}
bool IORead(IO_t) { return false; }
void IOInit(IO_t, resourceOwner_e, uint8_t) {}
void IOConfigGPIO(IO_t, ioConfig_t) {}
void spiSetDivisor(SPI_TypeDef *, uint16_t) {}
void spiPreinitCsByIO(IO_t) {}
void delay(uint32_t) {}
uint32_t millis(void) { return 0; }
bool busReadRegisterBuffer(const extDevice_t *, uint8_t, uint8_t *, uint8_t) { return false; }

} // extern "C"
