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

// STM32H7 counterpart to bus_spi_unittest.cc, which only ever stood up STM32F4.
// H7 has no USE_TX_IRQ_HANDLER (bus_spi.c:33-35), so Tx-only success discards dmaTx instead of falling back.

extern "C" {

#include "platform.h"

#if !defined(UNUSED)
#define UNUSED(x) (void)(x)
#endif

#include "drivers/bus.h"
#include "drivers/bus_spi.h"
#include "drivers/bus_spi_impl.h"
#include "drivers/dma.h"
#include "drivers/dma_reqmap.h"
#include "drivers/nvic.h"

static int dmaTxStreamMarker;
static int dmaRxStreamMarker;

static bool txSpecAvailable;
static bool rxSpecAvailable;
static bool txAllocSucceeds;
static bool rxAllocSucceeds;
static int dmaSetHandlerCallCount;
static dmaIdentifier_e lastHandlerIdentifier;

static dmaChannelSpec_t txSpec;
static dmaChannelSpec_t rxSpec;
static dmaChannelDescriptor_t txDescriptor;
static dmaChannelDescriptor_t rxDescriptor;

const dmaChannelSpec_t *dmaGetChannelSpecByPeripheral(dmaPeripheral_e device, uint8_t index, int8_t opt) {
    UNUSED(index);
    if (opt != 0) {
        return NULL;
    }
    if (device == DMA_PERIPH_SPI_SDO) {
        return txSpecAvailable ? &txSpec : NULL;
    }
    if (device == DMA_PERIPH_SPI_SDI) {
        return rxSpecAvailable ? &rxSpec : NULL;
    }
    return NULL;
}

dmaIdentifier_e dmaGetIdentifier(const DMA_Stream_TypeDef *stream) {
    if ((const void *)stream == (const void *)&dmaTxStreamMarker) {
        return DMA1_ST0_HANDLER;
    }
    if ((const void *)stream == (const void *)&dmaRxStreamMarker) {
        return DMA1_ST1_HANDLER;
    }
    return DMA_NONE;
}

bool dmaAllocate(dmaIdentifier_e identifier, resourceOwner_e owner, uint8_t resourceIndex) {
    UNUSED(owner);
    UNUSED(resourceIndex);
    if (identifier == DMA1_ST0_HANDLER) {
        return txAllocSucceeds;
    }
    if (identifier == DMA1_ST1_HANDLER) {
        return rxAllocSucceeds;
    }
    return false;
}

dmaChannelDescriptor_t *dmaGetDescriptorByIdentifier(const dmaIdentifier_e identifier) {
    if (identifier == DMA1_ST0_HANDLER) {
        return &txDescriptor;
    }
    if (identifier == DMA1_ST1_HANDLER) {
        return &rxDescriptor;
    }
    return NULL;
}

void dmaEnable(dmaIdentifier_e identifier) {
    UNUSED(identifier);
}

void dmaSetHandler(dmaIdentifier_e identifier, dmaCallbackHandlerFuncPtr callback, uint32_t priority, uintptr_t userParam) {
    UNUSED(callback);
    UNUSED(priority);
    UNUSED(userParam);
    dmaSetHandlerCallCount++;
    lastHandlerIdentifier = identifier;
}

void spiInternalResetStream(dmaChannelDescriptor_t *descriptor) {
    UNUSED(descriptor);
}

void spiInternalResetDescriptors(busDevice_t *bus) {
    UNUSED(bus);
}

void spiInternalInitStream(const extDevice_t *dev, bool preInit) {
    UNUSED(dev);
    UNUSED(preInit);
}

void spiInternalStartDMA(const extDevice_t *dev) {
    UNUSED(dev);
}

void spiInternalStopDMA(const extDevice_t *dev) {
    UNUSED(dev);
}

void spiInitDevice(SPIDevice device) {
    UNUSED(device);
}

void spiSequenceStart(const extDevice_t *dev) {
    UNUSED(dev);
}

void IOHi(IO_t io) {
    UNUSED(io);
}

void IOLo(IO_t io) {
    UNUSED(io);
}

// link-only: spiRxIrqHandler()'s F7/H7 cache-invalidate call, never reached by these tests.
void SCB_InvalidateDCache_by_Addr(uint32_t *addr, int32_t dsize) {
    UNUSED(addr);
    UNUSED(dsize);
}

}

#include "unittest_macros.h"
#include "gtest/gtest.h"

namespace {

SPI_TypeDef fakeSpi1;

void resetSpiTestState()
{
    memset(&fakeSpi1, 0, sizeof(fakeSpi1));

    for (int device = 0; device < SPIDEV_COUNT; device++) {
        busDevice_t *bus = spiBusByDevice(static_cast<SPIDevice>(device));
        memset(bus, 0, sizeof(*bus));
        memset(&spiDevice[device], 0, sizeof(spiDevice[device]));
    }
    spiDevice[SPIDEV_1].dev = &fakeSpi1;

    txSpecAvailable = true;
    rxSpecAvailable = true;
    txAllocSucceeds = true;
    rxAllocSucceeds = true;
    dmaSetHandlerCallCount = 0;
    lastHandlerIdentifier = DMA_NONE;

    txSpec = { 0, (dmaResource_t *)&dmaTxStreamMarker, 0 };
    rxSpec = { 0, (dmaResource_t *)&dmaRxStreamMarker, 0 };
    memset(&txDescriptor, 0, sizeof(txDescriptor));
    memset(&rxDescriptor, 0, sizeof(rxDescriptor));
}

} // namespace

TEST(BusSpiH7Unittest, FullDuplexEnablesDmaSameAsF4)
{
    resetSpiTestState();
    extDevice_t dev = {};
    ASSERT_TRUE(spiSetBusInstance(&dev, SPI_DEV_TO_CFG(SPIDEV_1)));

    spiInitBusDMA();

    busDevice_t *bus = spiBusByDevice(SPIDEV_1);
    EXPECT_TRUE(bus->useDMA);
    EXPECT_EQ(bus->dmaTx, &txDescriptor);
    EXPECT_EQ(bus->dmaRx, &rxDescriptor);
    EXPECT_EQ(dmaSetHandlerCallCount, 1);
    EXPECT_EQ(lastHandlerIdentifier, DMA1_ST1_HANDLER);
}

TEST(BusSpiH7Unittest, TxOnlySuccessDoesNotEnableDmaUnlikeF4F7)
{
    // H7 has no Tx-only fallback: bus stays polled and dmaTx is discarded, unlike F4/F7.
    resetSpiTestState();
    rxSpecAvailable = false;
    extDevice_t dev = {};
    ASSERT_TRUE(spiSetBusInstance(&dev, SPI_DEV_TO_CFG(SPIDEV_1)));

    spiInitBusDMA();

    busDevice_t *bus = spiBusByDevice(SPIDEV_1);
    EXPECT_FALSE(bus->useDMA);
    EXPECT_EQ(bus->dmaTx, nullptr); // discarded, even though dmaAllocate() succeeded for Tx
    EXPECT_EQ(bus->dmaRx, nullptr);
    EXPECT_EQ(dmaSetHandlerCallCount, 0);
}

TEST(BusSpiH7Unittest, AllocationFailureLeavesBusPolled)
{
    // adversarial: both channels rejected by the DMA allocator.
    resetSpiTestState();
    txAllocSucceeds = false;
    rxAllocSucceeds = false;
    extDevice_t dev = {};
    ASSERT_TRUE(spiSetBusInstance(&dev, SPI_DEV_TO_CFG(SPIDEV_1)));

    spiInitBusDMA();

    busDevice_t *bus = spiBusByDevice(SPIDEV_1);
    EXPECT_FALSE(bus->useDMA);
    EXPECT_EQ(bus->dmaTx, nullptr);
    EXPECT_EQ(bus->dmaRx, nullptr);
    EXPECT_EQ(dmaSetHandlerCallCount, 0);
}
