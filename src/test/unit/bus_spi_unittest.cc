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

#include <cstring>

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

// Stand-ins for real DMA_Stream_TypeDef addresses; only pointer identity matters here.
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

// Scripts spiInitBusDMA()'s DMA-registration boundary; the real implementation touches RCC/NVIC registers absent on host.
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

void dmaSetHandler(dmaIdentifier_e identifier, dmaCallbackHandlerFuncPtr callback, uint32_t priority, uint32_t userParam) {
    UNUSED(callback);
    UNUSED(priority);
    UNUSED(userParam);
    dmaSetHandlerCallCount++;
    lastHandlerIdentifier = identifier;
}

// spiInitBusDMA() calls these directly on its DMA-enabled paths; real versions touch LL/StdPeriph registers.
void spiInternalResetStream(dmaChannelDescriptor_t *descriptor) {
    UNUSED(descriptor);
}

void spiInternalResetDescriptors(busDevice_t *bus) {
    UNUSED(bus);
}

// Link-only stubs: unreachable from the tests below, needed to satisfy the rest of the TU.
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

}

#include "unittest_macros.h"
#include "gtest/gtest.h"

namespace {

SPI_TypeDef fakeSpi1;
SPI_TypeDef fakeSpi2;

// spiBusDevice[]/spiDevice[] are module statics that persist across TEST() bodies otherwise.
void resetSpiTestState()
{
    for (int device = 0; device < SPIDEV_COUNT; device++) {
        busDevice_t *bus = spiBusByDevice(static_cast<SPIDevice>(device));
        memset(bus, 0, sizeof(*bus));
        memset(&spiDevice[device], 0, sizeof(spiDevice[device]));
    }
    spiDevice[SPIDEV_1].dev = &fakeSpi1;
    spiDevice[SPIDEV_2].dev = &fakeSpi2;

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

// --- spiSetBusInstance(): first-registration vs. repeat-registration branches ---

TEST(BusSpiUnittest, SetBusInstanceFirstRegistrationInitializesBus)
{
    resetSpiTestState();

    extDevice_t dev = {};
    bool result = spiSetBusInstance(&dev, SPI_DEV_TO_CFG(SPIDEV_1));

    EXPECT_TRUE(result);
    ASSERT_NE(dev.bus, nullptr);
    EXPECT_EQ(dev.bus, spiBusByDevice(SPIDEV_1));
    EXPECT_EQ(dev.bus->busType, BUS_TYPE_SPI);
    EXPECT_EQ(dev.bus->busType_u.spi.instance, &fakeSpi1);
    EXPECT_EQ(dev.bus->deviceCount, 1);
    EXPECT_FALSE(dev.bus->useDMA); // enabled later by spiInitBusDMA, not at registration
    EXPECT_TRUE(dev.useDMA);       // per-device DMA opt-in defaults on
    EXPECT_EQ((uintptr_t)dev.bus->curSegment, (uintptr_t)BUS_SPI_FREE);
}

TEST(BusSpiUnittest, SetBusInstanceRepeatRegistrationIncrementsDeviceCount)
{
    resetSpiTestState();

    extDevice_t dev1 = {};
    extDevice_t dev2 = {};
    ASSERT_TRUE(spiSetBusInstance(&dev1, SPI_DEV_TO_CFG(SPIDEV_1)));
    bool result = spiSetBusInstance(&dev2, SPI_DEV_TO_CFG(SPIDEV_1));

    EXPECT_TRUE(result);
    EXPECT_EQ(dev1.bus, dev2.bus); // both devices share one busDevice_t
    EXPECT_EQ(dev2.bus->deviceCount, 2);
    EXPECT_EQ(dev2.bus->busType_u.spi.instance, &fakeSpi1); // unchanged, not re-initialized
}

TEST(BusSpiUnittest, SetBusInstanceRejectsInvalidOrAbsentDevice)
{
    resetSpiTestState();
    extDevice_t dev = {};

    EXPECT_FALSE(spiSetBusInstance(&dev, 0)); // 0 means "disabled" in CLI convention
    EXPECT_FALSE(spiSetBusInstance(&dev, SPIDEV_COUNT + 1)); // out of range

    spiDevice[SPIDEV_2].dev = NULL; // peripheral absent on this target
    EXPECT_FALSE(spiSetBusInstance(&dev, SPI_DEV_TO_CFG(SPIDEV_2)));
    EXPECT_EQ(dev.bus, nullptr); // rejected before any bookkeeping runs
}

// --- spiInitBusDMA(): full / TX-only / failed allocation paths ---

TEST(BusSpiUnittest, InitBusDmaFullDuplexEnablesDmaOnBothChannels)
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
    EXPECT_EQ(lastHandlerIdentifier, DMA1_ST1_HANDLER); // Rx TC handler, not Tx
}

TEST(BusSpiUnittest, InitBusDmaFallsBackToTxOnlyWhenNoRxChannel)
{
    resetSpiTestState();
    rxSpecAvailable = false; // bus has a Tx DMA stream but no Rx stream
    extDevice_t dev = {};
    ASSERT_TRUE(spiSetBusInstance(&dev, SPI_DEV_TO_CFG(SPIDEV_1)));

    spiInitBusDMA();

    busDevice_t *bus = spiBusByDevice(SPIDEV_1);
    EXPECT_TRUE(bus->useDMA);
    EXPECT_EQ(bus->dmaTx, &txDescriptor);
    EXPECT_EQ(bus->dmaRx, nullptr);
    EXPECT_EQ(dmaSetHandlerCallCount, 1);
    EXPECT_EQ(lastHandlerIdentifier, DMA1_ST0_HANDLER); // Tx handler used instead
}

TEST(BusSpiUnittest, InitBusDmaAllocationFailureLeavesBusPolled)
{
    resetSpiTestState();
    txAllocSucceeds = false; // adversarial: both channels rejected by the DMA allocator
    rxAllocSucceeds = false;
    extDevice_t dev = {};
    ASSERT_TRUE(spiSetBusInstance(&dev, SPI_DEV_TO_CFG(SPIDEV_1)));

    spiInitBusDMA();

    busDevice_t *bus = spiBusByDevice(SPIDEV_1);
    EXPECT_FALSE(bus->useDMA); // stays in the polled path spiSetBusInstance left it in
    EXPECT_EQ(bus->dmaTx, nullptr);
    EXPECT_EQ(bus->dmaRx, nullptr);
    EXPECT_EQ(dmaSetHandlerCallCount, 0);
}

TEST(BusSpiUnittest, InitBusDmaSkipsBusesNotConfiguredForSpi)
{
    resetSpiTestState();
    // No spiSetBusInstance() call for any device — busType stays BUS_TYPE_NONE.

    spiInitBusDMA();

    for (int device = 0; device < SPIDEV_COUNT; device++) {
        busDevice_t *bus = spiBusByDevice(static_cast<SPIDevice>(device));
        EXPECT_FALSE(bus->useDMA);
    }
    EXPECT_EQ(dmaSetHandlerCallCount, 0);
}
