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

#include "pg/bus_spi.h"
#include "pg/pg_ids.h"

PG_REGISTER_ARRAY(spiPinConfig_t, SPIDEV_COUNT, spiPinConfig, PG_SPI_PIN_CONFIG, 2);

// Stand-ins for real DMA_Stream_TypeDef addresses; only pointer identity matters here.
static int dmaTxStreamMarker;
static int dmaRxStreamMarker;
static int dmaTxStreamMarker1; // second table option per direction (F4/F7 rows list two)
static int dmaRxStreamMarker1;

static bool txSpecAvailable;
static bool rxSpecAvailable;
static bool txSpec1Available;
static bool rxSpec1Available;
static bool txAllocSucceeds;
static bool rxAllocSucceeds;
static bool txAlloc1Succeeds;
static bool rxAlloc1Succeeds;
static int txProbeCount;
static int rxProbeCount;
static int8_t txProbes[8];
static int8_t rxProbes[8];
static int dmaSetHandlerCallCount;
static dmaIdentifier_e lastHandlerIdentifier;
static dmaCallbackHandlerFuncPtr lastHandlerCallback;

static dmaChannelSpec_t txSpec;
static dmaChannelSpec_t rxSpec;
static dmaChannelSpec_t txSpec1;
static dmaChannelSpec_t rxSpec1;
static dmaChannelDescriptor_t txDescriptor;
static dmaChannelDescriptor_t rxDescriptor;
static dmaChannelDescriptor_t txDescriptor1;
static dmaChannelDescriptor_t rxDescriptor1;

// Scripts spiInitBusDMA()'s DMA-registration boundary; the real implementation touches RCC/NVIC registers absent on host.
const dmaChannelSpec_t *dmaGetChannelSpecByPeripheral(dmaPeripheral_e device, uint8_t index, int8_t opt) {
    UNUSED(index);
    if (device == DMA_PERIPH_SPI_SDO && txProbeCount < 8) {
        txProbes[txProbeCount++] = opt;
    }
    if (device == DMA_PERIPH_SPI_SDI && rxProbeCount < 8) {
        rxProbes[rxProbeCount++] = opt;
    }
    if (opt != 0 && opt != 1) {
        return NULL; // like the real table: an option outside the row is absent
    }
    if (device == DMA_PERIPH_SPI_SDO) {
        if (opt == 0) {
            return txSpecAvailable ? &txSpec : NULL;
        }
        return txSpec1Available ? &txSpec1 : NULL;
    }
    if (device == DMA_PERIPH_SPI_SDI) {
        if (opt == 0) {
            return rxSpecAvailable ? &rxSpec : NULL;
        }
        return rxSpec1Available ? &rxSpec1 : NULL;
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
    if ((const void *)stream == (const void *)&dmaTxStreamMarker1) {
        return DMA1_ST2_HANDLER;
    }
    if ((const void *)stream == (const void *)&dmaRxStreamMarker1) {
        return DMA1_ST3_HANDLER;
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
    if (identifier == DMA1_ST2_HANDLER) {
        return txAlloc1Succeeds;
    }
    if (identifier == DMA1_ST3_HANDLER) {
        return rxAlloc1Succeeds;
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
    if (identifier == DMA1_ST2_HANDLER) {
        return &txDescriptor1;
    }
    if (identifier == DMA1_ST3_HANDLER) {
        return &rxDescriptor1;
    }
    return NULL;
}

void dmaEnable(dmaIdentifier_e identifier) {
    UNUSED(identifier);
}

void dmaSetHandler(dmaIdentifier_e identifier, dmaCallbackHandlerFuncPtr callback, uint32_t priority, uintptr_t userParam) {
    UNUSED(priority);
    UNUSED(userParam);
    dmaSetHandlerCallCount++;
    lastHandlerIdentifier = identifier;
    lastHandlerCallback = callback;
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

// spiBusDevice[]/spiDevice[]/fakeSpi{1,2} are module statics that persist across TEST() bodies otherwise.
void resetSpiTestState()
{
    memset(&fakeSpi1, 0, sizeof(fakeSpi1));
    memset(&fakeSpi2, 0, sizeof(fakeSpi2));

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
    txSpec1Available = false;
    rxSpec1Available = false;
    txAlloc1Succeeds = true;
    rxAlloc1Succeeds = true;
    txProbeCount = 0;
    rxProbeCount = 0;
    for (int device = 0; device < SPIDEV_COUNT; device++) {
        spiPinConfigMutable(device)->txDmaopt = DMA_OPT_UNUSED;
        spiPinConfigMutable(device)->rxDmaopt = DMA_OPT_UNUSED;
    }
    dmaSetHandlerCallCount = 0;
    lastHandlerIdentifier = DMA_NONE;
    lastHandlerCallback = NULL;

    txSpec = { 0, (dmaResource_t *)&dmaTxStreamMarker, 0 };
    rxSpec = { 0, (dmaResource_t *)&dmaRxStreamMarker, 0 };
    txSpec1 = { 0, (dmaResource_t *)&dmaTxStreamMarker1, 0 };
    rxSpec1 = { 0, (dmaResource_t *)&dmaRxStreamMarker1, 0 };
    memset(&txDescriptor, 0, sizeof(txDescriptor));
    memset(&rxDescriptor, 0, sizeof(rxDescriptor));
    memset(&txDescriptor1, 0, sizeof(txDescriptor1));
    memset(&rxDescriptor1, 0, sizeof(rxDescriptor1));
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

// --- DMA completion IRQ handlers: userParam device-pointer round-trip ---
// dmaSetHandler()'s registered callback is a static function in bus_spi.c, reachable
// here only via the pointer the mock above captures; both handlers reconstruct the
// extDevice_t* from descriptor->userParam, so a truncating field width would corrupt
// dev before bus->curSegment is ever read.

TEST(BusSpiUnittest, RxIrqHandlerPreservesDevicePointerThroughUserParam)
{
    resetSpiTestState();
    extDevice_t dev = {};
    ASSERT_TRUE(spiSetBusInstance(&dev, SPI_DEV_TO_CFG(SPIDEV_1)));
    spiInitBusDMA();
    ASSERT_EQ(dmaSetHandlerCallCount, 1);
    ASSERT_NE(lastHandlerCallback, nullptr);

    busSegment_t segments[2] = {}; // segments[0]: one-shot transfer; segments[1]: list terminator (len == 0)
    segments[0].len = 1;
    dev.bus->curSegment = segments;
    rxDescriptor.userParam = (uintptr_t)&dev;

    lastHandlerCallback(&rxDescriptor);

    EXPECT_EQ((uintptr_t)dev.bus->curSegment, (uintptr_t)BUS_SPI_FREE);
}

TEST(BusSpiUnittest, TxIrqHandlerPreservesDevicePointerThroughUserParam)
{
    resetSpiTestState();
    rxSpecAvailable = false; // forces the Tx-only path, which registers spiTxIrqHandler instead
    extDevice_t dev = {};
    ASSERT_TRUE(spiSetBusInstance(&dev, SPI_DEV_TO_CFG(SPIDEV_1)));
    spiInitBusDMA();
    ASSERT_EQ(dmaSetHandlerCallCount, 1);
    ASSERT_NE(lastHandlerCallback, nullptr);

    busSegment_t segments[2] = {};
    segments[0].len = 1;
    dev.bus->curSegment = segments;
    txDescriptor.userParam = (uintptr_t)&dev;

    lastHandlerCallback(&txDescriptor);

    EXPECT_EQ((uintptr_t)dev.bus->curSegment, (uintptr_t)BUS_SPI_FREE);
}

// --- spiDmaEnable(): per-device flag, independent of bus-level useDMA ---

TEST(BusSpiUnittest, DmaEnableSetsPerDeviceFlagIndependentOfBus)
{
    resetSpiTestState();
    extDevice_t dev = {};
    ASSERT_TRUE(spiSetBusInstance(&dev, SPI_DEV_TO_CFG(SPIDEV_1)));
    spiInitBusDMA();
    busDevice_t *bus = spiBusByDevice(SPIDEV_1);
    ASSERT_TRUE(bus->useDMA);

    spiDmaEnable(&dev, false);

    EXPECT_FALSE(dev.useDMA); // gates this device only
    EXPECT_TRUE(bus->useDMA); // other devices sharing the bus keep DMA

    spiDmaEnable(&dev, true);

    EXPECT_TRUE(dev.useDMA);
}

// --- spiInitBusDMA(): stored txDmaopt/rxDmaopt (pin-or-scan) ---
// Spec: -1 scans options 0..MAX-1 and the first that allocates wins; 0..MAX-1 probes that option only;
// anything else leaves that direction without DMA. The F4/F7 row has two options (MAX_PERIPHERAL_DMA_OPTIONS == 2).

static void initDevice1WithOptions(int8_t tx, int8_t rx)
{
    extDevice_t dev = {};
    ASSERT_TRUE(spiSetBusInstance(&dev, SPI_DEV_TO_CFG(SPIDEV_1)));
    spiPinConfigMutable(SPIDEV_1)->txDmaopt = tx;
    spiPinConfigMutable(SPIDEV_1)->rxDmaopt = rx;
    spiInitBusDMA();
}

TEST(BusSpiUnittest, UnsetOptionsScanAndStopAtFirstFreeOption)
{
    resetSpiTestState();
    txSpec1Available = true;
    rxSpec1Available = true;
    initDevice1WithOptions(DMA_OPT_UNUSED, DMA_OPT_UNUSED);

    EXPECT_EQ(txProbeCount, 1); // option 0 allocates, no further probe
    EXPECT_EQ(rxProbeCount, 1);
    EXPECT_EQ(txProbes[0], 0);
    EXPECT_EQ(rxProbes[0], 0);
    EXPECT_EQ(spiBusByDevice(SPIDEV_1)->dmaTx, &txDescriptor);
}

TEST(BusSpiUnittest, UnsetOptionsFallBackToSecondOptionWhenFirstIsClaimed)
{
    resetSpiTestState();
    txSpec1Available = true;
    txAllocSucceeds = false; // option 0 held elsewhere
    initDevice1WithOptions(DMA_OPT_UNUSED, DMA_OPT_UNUSED);

    EXPECT_EQ(txProbeCount, 2);
    EXPECT_EQ(txProbes[0], 0);
    EXPECT_EQ(txProbes[1], 1);
    EXPECT_EQ(spiBusByDevice(SPIDEV_1)->dmaTx, &txDescriptor1);
    EXPECT_TRUE(spiBusByDevice(SPIDEV_1)->useDMA);
}

TEST(BusSpiUnittest, PinnedTxOptionProbesOnlyThatOption)
{
    resetSpiTestState();
    txSpec1Available = true;
    initDevice1WithOptions(1, DMA_OPT_UNUSED);

    EXPECT_EQ(txProbeCount, 1);
    EXPECT_EQ(txProbes[0], 1);
    EXPECT_EQ(spiBusByDevice(SPIDEV_1)->dmaTx, &txDescriptor1); // option 1 even though option 0 was free
    EXPECT_EQ(spiBusByDevice(SPIDEV_1)->dmaRx, &rxDescriptor);  // Rx still auto
}

TEST(BusSpiUnittest, PinnedOptionHeldElsewhereDoesNotFallBackToOtherOption)
{
    resetSpiTestState();
    txSpec1Available = true;
    txAlloc1Succeeds = false; // pinned stream already owned; option 0 is free
    initDevice1WithOptions(1, DMA_OPT_UNUSED);

    EXPECT_EQ(txProbeCount, 1);
    EXPECT_EQ(txProbes[0], 1);
    busDevice_t *bus = spiBusByDevice(SPIDEV_1);
    EXPECT_FALSE(bus->useDMA); // no Tx stream: polled, option 0 never taken
    EXPECT_EQ(bus->dmaTx, nullptr);
}

TEST(BusSpiUnittest, PinnedRxOptionIsIndependentOfTx)
{
    resetSpiTestState();
    rxSpec1Available = true;
    initDevice1WithOptions(DMA_OPT_UNUSED, 1);

    EXPECT_EQ(txProbes[0], 0);
    EXPECT_EQ(rxProbeCount, 1);
    EXPECT_EQ(rxProbes[0], 1);
    EXPECT_EQ(spiBusByDevice(SPIDEV_1)->dmaTx, &txDescriptor);
    EXPECT_EQ(spiBusByDevice(SPIDEV_1)->dmaRx, &rxDescriptor1);
}

TEST(BusSpiUnittest, PinnedOptionOnlyAffectsItsOwnDevice)
{
    resetSpiTestState();
    txSpec1Available = true;
    extDevice_t dev1 = {};
    extDevice_t dev2 = {};
    ASSERT_TRUE(spiSetBusInstance(&dev1, SPI_DEV_TO_CFG(SPIDEV_1)));
    ASSERT_TRUE(spiSetBusInstance(&dev2, SPI_DEV_TO_CFG(SPIDEV_2)));
    spiPinConfigMutable(SPIDEV_2)->txDmaopt = 1;
    spiInitBusDMA();

    ASSERT_GE(txProbeCount, 2);
    EXPECT_EQ(txProbes[0], 0); // SPIDEV_1 unset: first probe is option 0
    EXPECT_EQ(txProbes[txProbeCount - 1], 1); // SPIDEV_2 pinned
}

TEST(BusSpiUnittest, OutOfRangePinnedOptionLeavesBusPolledWithoutProbing)
{
    // Adversarial: values a corrupted or hand-edited config could hold. None may allocate DMA,
    // wrap into a valid option, or probe past the table.
    const int8_t bad[] = { -2, -128, 2, 3, 16, 127 };
    for (size_t i = 0; i < sizeof(bad); i++) {
        resetSpiTestState();
        txSpec1Available = true;
        rxSpec1Available = true;
        initDevice1WithOptions(bad[i], bad[i]);

        busDevice_t *bus = spiBusByDevice(SPIDEV_1);
        EXPECT_FALSE(bus->useDMA) << "opt " << (int)bad[i];
        EXPECT_EQ(bus->dmaTx, nullptr) << "opt " << (int)bad[i];
        EXPECT_EQ(bus->dmaRx, nullptr) << "opt " << (int)bad[i];
        EXPECT_EQ(txProbeCount, 0) << "opt " << (int)bad[i];
        EXPECT_EQ(rxProbeCount, 0) << "opt " << (int)bad[i];
    }
}

TEST(BusSpiUnittest, OutOfRangeTxOptionKeepsValidRxOptionWorking)
{
    resetSpiTestState();
    initDevice1WithOptions(2, DMA_OPT_UNUSED);

    busDevice_t *bus = spiBusByDevice(SPIDEV_1);
    EXPECT_EQ(bus->dmaTx, nullptr);
    EXPECT_EQ(rxProbeCount, 1);
    EXPECT_FALSE(bus->useDMA); // Tx missing and Rx present: Rx alone is not used, and the Tx-only path needs Tx
}
