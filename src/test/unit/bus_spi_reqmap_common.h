/*
 * This file is part of EmuFlight.
 *
 * EmuFlight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * EmuFlight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with EmuFlight.  If not, see <http://www.gnu.org/licenses/>.
 */

// spiInitBusDMA() against the real dma_reqmap_mcu.c tables, with a fake stream allocator.
// Expected streams come from the reqmap tables; the pre-change scan loop is re-stated here as
// the reference for the "all options unset" equivalence check. Included by the per-family
// *_unittest.cc files. Not verifiable here: real stream arbitration, ISR timing, cache effects.

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

static const void *const streamAddress[16] = {
    DMA1_Stream0, DMA1_Stream1, DMA1_Stream2, DMA1_Stream3, DMA1_Stream4, DMA1_Stream5, DMA1_Stream6, DMA1_Stream7,
    DMA2_Stream0, DMA2_Stream1, DMA2_Stream2, DMA2_Stream3, DMA2_Stream4, DMA2_Stream5, DMA2_Stream6, DMA2_Stream7,
};

static uint32_t claimedStreams; // bit n = DMA1_ST0_HANDLER + n is owned
static dmaChannelDescriptor_t fakeDescriptors[DMA_LAST_HANDLER + 1];

dmaIdentifier_e dmaGetIdentifier(const DMA_Stream_TypeDef *stream) {
    for (int n = 0; n < 16; n++) {
        if ((const void *)stream == streamAddress[n]) {
            return (dmaIdentifier_e)(DMA1_ST0_HANDLER + n);
        }
    }
    return DMA_NONE;
}

bool dmaAllocate(dmaIdentifier_e identifier, resourceOwner_e owner, uint8_t resourceIndex) {
    UNUSED(owner);
    UNUSED(resourceIndex);
    const uint32_t bit = 1u << DMA_IDENTIFIER_TO_INDEX(identifier);
    if (claimedStreams & bit) {
        return false;
    }
    claimedStreams |= bit;
    return true;
}

dmaChannelDescriptor_t *dmaGetDescriptorByIdentifier(const dmaIdentifier_e identifier) {
    return &fakeDescriptors[identifier];
}

void dmaEnable(dmaIdentifier_e identifier) { UNUSED(identifier); }
void dmaSetHandler(dmaIdentifier_e identifier, dmaCallbackHandlerFuncPtr callback, uint32_t priority, uintptr_t userParam) {
    UNUSED(identifier); UNUSED(callback); UNUSED(priority); UNUSED(userParam);
}
void spiInternalResetStream(dmaChannelDescriptor_t *descriptor) { UNUSED(descriptor); }
void spiInternalResetDescriptors(busDevice_t *bus) { UNUSED(bus); }
void spiInternalInitStream(const extDevice_t *dev, bool preInit) { UNUSED(dev); UNUSED(preInit); }
void spiInternalStartDMA(const extDevice_t *dev) { UNUSED(dev); }
void spiInternalStopDMA(const extDevice_t *dev) { UNUSED(dev); }
void spiInitDevice(SPIDevice device) { UNUSED(device); }
void spiSequenceStart(const extDevice_t *dev) { UNUSED(dev); }
void IOHi(IO_t io) { UNUSED(io); }
void IOLo(IO_t io) { UNUSED(io); }
#if defined(STM32H7)
void SCB_InvalidateDCache_by_Addr(uint32_t *addr, int32_t dsize) { UNUSED(addr); UNUSED(dsize); }
#endif

}

#include "unittest_macros.h"
#include "gtest/gtest.h"

namespace {

#if defined(STM32H7)
constexpr bool kTxOnlyFallback = false; // no USE_TX_IRQ_HANDLER on H7
#else
constexpr bool kTxOnlyFallback = true;
#endif

constexpr int8_t kUnset = DMA_OPT_UNUSED;

// Stream identifier for DMA<controller> stream <stream>.
int streamId(int controller, int stream) { return DMA1_ST0_HANDLER + (controller - 1) * 8 + stream; }

int idOf(const dmaChannelDescriptor_t *d) { return d ? (int)(d - fakeDescriptors) : (int)DMA_NONE; }

void resetState()
{
    claimedStreams = 0;
    memset(fakeDescriptors, 0, sizeof(fakeDescriptors));
    for (int device = 0; device < SPIDEV_COUNT; device++) {
        busDevice_t *bus = spiBusByDevice(static_cast<SPIDevice>(device));
        memset(bus, 0, sizeof(*bus));
        bus->busType = BUS_TYPE_SPI;
        spiPinConfigMutable(device)->txDmaopt = kUnset;
        spiPinConfigMutable(device)->rxDmaopt = kUnset;
    }
}

void setOptions(int device, int8_t tx, int8_t rx)
{
    spiPinConfigMutable(device)->txDmaopt = tx;
    spiPinConfigMutable(device)->rxDmaopt = rx;
}

int txId(int device) { return idOf(spiBusByDevice(static_cast<SPIDevice>(device))->dmaTx); }
int rxId(int device) { return idOf(spiBusByDevice(static_cast<SPIDevice>(device))->dmaRx); }
bool dmaOn(int device) { return spiBusByDevice(static_cast<SPIDevice>(device))->useDMA; }

#if defined(STM32H7)
// Leave only SPIDEV_1 in use so other buses do not claim streams the test inspects.
void onlyDevice1()
{
    for (int device = SPIDEV_2; device < SPIDEV_COUNT; device++) {
        spiBusByDevice(static_cast<SPIDevice>(device))->busType = BUS_TYPE_NONE;
    }
}
#endif

bool claimed(int id) { return claimedStreams & (1u << DMA_IDENTIFIER_TO_INDEX(id)); }

// The scan as it ran before options were stored: try every option, first free stream wins.
int referenceScan(dmaPeripheral_e periph, int device, uint32_t *claims)
{
    for (int opt = 0; opt < MAX_PERIPHERAL_DMA_OPTIONS; opt++) {
        const dmaChannelSpec_t *spec = dmaGetChannelSpecByPeripheral(periph, device, opt);
        if (!spec) {
            continue;
        }
        const int id = dmaGetIdentifier((const DMA_Stream_TypeDef *)spec->ref);
        const uint32_t bit = 1u << DMA_IDENTIFIER_TO_INDEX(id);
        if (*claims & bit) {
            continue;
        }
        *claims |= bit;
        return id;
    }
    return DMA_NONE;
}

} // namespace

// With every option unset the result must match the old scan for every pre-claimed stream set:
// 2^16 sets cover every subset of the 16 streams, so all contention patterns are checked.
TEST(BusSpiReqmapUnittest, UnsetOptionsMatchOldScanForEveryClaimedStreamSet)
{
    for (uint32_t preClaimed = 0; preClaimed < (1u << 16); preClaimed++) {
        resetState();
        claimedStreams = preClaimed;
        spiInitBusDMA();

        uint32_t refClaims = preClaimed;
        for (int device = 0; device < SPIDEV_COUNT; device++) {
            const int tx = referenceScan(DMA_PERIPH_SPI_SDO, device, &refClaims);
            const int rx = referenceScan(DMA_PERIPH_SPI_SDI, device, &refClaims);
            const bool full = tx != DMA_NONE && rx != DMA_NONE;
            const bool txOnly = kTxOnlyFallback && tx != DMA_NONE && rx == DMA_NONE;
            ASSERT_EQ(dmaOn(device), full || txOnly) << "claims " << preClaimed << " device " << device;
            ASSERT_EQ(txId(device), (full || txOnly) ? tx : (int)DMA_NONE) << "claims " << preClaimed << " device " << device;
            ASSERT_EQ(rxId(device), full ? rx : (int)DMA_NONE) << "claims " << preClaimed << " device " << device;
        }
        ASSERT_EQ(claimedStreams, refClaims) << "claims " << preClaimed;
    }
}

#if !defined(STM32H7)
// Tables (generic F4/F7 row): SPI1 SDO {DMA2 S3, DMA2 S5}, SDI {DMA2 S0, DMA2 S2};
// SPI2 SDO {DMA1 S4}, SDI {DMA1 S3}; SPI3 SDO {DMA1 S5, DMA1 S7}, SDI {DMA1 S0, DMA1 S2}.
TEST(BusSpiReqmapUnittest, UnsetOptionsPickFirstTableOptionPerBus)
{
    resetState();
    spiInitBusDMA();

    EXPECT_EQ(txId(SPIDEV_1), streamId(2, 3));
    EXPECT_EQ(rxId(SPIDEV_1), streamId(2, 0));
    EXPECT_EQ(txId(SPIDEV_2), streamId(1, 4));
    EXPECT_EQ(rxId(SPIDEV_2), streamId(1, 3));
    EXPECT_EQ(txId(SPIDEV_3), streamId(1, 5));
    EXPECT_EQ(rxId(SPIDEV_3), streamId(1, 0));
}

TEST(BusSpiReqmapUnittest, PinnedSecondOptionSelectsSecondTableStreamEvenIfFirstIsFree)
{
    resetState();
    setOptions(SPIDEV_1, 1, 1);
    setOptions(SPIDEV_3, 1, kUnset);
    spiInitBusDMA();

    EXPECT_EQ(txId(SPIDEV_1), streamId(2, 5));
    EXPECT_EQ(rxId(SPIDEV_1), streamId(2, 2));
    EXPECT_TRUE(dmaOn(SPIDEV_1));
    EXPECT_FALSE(claimed(streamId(2, 3))); // option 0 left untouched
    EXPECT_FALSE(claimed(streamId(2, 0)));
    EXPECT_EQ(txId(SPIDEV_3), streamId(1, 7));
    EXPECT_EQ(rxId(SPIDEV_3), streamId(1, 0)); // Rx still auto
}

TEST(BusSpiReqmapUnittest, PinnedOptionOnBusWithOneCandidateLeavesBusPolled)
{
    // SPI2 rows list a single option, so option 1 is absent: no DMA, and option 0 stays free.
    resetState();
    setOptions(SPIDEV_2, 1, 1);
    spiInitBusDMA();

    EXPECT_FALSE(dmaOn(SPIDEV_2));
    EXPECT_EQ(txId(SPIDEV_2), (int)DMA_NONE);
    EXPECT_EQ(rxId(SPIDEV_2), (int)DMA_NONE);
    EXPECT_FALSE(claimed(streamId(1, 4)));
    EXPECT_FALSE(claimed(streamId(1, 3)));
}

TEST(BusSpiReqmapUnittest, PinnedStreamOwnedElsewhereDoesNotFallBackToOtherOption)
{
    resetState();
    claimedStreams = 1u << DMA_IDENTIFIER_TO_INDEX(streamId(2, 5)); // e.g. a motor holds DMA2 S5
    setOptions(SPIDEV_1, 1, kUnset);
    spiInitBusDMA();

    EXPECT_EQ(txId(SPIDEV_1), (int)DMA_NONE);
    EXPECT_FALSE(claimed(streamId(2, 3))); // option 0 was never tried
    EXPECT_FALSE(dmaOn(SPIDEV_1));
}

TEST(BusSpiReqmapUnittest, OutOfRangeOptionsLeaveBusPolledAndClaimNothing)
{
    // MAX_PERIPHERAL_DMA_OPTIONS is 2 here.
    const int8_t bad[] = { -2, -128, 2, 16, 127 };
    for (size_t i = 0; i < sizeof(bad); i++) {
        resetState();
        setOptions(SPIDEV_1, bad[i], bad[i]);
        setOptions(SPIDEV_3, bad[i], kUnset);
        spiInitBusDMA();

        EXPECT_FALSE(dmaOn(SPIDEV_1)) << "opt " << (int)bad[i];
        EXPECT_EQ(txId(SPIDEV_1), (int)DMA_NONE) << "opt " << (int)bad[i];
        EXPECT_EQ(rxId(SPIDEV_1), (int)DMA_NONE) << "opt " << (int)bad[i];
        EXPECT_FALSE(claimed(streamId(2, 3))) << "opt " << (int)bad[i];
        EXPECT_FALSE(claimed(streamId(2, 5))) << "opt " << (int)bad[i];
        EXPECT_FALSE(dmaOn(SPIDEV_3)) << "opt " << (int)bad[i]; // Tx invalid -> polled even though Rx is auto
        EXPECT_EQ(txId(SPIDEV_2), streamId(1, 4)); // other buses unaffected
    }
}
#else
// H7 tables: option n is pool stream n (0-7 = DMA1 S0-S7, 8-15 = DMA2 S0-S7), any option valid
// for any bus that has a request row (SPI1..SPI4 here).
TEST(BusSpiReqmapUnittest, UnsetOptionsTakeLowestFreePoolStreams)
{
    resetState();
    spiInitBusDMA();

    // Devices are served in order, Tx then Rx, each taking the lowest free stream: 0,1 / 2,3 / ...
    for (int device = 0; device < SPIDEV_COUNT; device++) {
        EXPECT_EQ(txId(device), DMA1_ST0_HANDLER + 2 * device) << "device " << device;
        EXPECT_EQ(rxId(device), DMA1_ST0_HANDLER + 2 * device + 1) << "device " << device;
    }
}

TEST(BusSpiReqmapUnittest, PinnedOptionSelectsPoolStream)
{
    resetState();
    onlyDevice1();
    setOptions(SPIDEV_1, 9, 15);
    spiInitBusDMA();

    EXPECT_EQ(txId(SPIDEV_1), streamId(2, 1)); // option 9 = DMA2 S1
    EXPECT_EQ(rxId(SPIDEV_1), streamId(2, 7)); // option 15 = DMA2 S7, last valid
    EXPECT_TRUE(dmaOn(SPIDEV_1));
    EXPECT_FALSE(claimed(streamId(1, 0)));
}

TEST(BusSpiReqmapUnittest, PinnedStreamOwnedElsewhereLeavesBusPolled)
{
    resetState();
    onlyDevice1();
    const uint32_t owned = 1u << DMA_IDENTIFIER_TO_INDEX(streamId(2, 1));
    claimedStreams = owned;
    setOptions(SPIDEV_1, 9, 3);
    spiInitBusDMA();

    EXPECT_EQ(txId(SPIDEV_1), (int)DMA_NONE);
    EXPECT_FALSE(dmaOn(SPIDEV_1));
    // Only the pinned Rx stream (option 3 = DMA1 S3) is taken: Tx did not fall back to the lowest free stream.
    EXPECT_EQ(claimedStreams, owned | (1u << DMA_IDENTIFIER_TO_INDEX(streamId(1, 3))));
}

TEST(BusSpiReqmapUnittest, OutOfRangeOptionsLeaveBusPolledAndClaimNothing)
{
    const int8_t bad[] = { -2, -128, 16, 17, 127 };
    for (size_t i = 0; i < sizeof(bad); i++) {
        resetState();
        onlyDevice1();
        setOptions(SPIDEV_1, bad[i], bad[i]);
        spiInitBusDMA();

        EXPECT_FALSE(dmaOn(SPIDEV_1)) << "opt " << (int)bad[i];
        EXPECT_EQ(txId(SPIDEV_1), (int)DMA_NONE) << "opt " << (int)bad[i];
        EXPECT_EQ(rxId(SPIDEV_1), (int)DMA_NONE) << "opt " << (int)bad[i];
        EXPECT_EQ(claimedStreams, 0u) << "opt " << (int)bad[i]; // no wrap to a valid option
    }
}
#endif
