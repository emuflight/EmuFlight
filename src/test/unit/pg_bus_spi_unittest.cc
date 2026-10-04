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

// The real pg/bus_spi.c: reset defaults and the PG version contract of spiPinConfig.

#include <stddef.h>
#include <string.h>

extern "C" {

#include "platform.h"
#include "drivers/bus_spi.h"
#include "drivers/dma_reqmap.h"
#include "drivers/io.h"
#include "pg/bus_spi.h"
#include "pg/pg.h"
#include "pg/pg_ids.h"

}

#include "unittest_macros.h"
#include "gtest/gtest.h"

// Layout the stored records and the CLI stride/offset rely on.
static_assert(sizeof(spiPinConfig_t) == 5, "spiPinConfig_t stride");
static_assert(offsetof(spiPinConfig_t, txDmaopt) == 3, "txDmaopt offset");
static_assert(offsetof(spiPinConfig_t, rxDmaopt) == 4, "rxDmaopt offset");

namespace {
constexpr uint8_t kLegacyStride = 3; // sck, miso, mosi
}

TEST(PgBusSpiUnittest, ResetSetsEveryDeviceOptionToUnset)
{
    memset(spiPinConfigMutable(0), 0, sizeof(spiPinConfig_t) * SPIDEV_COUNT);
    pgResetAll();

    // SPIDEV_3 has no entry in spiDefaultConfig[] on this target; zero would pin option 0 there.
    for (int device = 0; device < SPIDEV_COUNT; device++) {
        EXPECT_EQ(spiPinConfig(device)->txDmaopt, DMA_OPT_UNUSED) << "device " << device;
        EXPECT_EQ(spiPinConfig(device)->rxDmaopt, DMA_OPT_UNUSED) << "device " << device;
    }
}

TEST(PgBusSpiUnittest, ResetKeepsBoardPinsAndLeavesAbsentDevicePinsEmpty)
{
    memset(spiPinConfigMutable(0), 0xFF, sizeof(spiPinConfig_t) * SPIDEV_COUNT);
    pgResetAll();

    EXPECT_EQ(spiPinConfig(SPIDEV_1)->ioTagSck, IO_TAG(SPI1_SCK_PIN));
    EXPECT_EQ(spiPinConfig(SPIDEV_1)->ioTagMiso, IO_TAG(SPI1_MISO_PIN));
    EXPECT_EQ(spiPinConfig(SPIDEV_1)->ioTagMosi, IO_TAG(SPI1_MOSI_PIN));
    EXPECT_EQ(spiPinConfig(SPIDEV_2)->ioTagSck, IO_TAG(SPI2_SCK_PIN));
    EXPECT_EQ(spiPinConfig(SPIDEV_3)->ioTagSck, 0);
    EXPECT_EQ(spiPinConfig(SPIDEV_3)->ioTagMosi, 0);
}

TEST(PgBusSpiUnittest, VersionOneRecordIsRejectedAndLeavesDefaults)
{
    // A stored record from before the options existed: 3-byte entries. Loading it into the 5-byte
    // layout would shift every pin, so the version must differ and the load must be refused.
    uint8_t legacy[kLegacyStride * SPIDEV_COUNT];
    memset(legacy, 0x5A, sizeof(legacy));
    const pgRegistry_t *reg = pgFind(PG_SPI_PIN_CONFIG);
    ASSERT_NE(reg, nullptr);
    ASSERT_NE(pgVersion(reg), 1);

    EXPECT_FALSE(pgLoad(reg, legacy, sizeof(legacy), 1));

    for (int device = 0; device < SPIDEV_COUNT; device++) {
        EXPECT_EQ(spiPinConfig(device)->txDmaopt, DMA_OPT_UNUSED) << "device " << device;
        EXPECT_EQ(spiPinConfig(device)->rxDmaopt, DMA_OPT_UNUSED) << "device " << device;
    }
    EXPECT_EQ(spiPinConfig(SPIDEV_1)->ioTagSck, IO_TAG(SPI1_SCK_PIN)); // pins back at board defaults
}

TEST(PgBusSpiUnittest, CurrentVersionRecordRoundTripsOptions)
{
    pgResetAll();
    spiPinConfigMutable(SPIDEV_2)->txDmaopt = 1;
    spiPinConfigMutable(SPIDEV_2)->rxDmaopt = 0;

    const pgRegistry_t *reg = pgFind(PG_SPI_PIN_CONFIG);
    uint8_t stored[sizeof(spiPinConfig_t) * SPIDEV_COUNT];
    ASSERT_EQ(pgSize(reg), sizeof(stored));
    pgStore(reg, stored, sizeof(stored));

    pgResetAll();
    ASSERT_EQ(spiPinConfig(SPIDEV_2)->txDmaopt, DMA_OPT_UNUSED);
    EXPECT_TRUE(pgLoad(reg, stored, sizeof(stored), pgVersion(reg)));
    EXPECT_EQ(spiPinConfig(SPIDEV_2)->txDmaopt, 1);
    EXPECT_EQ(spiPinConfig(SPIDEV_2)->rxDmaopt, 0);
    EXPECT_EQ(spiPinConfig(SPIDEV_1)->txDmaopt, DMA_OPT_UNUSED);
}
