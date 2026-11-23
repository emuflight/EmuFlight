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

#include "drivers/bus.h"
#include "drivers/bus_spi.h"
#include "drivers/flash.h"
#include "drivers/flash_impl.h"
#include "drivers/flash_m25p16.h"

}

#include "unittest_macros.h"
#include "gtest/gtest.h"

// m25p16_identify() is the real driver code; only the bus and time calls it links against are faked.
// Expected values below come from the vendor datasheets (total capacity in bytes, 256-byte page,
// erase unit used by the 0xD8 sector-erase command), not from the driver table.

extern "C" {

void spiWait(const extDevice_t *dev) { (void)dev; }
void spiSequence(const extDevice_t *dev, busSegment_t *segments) { (void)dev; (void)segments; }
void spiReadWriteBuf(const extDevice_t *dev, uint8_t *txData, uint8_t *rxData, int len)
{
    (void)dev; (void)txData; (void)rxData; (void)len;
}
bool spiIsBusy(const extDevice_t *dev) { (void)dev; return false; }
uint32_t millis(void) { return 0; }
void spiSetClkDivisor(const extDevice_t *dev, uint16_t divider) { (void)dev; (void)divider; }
uint16_t spiCalculateDivider(uint32_t freq) { (void)freq; return 2; }
void spiSetAtomicWait(const extDevice_t *dev) { (void)dev; }
uint16_t spiGetErrorCounter(SPI_TypeDef *instance) { (void)instance; return 0; }

}

typedef struct {
    uint32_t jedecId;
    uint32_t totalBytes;
    uint32_t eraseUnitBytes;
    const char *name;
} expectedChip_t;

#define KIB 1024u
#define MIB (1024u * 1024u)

// 64 KiB erase unit (0xD8) on every row below, so sectors = totalBytes / 64 KiB.
static const expectedChip_t expectedNewChips[] = {
    { 0xC22018, 16 * MIB, 64 * KIB, "MX25L12845G" },
    { 0xEF6017,  8 * MIB, 64 * KIB, "W25Q64JW-IQ" },
    { 0xEF8017,  8 * MIB, 64 * KIB, "W25Q64JW-IM" },
    { 0xEF4020, 64 * MIB, 64 * KIB, "W25Q512JV" },
};

static const expectedChip_t expectedExistingChips[] = {
    { 0xEF4018, 16 * MIB, 64 * KIB, "W25Q128" },
    { 0xEF4017,  8 * MIB, 64 * KIB, "W25Q64JV" },
    { 0xEF4015,  2 * MIB, 64 * KIB, "W25Q16" },
    { 0xC22019, 32 * MIB, 64 * KIB, "MX25L25635E" },
};

static bool identify(uint32_t jedecId, flashDevice_t *fdevice)
{
    memset(fdevice, 0, sizeof(*fdevice));
    fdevice->io.mode = FLASHIO_SPI;
    // 0xAA poison proves a reject path clears the geometry rather than leaving stale values.
    memset(&fdevice->geometry, 0xAA, sizeof(fdevice->geometry));
    return m25p16_identify(fdevice, jedecId);
}

static void expectChip(const expectedChip_t &chip)
{
    flashDevice_t fdevice;
    ASSERT_TRUE(identify(chip.jedecId, &fdevice)) << chip.name;
    const flashGeometry_t &g = fdevice.geometry;
    EXPECT_EQ(FLASH_TYPE_NOR, g.flashType) << chip.name;
    EXPECT_EQ(256u, g.pageSize) << chip.name;
    EXPECT_EQ(chip.totalBytes, g.totalSize) << chip.name;
    EXPECT_EQ(chip.eraseUnitBytes, g.sectorSize) << chip.name;
    EXPECT_EQ(chip.totalBytes / chip.eraseUnitBytes, g.sectors) << chip.name;
    EXPECT_EQ(g.sectorSize, g.pagesPerSector * g.pageSize) << chip.name;
    EXPECT_EQ(g.totalSize, g.sectors * g.sectorSize) << chip.name;
}

TEST(FlashM25p16Test, ExistingChipsKeepTheirGeometry)
{
    for (const expectedChip_t &chip : expectedExistingChips) {
        expectChip(chip);
    }
}

TEST(FlashM25p16Test, NewChipsMatchDatasheetGeometry)
{
    for (const expectedChip_t &chip : expectedNewChips) {
        expectChip(chip);
    }
}

TEST(FlashM25p16Test, ExpectedTablesHaveNoDuplicateIds)
{
    const size_t newCount = sizeof(expectedNewChips) / sizeof(expectedNewChips[0]);
    const size_t oldCount = sizeof(expectedExistingChips) / sizeof(expectedExistingChips[0]);
    for (size_t i = 0; i < newCount; i++) {
        for (size_t j = i + 1; j < newCount; j++) {
            EXPECT_NE(expectedNewChips[i].jedecId, expectedNewChips[j].jedecId);
        }
        for (size_t j = 0; j < oldCount; j++) {
            EXPECT_NE(expectedNewChips[i].jedecId, expectedExistingChips[j].jedecId);
        }
    }
}

TEST(FlashM25p16Test, UnknownIdsAreRejectedAndGeometryCleared)
{
    // Neighbours of supported IDs and the two values a floating or absent bus reads back.
    static const uint32_t unknown[] = { 0x000000, 0xFFFFFF, 0xEF4021, 0xEF4013, 0xC22015, 0xC2201A, 0x1C7116 };
    for (uint32_t id : unknown) {
        flashDevice_t fdevice;
        EXPECT_FALSE(identify(id, &fdevice)) << std::hex << id;
        EXPECT_EQ(0u, fdevice.geometry.sectors);
        EXPECT_EQ(0u, fdevice.geometry.pagesPerSector);
        EXPECT_EQ(0u, fdevice.geometry.sectorSize);
        EXPECT_EQ(0u, fdevice.geometry.totalSize);
    }
}

// Same JEDEC ID, different parts: the 3-byte ID cannot tell the variants apart, so one row must
// describe both. Row order must not change the result.
TEST(FlashM25p16Test, SharedIdMapsToOneGeometry)
{
    flashDevice_t a;
    flashDevice_t b;
    ASSERT_TRUE(identify(0xC22018, &a));
    ASSERT_TRUE(identify(0xC22018, &b));
    EXPECT_EQ(16 * MIB, a.geometry.totalSize);
    EXPECT_EQ(a.geometry.totalSize, b.geometry.totalSize);
}
