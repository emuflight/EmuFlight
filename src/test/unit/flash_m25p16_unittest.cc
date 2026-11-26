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
    { 0x684018, 16 * MIB, 64 * KIB, "BY25Q128ES" },
    { 0xC84015,  2 * MIB, 64 * KIB, "GD25Q16E" },
    { 0xC84018, 16 * MIB, 64 * KIB, "GD25Q128E" },
    { 0xA14017,  8 * MIB, 64 * KIB, "FM25Q64AI3" },
    { 0xA14018, 16 * MIB, 64 * KIB, "FM25Q128AI3" },
    { 0xBA6016,  4 * MIB, 64 * KIB, "ZD25WQ32C" },
    { 0xC46016,  4 * MIB, 64 * KIB, "GT25Q32A" },
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
    static const uint32_t unknown[] = { 0x000000, 0xFFFFFF, 0xEF4021, 0xEF4013, 0xC22015, 0xC2201A, 0x1C7116, 0xC84016, 0x684019, 0xA14019, 0x206019, 0x012017, 0x012019 };
    for (uint32_t id : unknown) {
        flashDevice_t fdevice;
        EXPECT_FALSE(identify(id, &fdevice)) << std::hex << id;
        EXPECT_EQ(0u, fdevice.geometry.sectors);
        EXPECT_EQ(0u, fdevice.geometry.pagesPerSector);
        EXPECT_EQ(0u, fdevice.geometry.sectorSize);
        EXPECT_EQ(0u, fdevice.geometry.totalSize);
    }
}

// S25FL128S (0x012018) RDID bytes from the datasheet ID table: 01h 20h 18h 4Dh (CFI length),
// then 04h = 01h (hybrid: 64 KB sectors, 256-byte page) or 00h (uniform: 256 KB sectors, 512-byte page),
// 05h = 80h (FL-S family). Only the hybrid layout is supported.
static const uint8_t s25fl128sHybridId[6]  = { 0x01, 0x20, 0x18, 0x4D, 0x01, 0x80 };
static const uint8_t s25fl128sUniformId[6] = { 0x01, 0x20, 0x18, 0x4D, 0x00, 0x80 };

TEST(FlashM25p16Test, S25fl128sNeedsExtendedRead)
{
    EXPECT_TRUE(m25p16_idNeedsExtendedRead(0x012018));
    EXPECT_EQ(6, M25P16_EXT_ID_LENGTH);
    // Neighbours and other 128 Mb parts keep the 3-byte decision.
    EXPECT_FALSE(m25p16_idNeedsExtendedRead(0x012019));
    EXPECT_FALSE(m25p16_idNeedsExtendedRead(0x012017));
    EXPECT_FALSE(m25p16_idNeedsExtendedRead(0xC22018));
    EXPECT_FALSE(m25p16_idNeedsExtendedRead(0xEF4018));
}

TEST(FlashM25p16Test, S25fl128sHybridIsAcceptedWithDatasheetGeometry)
{
    EXPECT_TRUE(m25p16_extendedIdSupported(0x012018, s25fl128sHybridId, sizeof(s25fl128sHybridId)));
    expectChip({ 0x012018, 16 * MIB, 64 * KIB, "S25FL128S hybrid" });
}

TEST(FlashM25p16Test, S25fl128sUniformIsRejected)
{
    EXPECT_FALSE(m25p16_extendedIdSupported(0x012018, s25fl128sUniformId, sizeof(s25fl128sUniformId)));
}

TEST(FlashM25p16Test, S25fl128sUnknownExtendedBytesAreRejected)
{
    uint8_t id[6];
    for (unsigned b4 = 0; b4 < 256; b4++) {
        for (unsigned b5 = 0; b5 < 256; b5++) {
            memcpy(id, s25fl128sHybridId, sizeof(id));
            id[4] = b4;
            id[5] = b5;
            const bool expected = (b4 == 0x01 && b5 == 0x80);
            EXPECT_EQ(expected, m25p16_extendedIdSupported(0x012018, id, sizeof(id))) << b4 << " " << b5;
        }
    }
}

TEST(FlashM25p16Test, S25fl128sShortOrEmptyResponseIsRejected)
{
    // A 4-byte or 5-byte read cannot carry byte 05h; a floating bus reads all-zero or all-one.
    EXPECT_FALSE(m25p16_extendedIdSupported(0x012018, s25fl128sHybridId, 0));
    EXPECT_FALSE(m25p16_extendedIdSupported(0x012018, s25fl128sHybridId, 4));
    EXPECT_FALSE(m25p16_extendedIdSupported(0x012018, s25fl128sHybridId, 5));
    static const uint8_t zeros[6] = { 0 };
    static const uint8_t ones[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
    EXPECT_FALSE(m25p16_extendedIdSupported(0x012018, zeros, sizeof(zeros)));
    EXPECT_FALSE(m25p16_extendedIdSupported(0x012018, ones, sizeof(ones)));
}

TEST(FlashM25p16Test, NeighbourIdIsUnaffectedByExtendedCheck)
{
    // 0x012019 (not in the table) and MX25L12845G (shared 128 Mb class) never depend on bytes 04h/05h.
    static const uint8_t junk[6] = { 0 };
    flashDevice_t fdevice;
    EXPECT_TRUE(m25p16_extendedIdSupported(0x012019, junk, sizeof(junk)));
    EXPECT_FALSE(identify(0x012019, &fdevice));
    EXPECT_TRUE(m25p16_extendedIdSupported(0xC22018, junk, 0));
    EXPECT_TRUE(identify(0xC22018, &fdevice));
}
