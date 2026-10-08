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

// Compiles the real pg/sdcard.c for an SDIO target with no SPI instance. The Makefile also
// defines the legacy SD DMA macros as undeclared symbols: the build fails if pg/sdcard.c reads them.

extern "C" {

#include "platform.h"
#include "drivers/io.h"
#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/sdcard.h"

}

#include "unittest_macros.h"
#include "gtest/gtest.h"

// Layout is device, cardDetectTag, chipSelectTag, cardDetectInverted, then mode: four one-byte
// fields before the enum. The firmware toolchain makes the enum one byte; the host makes it wider.
static_assert(sizeof(ioTag_t) == 1, "ioTag_t width changed: recheck sdcardConfig_t layout");
static_assert(offsetof(sdcardConfig_t, mode) == 4, "sdcardConfig_t layout changed: bump PG_SDCARD_CONFIG");

static void expectResetDefaults(void)
{
    EXPECT_EQ(sdcardConfig()->mode, SDCARD_MODE_SDIO);
    EXPECT_EQ(sdcardConfig()->device, 0);
    EXPECT_EQ(sdcardConfig()->cardDetectTag, IO_TAG_NONE);
    EXPECT_EQ(sdcardConfig()->chipSelectTag, IO_TAG_NONE);
    EXPECT_EQ(sdcardConfig()->cardDetectInverted, 1);
}

TEST(PgSdcardUnittest, VersionIsOneAboveTheLayoutWithLeadingEnabledByte)
{
    const pgRegistry_t *reg = pgFind(PG_SDCARD_CONFIG);
    ASSERT_NE(reg, nullptr);
    EXPECT_EQ(pgVersion(reg), 3);
    EXPECT_EQ(pgSize(reg), sizeof(sdcardConfig_t));
}

TEST(PgSdcardUnittest, ResetDefaultsForSdioTargetWithoutSpiInstance)
{
    pgResetAll();
    expectResetDefaults();
}

TEST(PgSdcardUnittest, CurrentVersionRecordLoadsEachFieldAtItsOffset)
{
    pgResetAll();
    const pgRegistry_t *reg = pgFind(PG_SDCARD_CONFIG);
    ASSERT_NE(reg, nullptr);

    sdcardConfig_t record = {};
    record.device = 3;
    record.cardDetectTag = 0x12;
    record.chipSelectTag = 0x34;
    record.cardDetectInverted = 0;
    record.mode = SDCARD_MODE_SPI;
    EXPECT_TRUE(pgLoad(reg, &record, sizeof(record), 3));
    EXPECT_EQ(sdcardConfig()->device, 3);
    EXPECT_EQ(sdcardConfig()->cardDetectTag, 0x12);
    EXPECT_EQ(sdcardConfig()->chipSelectTag, 0x34);
    EXPECT_EQ(sdcardConfig()->cardDetectInverted, 0);
    EXPECT_EQ(sdcardConfig()->mode, SDCARD_MODE_SPI);
}

TEST(PgSdcardUnittest, StaleVersionOneRecordIsRejectedAndLeavesDefaults)
{
    pgResetAll();
    const pgRegistry_t *reg = pgFind(PG_SDCARD_CONFIG);
    ASSERT_NE(reg, nullptr);

    // A version-1 record is 6 bytes: the five fields above plus the removed dmaIdentifier.
    const uint8_t oldRecord[6] = { 0, 3, 0x12, 0x34, 0, 7 };
    EXPECT_FALSE(pgLoad(reg, oldRecord, sizeof(oldRecord), 1));
    expectResetDefaults();
}

TEST(PgSdcardUnittest, StaleVersionTwoRecordWithLeadingEnabledByteIsRejected)
{
    pgResetAll();
    const pgRegistry_t *reg = pgFind(PG_SDCARD_CONFIG);
    ASSERT_NE(reg, nullptr);

    // Version 2 stored {enabled, device, cardDetectTag, chipSelectTag, cardDetectInverted}.
    const uint8_t oldRecord[5] = { 0, 3, 0x12, 0x34, 0 };
    EXPECT_FALSE(pgLoad(reg, oldRecord, sizeof(oldRecord), 2));
    expectResetDefaults();
}
