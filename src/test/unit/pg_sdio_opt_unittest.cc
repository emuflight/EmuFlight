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

// Compiles the real pg/sdio.c with a target-defined SDCARD_SDIO_DMA_OPT of 0 (reqmap option 0).

extern "C" {

#include "platform.h"
#include "drivers/dma_reqmap.h"
#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/sdio.h"

}

#include "unittest_macros.h"
#include "gtest/gtest.h"

// The PG layout is shared by every target; a changed layout needs a new version so a stored
// version-0 record is dropped instead of copied byte-for-byte into the new layout.
static_assert(sizeof(sdioConfig_t) == 5, "sdioConfig_t layout changed: bump PG_SDIO_CONFIG");

TEST(PgSdioOptUnittest, ResetDefaultDmaoptMatchesTargetSetting)
{
    pgResetAll();
    EXPECT_EQ(sdioConfig()->dmaopt, 0);
}

TEST(PgSdioOptUnittest, ResetKeepsOtherFieldsAtTheirDefaults)
{
    pgResetAll();
    EXPECT_EQ(sdioConfig()->clockBypass, 0);
    EXPECT_EQ(sdioConfig()->useCache, 0);
    EXPECT_EQ(sdioConfig()->device, 1);
}

TEST(PgSdioOptUnittest, StaleVersionZeroRecordIsRejectedAndLeavesDefaults)
{
    pgResetAll();
    const pgRegistry_t *reg = pgFind(PG_SDIO_CONFIG);
    ASSERT_NE(reg, nullptr);
    EXPECT_EQ(pgVersion(reg), 1);

    // A version-0 record is 4 bytes: clockBypass, useCache, use4BitWidth, device(=2).
    const uint8_t oldRecord[4] = { 1, 1, 1, 2 };
    EXPECT_FALSE(pgLoad(reg, oldRecord, sizeof(oldRecord), 0));
    EXPECT_EQ(sdioConfig()->device, 1);
    EXPECT_EQ(sdioConfig()->dmaopt, 0);
}
