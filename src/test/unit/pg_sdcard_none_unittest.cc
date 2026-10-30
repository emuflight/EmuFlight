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

// Compiles the real pg/sdcard.c for a USE_SDCARD target with neither an SPI instance nor SDIO.

extern "C" {

#include "platform.h"
#include "drivers/io.h"
#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/sdcard.h"

}

#include "unittest_macros.h"
#include "gtest/gtest.h"

TEST(PgSdcardNoneUnittest, ResetLeavesModeNone)
{
    pgResetAll();
    EXPECT_EQ(sdcardConfig()->mode, SDCARD_MODE_NONE);
    EXPECT_EQ(sdcardConfig()->device, 0);
}
