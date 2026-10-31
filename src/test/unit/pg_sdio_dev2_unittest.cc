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

// Compiles the real pg/sdio.c with SDIO_DEVICE=SDIODEV_2: the stored device is the 1-based SDMMC number.

extern "C" {

#include "platform.h"
#include "drivers/sdio.h"
#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/sdio.h"

}

#include "unittest_macros.h"
#include "gtest/gtest.h"

TEST(PgSdioDev2Unittest, DeviceFollowsTargetSetting)
{
    pgResetAll();
    EXPECT_EQ(sdioConfig()->device, 2);
    EXPECT_EQ(SDIO_CFG_TO_DEV(sdioConfig()->device), SDIODEV_2);
}

TEST(PgSdioDev2Unittest, DeviceIsNeverTheInvalidValue)
{
    pgResetAll();
    EXPECT_NE(sdioConfig()->device, 0);
}
