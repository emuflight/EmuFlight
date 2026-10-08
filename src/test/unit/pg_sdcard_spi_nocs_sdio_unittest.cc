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

// Compiles the real pg/sdcard.c: SPI instance without a chip-select pin, built with SDIO: SDIO stays
// Expected values follow the reference reset order (mode NONE, SDIO when built with SDIO, then
// SPI only when the SPI device resolves and a chip-select tag exists), not the code's own output.

extern "C" {

#include "platform.h"
#include "drivers/bus_spi.h"
#include "drivers/io.h"
#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/sdcard.h"

SPIDevice spiDeviceByInstance(SPI_TypeDef *)
{
    return SPIDEV_3;
}

}

#include "unittest_macros.h"
#include "gtest/gtest.h"

TEST(PgSdcardSpiNoCsSdioUnittest, ResetSelectsExpectedMode)
{
    pgResetAll();
    EXPECT_EQ(sdcardConfig()->mode, SDCARD_MODE_SDIO);
}
