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

extern "C" {

#include "platform.h"
#include "drivers/dma.h"

}

#include "unittest_macros.h"
#include "gtest/gtest.h"

// Mirrors the identifier-range guard duplicated in dma_stm32f4xx.c / dma_stm32f7xx.c / dma_stm32h7xx.c.
static bool dmaIdentifierIsValid(dmaIdentifier_e identifier) {
    return identifier > DMA_NONE && identifier <= DMA_LAST_HANDLER;
}

TEST(DmaBoundsUnittest, RejectsNoneIdentifier)
{
    EXPECT_FALSE(dmaIdentifierIsValid(DMA_NONE));
}

TEST(DmaBoundsUnittest, RejectsIdentifierPastLastHandler)
{
    const dmaIdentifier_e pastLast = static_cast<dmaIdentifier_e>(DMA_LAST_HANDLER + 1);
    EXPECT_FALSE(dmaIdentifierIsValid(pastLast));
}

TEST(DmaBoundsUnittest, AcceptsFirstAndLastHandler)
{
    const dmaIdentifier_e first = static_cast<dmaIdentifier_e>(DMA_NONE + 1);
    EXPECT_TRUE(dmaIdentifierIsValid(first));
    EXPECT_TRUE(dmaIdentifierIsValid(DMA_LAST_HANDLER));
}
