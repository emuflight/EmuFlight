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

// TIM_UP burst DShot resolves its stream through the TIMUP rows of the real H7
// dma_reqmap_mcu.c; the option is a pool stream index (DMA1 S0-S7 = 0-7, DMA2 S0-S7 = 8-15).

extern "C" {

#include "platform.h"
#include "drivers/dma_reqmap.h"

}

#include "unittest_macros.h"
#include "gtest/gtest.h"

// The index is the timer number minus one, matching the TIMUPn_DMA_OPT slot.
TEST(DmaReqmapH7TimUpUnittest, OptSelectsPoolStreamAndKeepsTimerUpRequest)
{
    const dmaChannelSpec_t *tim3 = dmaGetChannelSpecByPeripheral(DMA_PERIPH_TIMUP, 2, 2);
    ASSERT_NE(tim3, nullptr);
    EXPECT_EQ(tim3->ref, (dmaResource_t *)DMA1_Stream2);
    EXPECT_EQ(tim3->channel, static_cast<uint32_t>(DMA_REQUEST_TIM3_UP));

    const dmaChannelSpec_t *tim8 = dmaGetChannelSpecByPeripheral(DMA_PERIPH_TIMUP, 7, 11);
    ASSERT_NE(tim8, nullptr);
    EXPECT_EQ(tim8->ref, (dmaResource_t *)DMA2_Stream3);
    EXPECT_EQ(tim8->channel, static_cast<uint32_t>(DMA_REQUEST_TIM8_UP));
}

TEST(DmaReqmapH7TimUpUnittest, DifferentTimersGetDifferentRequestCodes)
{
    const dmaChannelSpec_t *tim1 = dmaGetChannelSpecByPeripheral(DMA_PERIPH_TIMUP, 0, 0);
    ASSERT_NE(tim1, nullptr);
    const uint32_t tim1Channel = tim1->channel;
    const dmaChannelSpec_t *tim4 = dmaGetChannelSpecByPeripheral(DMA_PERIPH_TIMUP, 3, 1);
    ASSERT_NE(tim4, nullptr);

    EXPECT_EQ(tim1Channel, static_cast<uint32_t>(DMA_REQUEST_TIM1_UP));
    EXPECT_EQ(tim4->channel, static_cast<uint32_t>(DMA_REQUEST_TIM4_UP));
}

TEST(DmaReqmapH7TimUpUnittest, RejectsUnusedAndOutOfRangeOpt)
{
    EXPECT_EQ(dmaGetChannelSpecByPeripheral(DMA_PERIPH_TIMUP, 2, DMA_OPT_UNUSED), nullptr);
    EXPECT_EQ(dmaGetChannelSpecByPeripheral(DMA_PERIPH_TIMUP, 2, MAX_PERIPHERAL_DMA_OPTIONS), nullptr);
    EXPECT_EQ(dmaGetChannelSpecByPeripheral(DMA_PERIPH_TIMUP, 2, INT8_MAX), nullptr);
}

TEST(DmaReqmapH7TimUpUnittest, RejectsTimersWithoutTimUpRequest)
{
    // TIM9-TIM14 (indices 8-13) have no TIM_UP DMA request on H7.
    for (uint8_t index = 8; index <= 13; index++) {
        EXPECT_EQ(dmaGetChannelSpecByPeripheral(DMA_PERIPH_TIMUP, index, 0), nullptr) << "index " << static_cast<int>(index);
    }
}

TEST(DmaReqmapH7TimUpUnittest, RejectsIndexPastTheTable)
{
    EXPECT_EQ(dmaGetChannelSpecByPeripheral(DMA_PERIPH_TIMUP, 17, 0), nullptr);
}
