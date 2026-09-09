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

// Compiles the real dma_reqmap_mcu.c under -DSTM32F7. Covers what an F4-only compile
// (dma_reqmap_unittest.cc) cannot exercise: the DMA_CHANNEL_x-macro variant of DMA()
// (F4 uses DMA_Channel_x, a different macro entirely) and the UARTDEV_7/UARTDEV_8 table
// rows that exist only under #if defined(STM32F7).

extern "C" {

#include "platform.h"
#include "drivers/dma_reqmap.h"
#include "drivers/serial.h"
#include "drivers/serial_uart.h"

}

#include "unittest_macros.h"
#include "gtest/gtest.h"

// --- F7-exclusive UART entries: no F4 equivalent, unreachable from the F4 test binary ---

TEST(DmaReqmapF7Unittest, Uart7RxAndTxResolveToDistinctStreams)
{
    const dmaChannelSpec_t *rx = dmaGetChannelSpecByPeripheral(DMA_PERIPH_UART_RX, UARTDEV_7, 0);
    const dmaChannelSpec_t *tx = dmaGetChannelSpecByPeripheral(DMA_PERIPH_UART_TX, UARTDEV_7, 0);

    ASSERT_NE(rx, nullptr);
    ASSERT_NE(tx, nullptr);
    EXPECT_NE(rx->ref, tx->ref);
}

TEST(DmaReqmapF7Unittest, Uart8RxAndTxResolveToDistinctStreams)
{
    const dmaChannelSpec_t *rx = dmaGetChannelSpecByPeripheral(DMA_PERIPH_UART_RX, UARTDEV_8, 0);
    const dmaChannelSpec_t *tx = dmaGetChannelSpecByPeripheral(DMA_PERIPH_UART_TX, UARTDEV_8, 0);

    ASSERT_NE(rx, nullptr);
    ASSERT_NE(tx, nullptr);
    EXPECT_NE(rx->ref, tx->ref);
}

TEST(DmaReqmapF7Unittest, Uart7HasNoSecondAlternateOption)
{
    // UARTDEV_7/8 list exactly one silicon-valid option each -- unlike UARTDEV_1, opt 1 must fail closed.
    EXPECT_EQ(dmaGetChannelSpecByPeripheral(DMA_PERIPH_UART_RX, UARTDEV_7, 1), nullptr);
}

// --- Shared UART entries: confirm the same rows F4 covers also resolve under the F7 compile ---

TEST(DmaReqmapF7Unittest, Uart1RxAndTxResolveToDistinctStreams)
{
    const dmaChannelSpec_t *rx = dmaGetChannelSpecByPeripheral(DMA_PERIPH_UART_RX, UARTDEV_1, 0);
    const dmaChannelSpec_t *tx = dmaGetChannelSpecByPeripheral(DMA_PERIPH_UART_TX, UARTDEV_1, 0);

    ASSERT_NE(rx, nullptr);
    ASSERT_NE(tx, nullptr);
    EXPECT_NE(rx->ref, tx->ref);
}

// --- Adversarial: same fail-closed contract must hold under the F7 macro variant too ---

TEST(DmaReqmapF7Unittest, RejectsNegativeOptIndex)
{
    EXPECT_EQ(dmaGetChannelSpecByPeripheral(DMA_PERIPH_UART_RX, UARTDEV_7, -1), nullptr);
}

TEST(DmaReqmapF7Unittest, RejectsOptIndexPastTableWidth)
{
    EXPECT_EQ(dmaGetChannelSpecByPeripheral(DMA_PERIPH_UART_RX, UARTDEV_7, MAX_PERIPHERAL_DMA_OPTIONS), nullptr);
}

// --- Timer table (IT #1396): dmaGetChannelSpecByTimerValue() was a stub returning NULL
// unconditionally on F4/F7 until this table was implemented -- verifies the real per-option
// data, not just that a value comes back. ---

TEST(DmaReqmapF7Unittest, Tim1Ch1AllThreeOptionsResolveToDistinctStreams)
{
    const dmaChannelSpec_t *opt0 = dmaGetChannelSpecByTimerValue((TIM_TypeDef *)TIM1, TIM_CHANNEL_1, 0);
    const dmaChannelSpec_t *opt1 = dmaGetChannelSpecByTimerValue((TIM_TypeDef *)TIM1, TIM_CHANNEL_1, 1);
    const dmaChannelSpec_t *opt2 = dmaGetChannelSpecByTimerValue((TIM_TypeDef *)TIM1, TIM_CHANNEL_1, 2);

    ASSERT_NE(opt0, nullptr);
    ASSERT_NE(opt1, nullptr);
    ASSERT_NE(opt2, nullptr);
    EXPECT_NE(opt0->ref, opt1->ref);
    EXPECT_NE(opt0->ref, opt2->ref);
    EXPECT_NE(opt1->ref, opt2->ref);
}

TEST(DmaReqmapF7Unittest, Tim8Ch1SameStreamDifferentChannelOptionsAreDistinguishable)
{
    // TIM8_CH1's two options (DMA(2,2,0) and DMA(2,2,7)) share one physical DMA2_Stream2 --
    // only .channel (the mux selector) tells them apart. A table or lookup bug that ignores
    // .channel would make these two options indistinguishable.
    const dmaChannelSpec_t *opt0 = dmaGetChannelSpecByTimerValue((TIM_TypeDef *)TIM8, TIM_CHANNEL_1, 0);
    const dmaChannelSpec_t *opt1 = dmaGetChannelSpecByTimerValue((TIM_TypeDef *)TIM8, TIM_CHANNEL_1, 1);

    ASSERT_NE(opt0, nullptr);
    ASSERT_NE(opt1, nullptr);
    EXPECT_EQ(opt0->ref, opt1->ref);
    EXPECT_NE(opt0->channel, opt1->channel);
}

TEST(DmaReqmapF7Unittest, RejectsUnmappedTimerChannel)
{
    // TIM3 is in the table, but a channel value past TIM_CHANNEL_4 has no entry.
    EXPECT_EQ(dmaGetChannelSpecByTimerValue((TIM_TypeDef *)TIM3, TIM_CHANNEL_4 + 1, 0), nullptr);
}

TEST(DmaReqmapF7Unittest, RejectsNegativeTimerOptIndex)
{
    EXPECT_EQ(dmaGetChannelSpecByTimerValue((TIM_TypeDef *)TIM1, TIM_CHANNEL_1, -1), nullptr);
}

TEST(DmaReqmapF7Unittest, RejectsTimerOptIndexPastTableWidth)
{
    EXPECT_EQ(dmaGetChannelSpecByTimerValue((TIM_TypeDef *)TIM1, TIM_CHANNEL_1, MAX_TIMER_DMA_OPTIONS), nullptr);
}
