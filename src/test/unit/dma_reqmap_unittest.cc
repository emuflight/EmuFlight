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

// Compiles the real dma_reqmap_mcu.c (F4/F7 branch) under UNIT_TEST; pure table lookup,
// no register access, so only fake DMAx_Streamy/channel literals are needed. This file
// covers the F4 compile only: F4 and F7 share the outer #elif guard but diverge inside it
// -- the DMA() macro itself differs (DMA_Channel_x vs DMA_CHANNEL_x) and UARTDEV_7/8 exist
// only under #if defined(STM32F7). See dma_reqmap_f7_unittest.cc for that coverage.

extern "C" {

#include "platform.h"
#include "drivers/dma_reqmap.h"
#include "drivers/serial.h"
#include "drivers/serial_uart.h"

}

#include "unittest_macros.h"
#include "gtest/gtest.h"

// --- SPI entries: sanity that the real table still resolves the primary bus ---

TEST(DmaReqmapUnittest, SpiSdoAndSdiResolveToDifferentStreamsOnSpiDev2)
{
    const dmaChannelSpec_t *tx = dmaGetChannelSpecByPeripheral(DMA_PERIPH_SPI_SDO, 1, 0);
    const dmaChannelSpec_t *rx = dmaGetChannelSpecByPeripheral(DMA_PERIPH_SPI_SDI, 1, 0);

    ASSERT_NE(tx, nullptr);
    ASSERT_NE(rx, nullptr);
    EXPECT_NE(tx->ref, rx->ref);
}

// --- UART entries: PR #1370/#1383's new reqmap consumers, previously untested ---

TEST(DmaReqmapUnittest, Uart1RxAndTxResolveToDistinctStreams)
{
    const dmaChannelSpec_t *rx = dmaGetChannelSpecByPeripheral(DMA_PERIPH_UART_RX, UARTDEV_1, 0);
    const dmaChannelSpec_t *tx = dmaGetChannelSpecByPeripheral(DMA_PERIPH_UART_TX, UARTDEV_1, 0);

    ASSERT_NE(rx, nullptr);
    ASSERT_NE(tx, nullptr);
    EXPECT_NE(rx->ref, tx->ref);
}

TEST(DmaReqmapUnittest, Uart1RxHasASecondAlternateOption)
{
    // UARTDEV_1 RX opt 1 must resolve to a different stream, giving serialUART() a real fallback.
    const dmaChannelSpec_t *opt0 = dmaGetChannelSpecByPeripheral(DMA_PERIPH_UART_RX, UARTDEV_1, 0);
    const dmaChannelSpec_t *opt1 = dmaGetChannelSpecByPeripheral(DMA_PERIPH_UART_RX, UARTDEV_1, 1);

    ASSERT_NE(opt0, nullptr);
    ASSERT_NE(opt1, nullptr);
    EXPECT_NE(opt0->ref, opt1->ref);
}

TEST(DmaReqmapUnittest, Uart6RxAndTxResolveToDistinctStreams)
{
    const dmaChannelSpec_t *rx = dmaGetChannelSpecByPeripheral(DMA_PERIPH_UART_RX, UARTDEV_6, 0);
    const dmaChannelSpec_t *tx = dmaGetChannelSpecByPeripheral(DMA_PERIPH_UART_TX, UARTDEV_6, 0);

    ASSERT_NE(rx, nullptr);
    ASSERT_NE(tx, nullptr);
    EXPECT_NE(rx->ref, tx->ref);
}

// --- Adversarial: invalid opt / unknown device must fail closed, not crash or alias ---

TEST(DmaReqmapUnittest, RejectsNegativeOptIndex)
{
    EXPECT_EQ(dmaGetChannelSpecByPeripheral(DMA_PERIPH_UART_RX, UARTDEV_1, -1), nullptr);
}

TEST(DmaReqmapUnittest, RejectsOptIndexPastTableWidth)
{
    EXPECT_EQ(dmaGetChannelSpecByPeripheral(DMA_PERIPH_UART_RX, UARTDEV_1, MAX_PERIPHERAL_DMA_OPTIONS), nullptr);
}

TEST(DmaReqmapUnittest, RejectsUartDeviceWithNoTableEntry)
{
    // UARTDEV_9/10 have no F4/F7 table rows at all (H7-only entries) -- must return NULL, not a stale/wrong row.
    EXPECT_EQ(dmaGetChannelSpecByPeripheral(DMA_PERIPH_UART_RX, UARTDEV_9, 0), nullptr);
}

// --- Timer table (IT #1396): dmaGetChannelSpecByTimerValue() was a stub returning NULL
// unconditionally on F4/F7 until this table was implemented -- verifies the real per-option
// data, not just that a value comes back. ---

TEST(DmaReqmapUnittest, Tim1Ch1AllThreeOptionsResolveToDistinctStreams)
{
    const dmaChannelSpec_t *opt0 = dmaGetChannelSpecByTimerValue((TIM_TypeDef *)TIM1, TIM_Channel_1, 0);
    const dmaChannelSpec_t *opt1 = dmaGetChannelSpecByTimerValue((TIM_TypeDef *)TIM1, TIM_Channel_1, 1);
    const dmaChannelSpec_t *opt2 = dmaGetChannelSpecByTimerValue((TIM_TypeDef *)TIM1, TIM_Channel_1, 2);

    ASSERT_NE(opt0, nullptr);
    ASSERT_NE(opt1, nullptr);
    ASSERT_NE(opt2, nullptr);
    EXPECT_NE(opt0->ref, opt1->ref);
    EXPECT_NE(opt0->ref, opt2->ref);
    EXPECT_NE(opt1->ref, opt2->ref);
}

TEST(DmaReqmapUnittest, Tim8Ch1SameStreamDifferentChannelOptionsAreDistinguishable)
{
    // TIM8_CH1's two options (DMA(2,2,0) and DMA(2,2,7)) share one physical DMA2_Stream2 --
    // only .channel (the mux selector) tells them apart. A table or lookup bug that ignores
    // .channel would make these two options indistinguishable.
    const dmaChannelSpec_t *opt0 = dmaGetChannelSpecByTimerValue((TIM_TypeDef *)TIM8, TIM_Channel_1, 0);
    const dmaChannelSpec_t *opt1 = dmaGetChannelSpecByTimerValue((TIM_TypeDef *)TIM8, TIM_Channel_1, 1);

    ASSERT_NE(opt0, nullptr);
    ASSERT_NE(opt1, nullptr);
    EXPECT_EQ(opt0->ref, opt1->ref);
    EXPECT_NE(opt0->channel, opt1->channel);
}

TEST(DmaReqmapUnittest, RejectsUnmappedTimerChannel)
{
    // TIM6/TIM7 have no channel-capture DMA entries in the table at all (UP-only on other MCUs).
    EXPECT_EQ(dmaGetChannelSpecByTimerValue((TIM_TypeDef *)TIM3, TIM_Channel_4 + 1, 0), nullptr);
}

TEST(DmaReqmapUnittest, RejectsNegativeTimerOptIndex)
{
    EXPECT_EQ(dmaGetChannelSpecByTimerValue((TIM_TypeDef *)TIM1, TIM_Channel_1, -1), nullptr);
}

TEST(DmaReqmapUnittest, RejectsTimerOptIndexPastTableWidth)
{
    EXPECT_EQ(dmaGetChannelSpecByTimerValue((TIM_TypeDef *)TIM1, TIM_Channel_1, MAX_TIMER_DMA_OPTIONS), nullptr);
}
