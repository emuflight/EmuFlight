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
#include "drivers/timer.h"

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
    // TIM3 is in the table, but a channel value past TIM_Channel_4 has no entry.
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

// --- dmaGetOptionByTimer()/dmaGetChannelSpecByTimer(): exercise the actual timerHardware_t
// matching path CodeRabbit flagged as untested -- the tests above only call
// dmaGetChannelSpecByTimerValue() with a hand-picked option index, never resolving that index
// from a timerHardware_t the way real call sites do. ---

TEST(DmaReqmapUnittest, GetOptionByTimerDisambiguatesSameStreamDifferentChannel)
{
    // TIM8_CH1's two table options (DMA(2,2,0) and DMA(2,2,7)) share DMA2_Stream2 -- only
    // .channel differs. A timer already configured with dmaChannel=7 must resolve to option 1,
    // not option 0, even though both share the same .ref.
    timerHardware_t timerOpt0 = {};
    timerOpt0.tim = (TIM_TypeDef *)TIM8;
    timerOpt0.channel = TIM_Channel_1;
    timerOpt0.dmaRef = (DMA_Stream_TypeDef *)DMA2_Stream2;
    timerOpt0.dmaChannel = 0;

    timerHardware_t timerOpt1 = timerOpt0;
    timerOpt1.dmaChannel = 7;

    EXPECT_EQ(dmaGetOptionByTimer(&timerOpt0), 0);
    EXPECT_EQ(dmaGetOptionByTimer(&timerOpt1), 1);

    const dmaChannelSpec_t *spec0 = dmaGetChannelSpecByTimer(&timerOpt0);
    const dmaChannelSpec_t *spec1 = dmaGetChannelSpecByTimer(&timerOpt1);
    ASSERT_NE(spec0, nullptr);
    ASSERT_NE(spec1, nullptr);
    EXPECT_EQ(spec0->ref, spec1->ref);
    EXPECT_NE(spec0->channel, spec1->channel);
    EXPECT_EQ(spec0->channel, 0u);
    EXPECT_EQ(spec1->channel, 7u);
}

TEST(DmaReqmapUnittest, GetOptionByTimerRejectsNullTimer)
{
    EXPECT_EQ(dmaGetOptionByTimer(nullptr), DMA_OPT_UNUSED);
    EXPECT_EQ(dmaGetChannelSpecByTimer(nullptr), nullptr);
}

TEST(DmaReqmapUnittest, GetOptionByTimerIgnoresZeroInitializedSlots)
{
    // TIM4_CH2 has only one populated table option (DMA(1,3,2)) -- options 1 and 2 are
    // zero-initialized (.ref == NULL, .channel == 0). A timer with dmaRef == NULL and
    // dmaChannel == 0 must not falsely match one of those empty slots.
    timerHardware_t timer = {};
    timer.tim = (TIM_TypeDef *)TIM4;
    timer.channel = TIM_Channel_2;
    timer.dmaRef = nullptr;
    timer.dmaChannel = 0;

    EXPECT_EQ(dmaGetOptionByTimer(&timer), DMA_OPT_UNUSED);
    EXPECT_EQ(dmaGetChannelSpecByTimer(&timer), nullptr);
}
