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

// Compiles the real dma_reqmap_mcu.c's STM32H7 branch under UNIT_TEST -- a different
// code path from the F4/F7 branch (DMAMUX request-code table vs. channelSpec arrays).

extern "C" {

#include "platform.h"
#include "drivers/dma_reqmap.h"
#include "drivers/serial.h"
#include "drivers/serial_uart.h"

}

#include "unittest_macros.h"
#include "gtest/gtest.h"

// H7's result aliases a shared opt-indexed scratch array; read fields out immediately,
// matching spiInitBusDMA() (bus_spi.c:326-335) -- see ResultAliasesSharedPerOptSlotAcrossCalls below.
TEST(DmaReqmapH7Unittest, Uart1RxAndTxResolveToDistinctRequestCodes)
{
    const dmaChannelSpec_t *rx = dmaGetChannelSpecByPeripheral(DMA_PERIPH_UART_RX, UARTDEV_1, 0);
    ASSERT_NE(rx, nullptr);
    const uint32_t rxChannel = rx->channel;

    const dmaChannelSpec_t *tx = dmaGetChannelSpecByPeripheral(DMA_PERIPH_UART_TX, UARTDEV_1, 0);
    ASSERT_NE(tx, nullptr);

    EXPECT_NE(rxChannel, tx->channel); // DMA_REQUEST_USART1_RX != _TX
}

TEST(DmaReqmapH7Unittest, DifferentOptsResolveToDifferentStreamsButSameRequestCode)
{
    // H7's DMAMUX: opt only selects the stream, unlike F4/F7's fixed per-stream table.
    const dmaChannelSpec_t *opt0 = dmaGetChannelSpecByPeripheral(DMA_PERIPH_UART_TX, UARTDEV_3, 0);
    ASSERT_NE(opt0, nullptr);
    const dmaResource_t *opt0Ref = opt0->ref;
    const uint32_t opt0Channel = opt0->channel;

    const dmaChannelSpec_t *opt1 = dmaGetChannelSpecByPeripheral(DMA_PERIPH_UART_TX, UARTDEV_3, 1);
    ASSERT_NE(opt1, nullptr);

    EXPECT_NE(opt0Ref, opt1->ref);
    EXPECT_EQ(opt0Channel, opt1->channel); // same peripheral request, different stream
}

TEST(DmaReqmapH7Unittest, ResultAliasesSharedPerOptSlotAcrossCalls)
{
    // adversarial: a live pointer from an earlier call mutates once a later call reuses its opt index.
    const dmaChannelSpec_t *rx = dmaGetChannelSpecByPeripheral(DMA_PERIPH_UART_RX, UARTDEV_1, 0);
    ASSERT_NE(rx, nullptr);
    ASSERT_EQ(rx->channel, static_cast<uint32_t>(DMA_REQUEST_USART1_RX));

    dmaGetChannelSpecByPeripheral(DMA_PERIPH_UART_TX, UARTDEV_1, 0); // same opt index

    EXPECT_EQ(rx->channel, static_cast<uint32_t>(DMA_REQUEST_USART1_TX)); // rx's slot was overwritten
}

TEST(DmaReqmapH7Unittest, RejectsNegativeOptIndex)
{
    EXPECT_EQ(dmaGetChannelSpecByPeripheral(DMA_PERIPH_UART_RX, UARTDEV_1, -1), nullptr);
}

TEST(DmaReqmapH7Unittest, RejectsOptIndexPastTableWidth)
{
    EXPECT_EQ(dmaGetChannelSpecByPeripheral(DMA_PERIPH_UART_RX, UARTDEV_1, MAX_PERIPHERAL_DMA_OPTIONS), nullptr);
}

TEST(DmaReqmapH7Unittest, RejectsUnmappedPeripheral)
{
    // adversarial: TIMUP requires USE_TIMER (undefined here); must fail closed, not read past the table.
    EXPECT_EQ(dmaGetChannelSpecByPeripheral(DMA_PERIPH_TIMUP, 0, 0), nullptr);
}
