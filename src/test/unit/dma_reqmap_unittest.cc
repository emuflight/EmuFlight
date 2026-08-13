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

// Compiles the real src/main/drivers/dma_reqmap_mcu.c (F4/F7 branch) under UNIT_TEST.
// dmaGetChannelSpecByPeripheral() is a pure static-table lookup here -- no CMSIS register
// access at all -- so it needs no register mocking, only fake DMAx_Streamy pointer/channel
// literals to stand in for the real stream identities the table is built from.
// F7 shares this exact source branch (`(STM32F4 || STM32F7) && USE_SPI`), so an F4 compile
// exercises the same logic F7 would; see the analogous reasoning already recorded for
// serial_uart_dma_claim_unittest.cc in fix/dmainit-ownership-check/CONTEXT.

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
    // UARTDEV_1 RX has two known-valid streams (see dma_reqmap_mcu.c); opt 1 must resolve
    // to a stream distinct from opt 0, giving serialUART() a real fallback to try.
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
