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

// Compiles dma_reqmap_mcu.c for an F4 target with USE_SDCARD_SDIO but neither USE_SPI nor
// USE_ADC: the stub table must still carry the SDIO row and nothing else.

extern "C" {

#include "platform.h"
#include "drivers/dma_reqmap.h"
#include "drivers/serial.h"
#include "drivers/serial_uart.h"

}

#include "unittest_macros.h"
#include "gtest/gtest.h"

TEST(DmaReqmapNoSpiSdioUnittest, SdioRowResolvesWithoutSpiOrAdc)
{
    const dmaChannelSpec_t *opt0 = dmaGetChannelSpecByPeripheral(DMA_PERIPH_SDIO, 0, 0);
    ASSERT_NE(opt0, nullptr);
    EXPECT_EQ(opt0->ref, (dmaResource_t *)DMA2_Stream3);
    EXPECT_EQ(opt0->channel, 4u);

    const dmaChannelSpec_t *opt1 = dmaGetChannelSpecByPeripheral(DMA_PERIPH_SDIO, 0, 1);
    ASSERT_NE(opt1, nullptr);
    EXPECT_EQ(opt1->ref, (dmaResource_t *)DMA2_Stream6);
    EXPECT_EQ(opt1->channel, 4u);
}

TEST(DmaReqmapNoSpiSdioUnittest, SdioRejectsUnusedAndOutOfRangeOpt)
{
    EXPECT_EQ(dmaGetChannelSpecByPeripheral(DMA_PERIPH_SDIO, 0, DMA_OPT_UNUSED), nullptr);
    EXPECT_EQ(dmaGetChannelSpecByPeripheral(DMA_PERIPH_SDIO, 0, 2), nullptr);
    EXPECT_EQ(dmaGetChannelSpecByPeripheral(DMA_PERIPH_SDIO, 0, INT8_MIN), nullptr);
    EXPECT_EQ(dmaGetChannelSpecByPeripheral(DMA_PERIPH_SDIO, 1, 0), nullptr);
}

TEST(DmaReqmapNoSpiSdioUnittest, NonSdioPeripheralsStayUnresolved)
{
    EXPECT_EQ(dmaGetChannelSpecByPeripheral(DMA_PERIPH_ADC, 0, 0), nullptr);
    EXPECT_EQ(dmaGetChannelSpecByPeripheral(DMA_PERIPH_UART_RX, 0, 0), nullptr);
}
