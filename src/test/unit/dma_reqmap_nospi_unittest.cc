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

// Compiles dma_reqmap_mcu.c for an F4 target that has USE_ADC but no USE_SPI: only the
// ADC rows exist there, every other peripheral must stay unresolved.

extern "C" {

#include "platform.h"
#include "drivers/adc.h"
#include "drivers/dma_reqmap.h"
#include "drivers/serial.h"
#include "drivers/serial_uart.h"

}

#include "unittest_macros.h"
#include "gtest/gtest.h"

TEST(DmaReqmapNoSpiUnittest, AdcRowsResolveWithoutSpi)
{
    const dmaChannelSpec_t *adc1Opt1 = dmaGetChannelSpecByPeripheral(DMA_PERIPH_ADC, ADCDEV_1, 1);
    ASSERT_NE(adc1Opt1, nullptr);
    EXPECT_EQ(adc1Opt1->ref, (dmaResource_t *)DMA2_Stream4);
    EXPECT_EQ(adc1Opt1->channel, 0u);

    const dmaChannelSpec_t *adc3Opt1 = dmaGetChannelSpecByPeripheral(DMA_PERIPH_ADC, ADCDEV_3, 1);
    ASSERT_NE(adc3Opt1, nullptr);
    EXPECT_EQ(adc3Opt1->ref, (dmaResource_t *)DMA2_Stream1);
    EXPECT_EQ(adc3Opt1->channel, 2u);
}

TEST(DmaReqmapNoSpiUnittest, AdcRejectsUnusedAndOutOfRangeOpt)
{
    EXPECT_EQ(dmaGetChannelSpecByPeripheral(DMA_PERIPH_ADC, ADCDEV_1, DMA_OPT_UNUSED), nullptr);
    EXPECT_EQ(dmaGetChannelSpecByPeripheral(DMA_PERIPH_ADC, ADCDEV_1, MAX_PERIPHERAL_DMA_OPTIONS), nullptr);
    EXPECT_EQ(dmaGetChannelSpecByPeripheral(DMA_PERIPH_ADC, ADCDEV_COUNT, 0), nullptr);
}

TEST(DmaReqmapNoSpiUnittest, NonAdcPeripheralsStayUnresolved)
{
    // Targets without SPI never had UART/SPI DMA rows; enabling ADC rows must not add them.
    EXPECT_EQ(dmaGetChannelSpecByPeripheral(DMA_PERIPH_UART_RX, UARTDEV_1, 0), nullptr);
    EXPECT_EQ(dmaGetChannelSpecByPeripheral(DMA_PERIPH_SPI_SDI, 0, 0), nullptr);
}

TEST(DmaReqmapNoSpiUnittest, SdioStaysUnresolvedWhenTargetHasNoSdio)
{
    EXPECT_EQ(dmaGetChannelSpecByPeripheral(DMA_PERIPH_SDIO, 0, 0), nullptr);
    EXPECT_EQ(dmaGetChannelSpecByPeripheral(DMA_PERIPH_SDIO, 0, 1), nullptr);
}
