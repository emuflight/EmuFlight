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

#include <stdint.h>

#include "platform.h"

#include "drivers/rcc_types.h"
#include "drivers/serial.h"
#include "drivers/serial_uart.h"

#ifdef USE_UART

#include "common/utils.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/serial_uart.h"

#include "drivers/dma_reqmap.h"
#include "drivers/serial_uart_impl.h"

PG_REGISTER_ARRAY_WITH_RESET_FN(serialUartConfig_t, UARTDEV_COUNT_MAX, serialUartConfig, PG_SERIAL_UART_CONFIG, 0);

// F4/F7 serialUART() resolves DMA via serialUartConfig()/dmaGetChannelSpecByPeripheral();
// H7's serial_uart_stm32h7xx.c uses its own pre-existing UARTn_{TX,RX}_DMA_STREAM macros
// and never reads this PG array for UART -- no per-UART default table exists for it here,
// only the blanket DMA_OPT_UNUSED below. Neither F4 nor F7 wire up UARTDEV_9/10 in their
// uartHardware[] tables, so those two are intentionally absent from this table.
#if defined(STM32F4) || defined(STM32F7)
typedef struct uartDmaopt_s {
    UARTDevice_e device;
    int8_t txDmaopt;
    int8_t rxDmaopt;
} uartDmaopt_t;

static const uartDmaopt_t uartDmaoptDefault[] = {
#ifdef USE_UART1
    { UARTDEV_1, UART1_TX_DMA_OPT, UART1_RX_DMA_OPT },
#endif
#ifdef USE_UART2
    { UARTDEV_2, UART2_TX_DMA_OPT, UART2_RX_DMA_OPT },
#endif
#ifdef USE_UART3
    { UARTDEV_3, UART3_TX_DMA_OPT, UART3_RX_DMA_OPT },
#endif
#ifdef USE_UART4
    { UARTDEV_4, UART4_TX_DMA_OPT, UART4_RX_DMA_OPT },
#endif
#ifdef USE_UART5
    { UARTDEV_5, UART5_TX_DMA_OPT, UART5_RX_DMA_OPT },
#endif
#ifdef USE_UART6
    { UARTDEV_6, UART6_TX_DMA_OPT, UART6_RX_DMA_OPT },
#endif
#ifdef USE_UART7
    { UARTDEV_7, UART7_TX_DMA_OPT, UART7_RX_DMA_OPT },
#endif
#ifdef USE_UART8
    { UARTDEV_8, UART8_TX_DMA_OPT, UART8_RX_DMA_OPT },
#endif
};
#endif // STM32F4 || STM32F7

void pgResetFn_serialUartConfig(serialUartConfig_t *config)
{
    for (unsigned i = 0; i < UARTDEV_COUNT_MAX; i++) {
        config[i].txDmaopt = DMA_OPT_UNUSED;
        config[i].rxDmaopt = DMA_OPT_UNUSED;
    }

#if defined(STM32F4) || defined(STM32F7)
    for (unsigned i = 0; i < ARRAYLEN(uartDmaoptDefault); i++) {
        const int device = uartDmaoptDefault[i].device;
        config[device].txDmaopt = uartDmaoptDefault[i].txDmaopt;
        config[device].rxDmaopt = uartDmaoptDefault[i].rxDmaopt;
    }
#endif
}

#endif
