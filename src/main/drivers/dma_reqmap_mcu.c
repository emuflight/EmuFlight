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

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "common/utils.h"

#include "drivers/adc.h"
#include "drivers/bus_spi.h"
#include "drivers/dma_reqmap.h"
#include "drivers/serial.h"
#include "drivers/serial_uart.h"
#include "drivers/timer.h"

typedef struct dmaPeripheralMapping_s {
    dmaPeripheral_e device;
    uint8_t         index;
#if defined(STM32H7)
    uint8_t         dmaRequest;
#else
    dmaChannelSpec_t channelSpec[MAX_PERIPHERAL_DMA_OPTIONS];
#endif
} dmaPeripheralMapping_t;

typedef struct dmaTimerMapping_s {
    TIM_TypeDef *tim;
    uint8_t      channel;
#if defined(STM32H7)
    uint8_t      dmaRequest;
#else
    dmaChannelSpec_t channelSpec[MAX_TIMER_DMA_OPTIONS];
#endif
} dmaTimerMapping_t;

#if defined(STM32H7)

#define REQMAP_DIR(periph, device, dir) { DMA_PERIPH_ ## periph ## _ ## dir, periph ## DEV_ ## device, DMA_REQUEST_ ## periph ## device ## _ ## dir }
#define REQMAP(periph, device)          { DMA_PERIPH_ ## periph, periph ## DEV_ ## device, DMA_REQUEST_ ## periph ## device }
#define REQMAP_TIMUP(periph, timno)     { DMA_PERIPH_TIMUP, timno - 1, DMA_REQUEST_ ## TIM ## timno ## _UP }

// Resolve UART/USART naming
#define DMA_REQUEST_UART1_RX  DMA_REQUEST_USART1_RX
#define DMA_REQUEST_UART1_TX  DMA_REQUEST_USART1_TX
#define DMA_REQUEST_UART2_RX  DMA_REQUEST_USART2_RX
#define DMA_REQUEST_UART2_TX  DMA_REQUEST_USART2_TX
#define DMA_REQUEST_UART3_RX  DMA_REQUEST_USART3_RX
#define DMA_REQUEST_UART3_TX  DMA_REQUEST_USART3_TX
#define DMA_REQUEST_UART6_RX  DMA_REQUEST_USART6_RX
#define DMA_REQUEST_UART6_TX  DMA_REQUEST_USART6_TX

// Resolve SDO/SDI as TX/RX
#define DMA_REQUEST_SPI1_SDO  DMA_REQUEST_SPI1_TX
#define DMA_REQUEST_SPI1_SDI  DMA_REQUEST_SPI1_RX
#define DMA_REQUEST_SPI2_SDO  DMA_REQUEST_SPI2_TX
#define DMA_REQUEST_SPI2_SDI  DMA_REQUEST_SPI2_RX
#define DMA_REQUEST_SPI3_SDO  DMA_REQUEST_SPI3_TX
#define DMA_REQUEST_SPI3_SDI  DMA_REQUEST_SPI3_RX
#define DMA_REQUEST_SPI4_SDO  DMA_REQUEST_SPI4_TX
#define DMA_REQUEST_SPI4_SDI  DMA_REQUEST_SPI4_RX
#define DMA_REQUEST_SPI5_SDO  DMA_REQUEST_SPI5_TX
#define DMA_REQUEST_SPI5_SDI  DMA_REQUEST_SPI5_RX

static const dmaPeripheralMapping_t dmaPeripheralMapping[] = {
#ifdef USE_SPI
    REQMAP_DIR(SPI, 1, SDO),
    REQMAP_DIR(SPI, 1, SDI),
    REQMAP_DIR(SPI, 2, SDO),
    REQMAP_DIR(SPI, 2, SDI),
    REQMAP_DIR(SPI, 3, SDO),
    REQMAP_DIR(SPI, 3, SDI),
    REQMAP_DIR(SPI, 4, SDO),
    REQMAP_DIR(SPI, 4, SDI),
#if defined(STM32H743xx) || defined(STM32H750xx) || defined(STM32H723xx) || defined(STM32H725xx) || defined(STM32H730xx) || defined(STM32H735xx)
    REQMAP_DIR(SPI, 5, SDO),
    REQMAP_DIR(SPI, 5, SDI),
#endif
#endif

#ifdef USE_ADC
    REQMAP(ADC, 1),
    REQMAP(ADC, 2),
#if defined(STM32H743xx) || defined(STM32H750xx) || defined(STM32H723xx) || defined(STM32H725xx) || defined(STM32H730xx) || defined(STM32H735xx)
    REQMAP(ADC, 3),
#endif
#endif

#ifdef USE_UART
    REQMAP_DIR(UART, 1, TX),
    REQMAP_DIR(UART, 1, RX),
    REQMAP_DIR(UART, 2, TX),
    REQMAP_DIR(UART, 2, RX),
    REQMAP_DIR(UART, 3, TX),
    REQMAP_DIR(UART, 3, RX),
    REQMAP_DIR(UART, 4, TX),
    REQMAP_DIR(UART, 4, RX),
    REQMAP_DIR(UART, 5, TX),
    REQMAP_DIR(UART, 5, RX),
    REQMAP_DIR(UART, 6, TX),
    REQMAP_DIR(UART, 6, RX),
    REQMAP_DIR(UART, 7, TX),
    REQMAP_DIR(UART, 7, RX),
    REQMAP_DIR(UART, 8, TX),
    REQMAP_DIR(UART, 8, RX),
#ifdef USE_UART9
    REQMAP_DIR(UART, 9, TX),
    REQMAP_DIR(UART, 9, RX),
#endif
#ifdef USE_UART10
    { DMA_PERIPH_UART_TX, UARTDEV_10, DMA_REQUEST_USART10_TX },
    { DMA_PERIPH_UART_RX, UARTDEV_10, DMA_REQUEST_USART10_RX },
#endif
#endif

#ifdef USE_TIMER
    REQMAP_TIMUP(TIMUP, 1),
    REQMAP_TIMUP(TIMUP, 2),
    REQMAP_TIMUP(TIMUP, 3),
    REQMAP_TIMUP(TIMUP, 4),
    REQMAP_TIMUP(TIMUP, 5),
    REQMAP_TIMUP(TIMUP, 6),
    REQMAP_TIMUP(TIMUP, 7),
    REQMAP_TIMUP(TIMUP, 8),
    REQMAP_TIMUP(TIMUP, 15),
    REQMAP_TIMUP(TIMUP, 16),
    REQMAP_TIMUP(TIMUP, 17),
#endif
};

#undef REQMAP_TIMUP
#undef REQMAP
#undef REQMAP_DIR

// Map CH1..CH4 directly to HAL TIM_CHANNEL_x constants (no timer_def.h needed)
#define TC_CH1  TIM_CHANNEL_1
#define TC_CH2  TIM_CHANNEL_2
#define TC_CH3  TIM_CHANNEL_3
#define TC_CH4  TIM_CHANNEL_4
#define TC(chan) TC_ ## chan
#define REQMAP_TIM(tim, chan) { tim, TC(chan), DMA_REQUEST_ ## tim ## _ ## chan }

static const dmaTimerMapping_t dmaTimerMapping[] = {
    REQMAP_TIM(TIM1, CH1),
    REQMAP_TIM(TIM1, CH2),
    REQMAP_TIM(TIM1, CH3),
    REQMAP_TIM(TIM1, CH4),
    REQMAP_TIM(TIM2, CH1),
    REQMAP_TIM(TIM2, CH2),
    REQMAP_TIM(TIM2, CH3),
    REQMAP_TIM(TIM2, CH4),
    REQMAP_TIM(TIM3, CH1),
    REQMAP_TIM(TIM3, CH2),
    REQMAP_TIM(TIM3, CH3),
    REQMAP_TIM(TIM3, CH4),
    REQMAP_TIM(TIM4, CH1),
    REQMAP_TIM(TIM4, CH2),
    REQMAP_TIM(TIM4, CH3),
    REQMAP_TIM(TIM5, CH1),
    REQMAP_TIM(TIM5, CH2),
    REQMAP_TIM(TIM5, CH3),
    REQMAP_TIM(TIM5, CH4),
    REQMAP_TIM(TIM8, CH1),
    REQMAP_TIM(TIM8, CH2),
    REQMAP_TIM(TIM8, CH3),
    REQMAP_TIM(TIM8, CH4),
    REQMAP_TIM(TIM15, CH1),
    REQMAP_TIM(TIM16, CH1),
    REQMAP_TIM(TIM17, CH1),
};

#undef TC_CH1
#undef TC_CH2
#undef TC_CH3
#undef TC_CH4
#undef TC
#undef REQMAP_TIM

// H7 has 16 DMA streams (DMA1: 0-7, DMA2: 0-7)
#define DMA(d, s) { DMA_CODE(d, s, 0), (dmaResource_t *)DMA ## d ## _Stream ## s, 0 }

static dmaChannelSpec_t dmaChannelSpec[MAX_PERIPHERAL_DMA_OPTIONS] = {
    DMA(1, 0),
    DMA(1, 1),
    DMA(1, 2),
    DMA(1, 3),
    DMA(1, 4),
    DMA(1, 5),
    DMA(1, 6),
    DMA(1, 7),
    DMA(2, 0),
    DMA(2, 1),
    DMA(2, 2),
    DMA(2, 3),
    DMA(2, 4),
    DMA(2, 5),
    DMA(2, 6),
    DMA(2, 7),
};

#undef DMA

static void dmaSetupRequest(dmaChannelSpec_t *dmaSpec, uint8_t request)
{
    dmaSpec->channel = request;
    dmaCode_t code = dmaSpec->code;
    dmaSpec->code = DMA_CODE(DMA_CODE_CONTROLLER(code), DMA_CODE_STREAM(code), dmaSpec->channel);
}

const dmaChannelSpec_t *dmaGetChannelSpecByPeripheral(dmaPeripheral_e device, uint8_t index, int8_t opt)
{
    if (opt < 0 || opt >= MAX_PERIPHERAL_DMA_OPTIONS) {
        return NULL;
    }

    for (unsigned i = 0; i < ARRAYLEN(dmaPeripheralMapping); i++) {
        const dmaPeripheralMapping_t *periph = &dmaPeripheralMapping[i];
        if (periph->device == device && periph->index == index) {
            dmaChannelSpec_t *dmaSpec = &dmaChannelSpec[opt];
            dmaSetupRequest(dmaSpec, periph->dmaRequest);
            return dmaSpec;
        }
    }

    return NULL;
}

dmaoptValue_t dmaoptByTag(ioTag_t ioTag)
{
    UNUSED(ioTag);
    return DMA_OPT_UNUSED;
}

const dmaChannelSpec_t *dmaGetChannelSpecByTimerValue(TIM_TypeDef *tim, uint8_t channel, dmaoptValue_t dmaopt)
{
    if (dmaopt < 0 || dmaopt >= MAX_TIMER_DMA_OPTIONS) {
        return NULL;
    }

    for (unsigned i = 0; i < ARRAYLEN(dmaTimerMapping); i++) {
        const dmaTimerMapping_t *timerMapping = &dmaTimerMapping[i];
        if (timerMapping->tim == tim && timerMapping->channel == channel) {
            dmaChannelSpec_t *dmaSpec = &dmaChannelSpec[dmaopt];
            dmaSetupRequest(dmaSpec, timerMapping->dmaRequest);
            return dmaSpec;
        }
    }

    return NULL;
}

dmaoptValue_t dmaGetOptionByTimer(const timerHardware_t *timer)
{
    for (unsigned opt = 0; opt < ARRAYLEN(dmaChannelSpec); opt++) {
        if ((dmaResource_t *)timer->dmaRef == dmaChannelSpec[opt].ref) {
            return (dmaoptValue_t)opt;
        }
    }

    return DMA_OPT_UNUSED;
}

const dmaChannelSpec_t *dmaGetChannelSpecByTimer(const timerHardware_t *timer)
{
    if (!timer) {
        return NULL;
    }

    dmaoptValue_t dmaopt = dmaGetOptionByTimer(timer);
    return dmaGetChannelSpecByTimerValue(timer->tim, timer->channel, dmaopt);
}

#elif (defined(STM32F4) || defined(STM32F7)) && defined(USE_SPI)

#if defined(STM32F4)
#define DMA(d, s, c) { DMA_CODE(d, s, c), (dmaResource_t *)DMA ## d ## _Stream ## s, DMA_Channel_ ## c }
#elif defined(STM32F7)
#define DMA(d, s, c) { DMA_CODE(d, s, c), (dmaResource_t *)DMA ## d ## _Stream ## s, DMA_CHANNEL_ ## c }
#endif

static const dmaPeripheralMapping_t dmaPeripheralMapping[] = {
#if defined(STM32F745xx) || defined(STM32F746xx) || defined(STM32F765xx)
    { DMA_PERIPH_SPI_SDO, SPIDEV_1, { DMA(2, 5, 3), DMA(2, 3, 3) } },
    { DMA_PERIPH_SPI_SDI, SPIDEV_1, { DMA(2, 2, 3), DMA(2, 0, 3) } },
#else
    { DMA_PERIPH_SPI_SDO, SPIDEV_1, { DMA(2, 3, 3), DMA(2, 5, 3) } },
    { DMA_PERIPH_SPI_SDI, SPIDEV_1, { DMA(2, 0, 3), DMA(2, 2, 3) } },
#endif
    { DMA_PERIPH_SPI_SDO, SPIDEV_2, { DMA(1, 4, 0) } },
    { DMA_PERIPH_SPI_SDI, SPIDEV_2, { DMA(1, 3, 0) } },
    { DMA_PERIPH_SPI_SDO, SPIDEV_3, { DMA(1, 5, 0), DMA(1, 7, 0) } },
    { DMA_PERIPH_SPI_SDI, SPIDEV_3, { DMA(1, 0, 0), DMA(1, 2, 0) } },

#if defined(STM32F411xE) || defined(STM32F745xx) || defined(STM32F746xx) || defined(STM32F765xx) || defined(STM32F722xx)
    { DMA_PERIPH_SPI_SDO, SPIDEV_4, { DMA(2, 1, 4), DMA(2, 4, 5) } },
    { DMA_PERIPH_SPI_SDI, SPIDEV_4, { DMA(2, 0, 4), DMA(2, 3, 5) } },
#endif
};

#undef DMA

const dmaChannelSpec_t *dmaGetChannelSpecByPeripheral(dmaPeripheral_e device, uint8_t index, int8_t opt)
{
    if (opt < 0 || opt >= MAX_PERIPHERAL_DMA_OPTIONS) {
        return NULL;
    }

    for (unsigned i = 0; i < ARRAYLEN(dmaPeripheralMapping); i++) {
        const dmaPeripheralMapping_t *periph = &dmaPeripheralMapping[i];
        if (periph->device == device && periph->index == index && periph->channelSpec[opt].ref) {
            return &periph->channelSpec[opt];
        }
    }

    return NULL;
}

dmaoptValue_t dmaoptByTag(ioTag_t ioTag)
{
    UNUSED(ioTag);
    return DMA_OPT_UNUSED;
}

const dmaChannelSpec_t *dmaGetChannelSpecByTimerValue(TIM_TypeDef *tim, uint8_t channel, dmaoptValue_t dmaopt)
{
    UNUSED(tim);
    UNUSED(channel);
    UNUSED(dmaopt);
    return NULL;
}

const dmaChannelSpec_t *dmaGetChannelSpecByTimer(const timerHardware_t *timer)
{
    UNUSED(timer);
    return NULL;
}

dmaoptValue_t dmaGetOptionByTimer(const timerHardware_t *timer)
{
    UNUSED(timer);
    return DMA_OPT_UNUSED;
}

#else  // F1/F3 or no SPI → stubs

const dmaChannelSpec_t *dmaGetChannelSpecByPeripheral(dmaPeripheral_e device, uint8_t index, int8_t opt)
{
    UNUSED(device);
    UNUSED(index);
    UNUSED(opt);
    return NULL;
}

dmaoptValue_t dmaoptByTag(ioTag_t ioTag)
{
    UNUSED(ioTag);
    return DMA_OPT_UNUSED;
}

const dmaChannelSpec_t *dmaGetChannelSpecByTimerValue(TIM_TypeDef *tim, uint8_t channel, dmaoptValue_t dmaopt)
{
    UNUSED(tim);
    UNUSED(channel);
    UNUSED(dmaopt);
    return NULL;
}

const dmaChannelSpec_t *dmaGetChannelSpecByTimer(const timerHardware_t *timer)
{
    UNUSED(timer);
    return NULL;
}

dmaoptValue_t dmaGetOptionByTimer(const timerHardware_t *timer)
{
    UNUSED(timer);
    return DMA_OPT_UNUSED;
}

#endif
