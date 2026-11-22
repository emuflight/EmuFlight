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

#if defined(USE_DSHOT_BITBANG) && defined(AT32F4)

#include "drivers/dma.h"
#include "drivers/dshot_bitbang_impl.h"
#include "drivers/rcc.h"
#include "drivers/timer.h"

void bbTimerChannelInit(const bbPort_t *bbPort)
{
    TIM_TypeDef *tim = bbPort->tim;
    const uint8_t channel = bbPort->timChannel;

    RCC_ClockCmd(timerRCC(tim), ENABLE);

    tmr_counter_enable(tim, FALSE);
    tmr_base_init(tim, bbPort->outputARR, 0); // feed the raw timer clock
    tmr_clock_source_div_set(tim, TMR_CLOCK_DIV1);
    tmr_cnt_dir_set(tim, TMR_COUNT_UP);
    tmr_period_buffer_enable(tim, TRUE);

    tmr_output_config_type ocInit;
    tmr_output_default_para_init(&ocInit);
    ocInit.oc_mode = TMR_OUTPUT_CONTROL_PWM_MODE_A;
    ocInit.oc_idle_state = TRUE;
    ocInit.oc_output_state = TRUE;
    ocInit.oc_polarity = TMR_OUTPUT_ACTIVE_LOW;
    timerOCInit(tim, channel, &ocInit);
    tmr_channel_value_set(tim, TIM_CH_TO_SELCHANNEL(channel), 10); // the compare event only clocks the DMA request
    tmr_channel_enable(tim, TIM_CH_TO_SELCHANNEL(channel), TRUE);
    timerOCPreloadConfig(tim, channel, TRUE);

    tmr_counter_enable(tim, TRUE);
}

void bbDMAPreconfigure(bbPort_t *bbPort)
{
    dma_init_type init;
    dma_default_para_init(&init);
    init.loop_mode_enable = FALSE;
    init.peripheral_inc_enable = FALSE;
    init.memory_inc_enable = TRUE;
    init.priority = DMA_PRIORITY_VERY_HIGH;
    init.direction = DMA_DIR_MEMORY_TO_PERIPHERAL;
    init.buffer_size = bbPort->portOutputCount;
    init.peripheral_base_addr = (uint32_t)&bbPort->gpio->scr;
    init.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_WORD;
    init.memory_base_addr = (uint32_t)bbPort->portOutputBuffer;
    init.memory_data_width = DMA_MEMORY_DATA_WIDTH_WORD;

    dmaMuxEnable(bbPort->dmaIdentifier, bbPort->dmaChannel);
    xDMA_DeInit(bbPort->dmaRef);
    xDMA_Init(bbPort->dmaRef, &init);
    xDMA_ITConfig(bbPort->dmaRef, DMA_FDT_INT | DMA_DTERR_INT, TRUE);
}

void bbDMACmd(const bbPort_t *bbPort, bool enable)
{
    xDMA_Cmd(bbPort->dmaRef, enable ? TRUE : FALSE);
}

void bbTIMDMACmd(TIM_TypeDef *tim, uint16_t dmaSources, bool enable)
{
    tmr_dma_request_enable(tim, dmaSources, enable ? TRUE : FALSE);
}

#endif // USE_DSHOT_BITBANG && AT32F4
