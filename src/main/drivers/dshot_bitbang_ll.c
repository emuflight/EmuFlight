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

#if defined(USE_DSHOT_BITBANG) && defined(USE_HAL_DRIVER)

#include "drivers/dma.h"
#include "drivers/dshot_bitbang_impl.h"
#include "drivers/rcc.h"
#include "drivers/timer.h"

void bbTimerChannelInit(const bbPort_t *bbPort)
{
    TIM_TypeDef *tim = bbPort->tim;
    uint32_t llChannel = LL_TIM_CHANNEL_CH1;

    switch (bbPort->timChannel) {
    case TIM_CHANNEL_1: llChannel = LL_TIM_CHANNEL_CH1; break;
    case TIM_CHANNEL_2: llChannel = LL_TIM_CHANNEL_CH2; break;
    case TIM_CHANNEL_3: llChannel = LL_TIM_CHANNEL_CH3; break;
    case TIM_CHANNEL_4: llChannel = LL_TIM_CHANNEL_CH4; break;
    }

    RCC_ClockCmd(timerRCC(tim), ENABLE);

    LL_TIM_InitTypeDef init;
    LL_TIM_StructInit(&init);
    init.Prescaler = 0; // feed the raw timer clock
    init.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
    init.CounterMode = LL_TIM_COUNTERMODE_UP;
    init.Autoreload = bbPort->outputARR;

    LL_TIM_DisableCounter(tim);
    LL_TIM_Init(tim, &init);
    LL_TIM_EnableARRPreload(tim);

    LL_TIM_OC_InitTypeDef ocInit;
    LL_TIM_OC_StructInit(&ocInit);
    ocInit.OCMode = LL_TIM_OCMODE_PWM1;
    ocInit.OCIdleState = LL_TIM_OCIDLESTATE_HIGH;
    ocInit.OCState = LL_TIM_OCSTATE_ENABLE;
    ocInit.OCPolarity = LL_TIM_OCPOLARITY_LOW;
    ocInit.CompareValue = 10; // the compare event only clocks the DMA request
    LL_TIM_OC_Init(tim, llChannel, &ocInit);
    LL_TIM_OC_EnablePreload(tim, llChannel);

    LL_TIM_EnableCounter(tim);
}

void bbDMAPreconfigure(bbPort_t *bbPort)
{
    LL_DMA_InitTypeDef init;
    LL_DMA_StructInit(&init);
    init.Mode = LL_DMA_MODE_NORMAL;
#if defined(STM32H7)
    init.PeriphRequest = bbPort->dmaChannel;
#else
    init.Channel = bbPort->dmaChannel;
#endif
    init.FIFOMode = LL_DMA_FIFOMODE_ENABLE;
    init.FIFOThreshold = LL_DMA_FIFOTHRESHOLD_1_4;
    init.PeriphOrM2MSrcIncMode = LL_DMA_PERIPH_NOINCREMENT;
    init.MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT;
    init.Priority = LL_DMA_PRIORITY_VERYHIGH;
    init.Direction = LL_DMA_DIRECTION_MEMORY_TO_PERIPH;
    init.NbData = bbPort->portOutputCount;
    init.PeriphOrM2MSrcAddress = (uint32_t)&bbPort->gpio->BSRR;
    init.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_WORD;
    init.MemoryOrM2MDstAddress = (uint32_t)bbPort->portOutputBuffer;
    init.MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_WORD;

    DMA_Stream_TypeDef *stream = bbPort->dmaRef;
    LL_EX_DMA_DeInit(stream);
    LL_EX_DMA_Init(stream, &init);
    SET_BIT(stream->CR, DMA_SxCR_TCIE | DMA_SxCR_TEIE);

    bbPort->dmaRegs.CR = stream->CR;
    bbPort->dmaRegs.FCR = stream->FCR;
    bbPort->dmaRegs.NDTR = stream->NDTR;
    bbPort->dmaRegs.PAR = stream->PAR;
    bbPort->dmaRegs.M0AR = stream->M0AR;
}

// The stream registers are rewritten each frame because NDTR is spent and a transfer error may leave the stream half-configured
void bbDMALoadRegs(const bbPort_t *bbPort)
{
    DMA_Stream_TypeDef *stream = bbPort->dmaRef;

    stream->CR = bbPort->dmaRegs.CR;
    stream->FCR = bbPort->dmaRegs.FCR;
    stream->NDTR = bbPort->dmaRegs.NDTR;
    stream->PAR = bbPort->dmaRegs.PAR;
    stream->M0AR = bbPort->dmaRegs.M0AR;
}

void bbDMACmd(const bbPort_t *bbPort, bool enable)
{
    if (enable) {
        LL_EX_DMA_EnableStream(bbPort->dmaRef);
    } else {
        LL_EX_DMA_DisableStream(bbPort->dmaRef);
    }
}

bool bbDMAIsEnabled(const bbPort_t *bbPort)
{
    return (bbPort->dmaRef->CR & DMA_SxCR_EN) != 0;
}

void bbTIMDMACmd(TIM_TypeDef *tim, uint16_t dmaSources, bool enable)
{
    if (enable) {
        SET_BIT(tim->DIER, dmaSources);
    } else {
        CLEAR_BIT(tim->DIER, dmaSources);
    }
}

#endif // USE_DSHOT_BITBANG && USE_HAL_DRIVER
