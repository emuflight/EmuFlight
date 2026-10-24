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

#if defined(USE_DSHOT_BITBANG) && !defined(USE_HAL_DRIVER)

#include "drivers/dma.h"
#include "drivers/dshot_bitbang_impl.h"
#include "drivers/rcc.h"
#include "drivers/timer.h"

void bbTimerChannelInit(const bbPort_t *bbPort)
{
    TIM_TypeDef *tim = bbPort->tim;

    RCC_ClockCmd(timerRCC(tim), ENABLE);

    TIM_TimeBaseInitTypeDef init;
    TIM_TimeBaseStructInit(&init);
    init.TIM_Prescaler = 0; // feed the raw timer clock
    init.TIM_ClockDivision = TIM_CKD_DIV1;
    init.TIM_CounterMode = TIM_CounterMode_Up;
    init.TIM_Period = bbPort->outputARR;

    TIM_Cmd(tim, DISABLE);
    TIM_TimeBaseInit(tim, &init);
    TIM_ARRPreloadConfig(tim, ENABLE);

    TIM_OCInitTypeDef ocInit;
    TIM_OCStructInit(&ocInit);
    ocInit.TIM_OCMode = TIM_OCMode_PWM1;
    ocInit.TIM_OCIdleState = TIM_OCIdleState_Set;
    ocInit.TIM_OutputState = TIM_OutputState_Enable;
    ocInit.TIM_OCPolarity = TIM_OCPolarity_Low;
    ocInit.TIM_Pulse = 10; // the compare event only clocks the DMA request
    timerOCInit(tim, bbPort->timChannel, &ocInit);

    TIM_Cmd(tim, ENABLE);
}

void bbDMAPreconfigure(bbPort_t *bbPort)
{
    DMA_InitTypeDef init;
    DMA_StructInit(&init);
    init.DMA_Mode = DMA_Mode_Normal;
    init.DMA_Channel = bbPort->dmaChannel;
    init.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    init.DMA_MemoryInc = DMA_MemoryInc_Enable;
    init.DMA_FIFOMode = DMA_FIFOMode_Enable;
    init.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
    init.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    init.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    init.DMA_Priority = DMA_Priority_High;
    init.DMA_DIR = DMA_DIR_MemoryToPeripheral;
    init.DMA_BufferSize = bbPort->portOutputCount;
    init.DMA_PeripheralBaseAddr = (uint32_t)&bbPort->gpio->BSRRL;
    init.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
    init.DMA_Memory0BaseAddr = (uint32_t)bbPort->portOutputBuffer;
    init.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;

    DMA_Stream_TypeDef *stream = bbPort->dmaRef;
    DMA_DeInit(stream);
    DMA_Init(stream, &init);
    DMA_ITConfig(stream, DMA_IT_TC | DMA_IT_TE, ENABLE);

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
    DMA_Cmd(bbPort->dmaRef, enable ? ENABLE : DISABLE);
}

bool bbDMAIsEnabled(const bbPort_t *bbPort)
{
    return (bbPort->dmaRef->CR & DMA_SxCR_EN) != 0;
}

void bbTIMDMACmd(TIM_TypeDef *tim, uint16_t dmaSources, bool enable)
{
    TIM_DMACmd(tim, dmaSources, enable ? ENABLE : DISABLE);
}

#endif // USE_DSHOT_BITBANG && !USE_HAL_DRIVER
