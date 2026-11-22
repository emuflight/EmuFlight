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
#include <math.h>

#include "platform.h"

#ifdef USE_DSHOT

#include "drivers/dma.h"
#include "drivers/dma_reqmap.h"
#include "drivers/io.h"
#include "drivers/nvic.h"
#include "drivers/rcc.h"
#include "drivers/time.h"
#include "drivers/timer.h"
#include "drivers/pwm_output.h"

static uint8_t dmaMotorTimerCount = 0;
static motorDmaTimer_t dmaMotorTimers[MAX_DMA_TIMERS];
static motorDmaOutput_t dmaMotors[MAX_SUPPORTED_MOTORS];

motorDmaOutput_t *getMotorDmaOutput(uint8_t index)
{
    return &dmaMotors[index];
}

// BF timer channel (1 based) to the AT32 channel selector; the complementary outputs exist for channels 1 to 3 only.
static tmr_channel_select_type toCHSelectType(const uint8_t channel, const bool useNChannel)
{
    if (useNChannel) {
        switch (channel) {
        case 1:
            return TMR_SELECT_CHANNEL_1C;
        case 2:
            return TMR_SELECT_CHANNEL_2C;
        default:
            return TMR_SELECT_CHANNEL_3C;
        }
    }

    switch (channel) {
    case 1:
        return TMR_SELECT_CHANNEL_1;
    case 2:
        return TMR_SELECT_CHANNEL_2;
    case 3:
        return TMR_SELECT_CHANNEL_3;
    default:
        return TMR_SELECT_CHANNEL_4;
    }
}

uint8_t getTimerIndex(TIM_TypeDef *timer)
{
    for (int i = 0; i < dmaMotorTimerCount; i++) {
        if (dmaMotorTimers[i].timer == timer) {
            return i;
        }
    }
    dmaMotorTimers[dmaMotorTimerCount++].timer = timer;
    return dmaMotorTimerCount - 1;
}

FAST_CODE void pwmWriteDshotInt(uint8_t index, uint16_t value)
{
    motorDmaOutput_t *const motor = &dmaMotors[index];

    if (!motor->configured) {
        return;
    }

    /*If there is a command ready to go overwrite the value and send that instead*/
    if (pwmDshotCommandIsProcessing()) {
        value = pwmGetDshotCommand(index);
        if (value) {
            motor->requestTelemetry = true;
        }
    }

    motor->value = value;

    const uint16_t packet = prepareDshotPacket(motor);
    const uint8_t bufferSize = loadDmaBuffer(motor->dmaBuffer, 1, packet);

    motor->timer->timerDmaSources |= motor->timerDmaSource;
    xDMA_SetCurrDataCounter(motor->dmaRef, bufferSize);
    xDMA_Cmd(motor->dmaRef, TRUE);
}

FAST_CODE void pwmCompleteDshotMotorUpdate(uint8_t motorCount)
{
    /* If there is a dshot command loaded up, time it correctly with motor update*/
    if (pwmDshotCommandIsQueued()) {
        if (!pwmDshotCommandOutputIsEnabled(motorCount)) {
            return;
        }
    }

    for (int i = 0; i < dmaMotorTimerCount; i++) {
        // Reset the counter so the first pulse has the full width.
        tmr_counter_value_set(dmaMotorTimers[i].timer, 0);

        // Ensure that overflow events are enabled.
        tmr_overflow_event_disable(dmaMotorTimers[i].timer, FALSE);

        // Generate requests from the timer to the DMA channels of the motors written this cycle.
        tmr_dma_request_enable(dmaMotorTimers[i].timer, dmaMotorTimers[i].timerDmaSources, TRUE);

        dmaMotorTimers[i].timerDmaSources = 0;
    }
}

FAST_CODE static void motor_DMA_IRQHandler(dmaChannelDescriptor_t *descriptor)
{
    if (DMA_GET_FLAG_STATUS(descriptor, DMA_IT_TCIF)) {
        motorDmaOutput_t * const motor = &dmaMotors[descriptor->userParam];

        // Disable the requests before the channel: a pending request would fire the moment the
        // channel is re-enabled and send fake pulses at the start of the next packet.
        tmr_dma_request_enable(motor->timerHardware->tim, motor->timerDmaSource, FALSE);
        xDMA_Cmd(motor->dmaRef, FALSE);

        DMA_CLEAR_FLAG(descriptor, DMA_IT_TCIF);
    }
}

bool pwmDshotMotorHardwareConfig(const timerHardware_t *timerHardware, uint8_t motorIndex, uint8_t reorderedMotorIndex, motorPwmProtocolTypes_e pwmProtocolType, uint8_t output)
{
    const dmaChannelSpec_t *dmaSpec = dmaGetChannelSpecByTimer(timerHardware);
    if (dmaSpec == NULL || dmaSpec->ref == NULL) {
        return false;
    }

    dmaResource_t *dmaRef = dmaSpec->ref;
    const uint32_t dmaMuxId = dmaSpec->channel;
    const dmaIdentifier_e dmaIdentifier = dmaGetIdentifier(dmaRef);

    if (!dmaAllocate(dmaIdentifier, OWNER_MOTOR, RESOURCE_INDEX(reorderedMotorIndex))) {
        return false;
    }

    motorDmaOutput_t * const motor = &dmaMotors[motorIndex];
    TIM_TypeDef *timer = timerHardware->tim;
    const uint8_t channel = timerHardware->channel;
    const bool useNChannel = (output & TIMER_OUTPUT_N_CHANNEL) != 0;

    // configureTimer is true for every channel of a timer processed consecutively;
    // the timer set-up is idempotent, so repeating it per channel is harmless.
    const uint8_t timerIndex = getTimerIndex(timer);
    const bool configureTimer = (timerIndex == dmaMotorTimerCount - 1);

    motor->timer = &dmaMotorTimers[timerIndex];
    motor->timerHardware = timerHardware;
    motor->dmaRef = dmaRef;

    const IO_t motorIO = IOGetByTag(timerHardware->tag);
    const uint8_t pupMode = (output & TIMER_OUTPUT_INVERTED) ? GPIO_PULL_DOWN : GPIO_PULL_UP;
    IOConfigGPIOAF(motorIO, IO_CONFIG(GPIO_MODE_MUX, GPIO_DRIVE_STRENGTH_MODERATE, GPIO_OUTPUT_PUSH_PULL, pupMode), timerHardware->alternateFunction);

    if (configureTimer) {
        RCC_ClockCmd(timerRCC(timer), ENABLE);

        tmr_counter_enable(timer, FALSE);

        const uint32_t prescaler = (uint16_t)(lrintf((float) timerClock(timer) / getDshotHz(pwmProtocolType) + 0.01f) - 1);
        const uint32_t period = (pwmProtocolType == PWM_TYPE_PROSHOT1000 ? MOTOR_NIBBLE_LENGTH_PROSHOT : MOTOR_BITLENGTH) - 1;

        tmr_clock_source_div_set(timer, TMR_CLOCK_DIV1);
        tmr_repetition_counter_set(timer, 0);
        tmr_cnt_dir_set(timer, TMR_COUNT_UP);
        tmr_base_init(timer, period, prescaler);
    }

    tmr_output_config_type ocConfig;
    tmr_output_default_para_init(&ocConfig);
    ocConfig.oc_mode = TMR_OUTPUT_CONTROL_PWM_MODE_A;
    if (useNChannel) {
        ocConfig.occ_output_state = TRUE;
        ocConfig.occ_idle_state = FALSE;
        ocConfig.occ_polarity = (output & TIMER_OUTPUT_INVERTED) ? TMR_OUTPUT_ACTIVE_LOW : TMR_OUTPUT_ACTIVE_HIGH;
    } else {
        ocConfig.oc_output_state = TRUE;
        ocConfig.oc_idle_state = FALSE;
        ocConfig.oc_polarity = (output & TIMER_OUTPUT_INVERTED) ? TMR_OUTPUT_ACTIVE_LOW : TMR_OUTPUT_ACTIVE_HIGH;
    }

    motor->timerDmaSource = timerDmaSource(channel);
    // timerDmaSources holds one bit per motor channel of a shared timer; clear this motor's bit.
    motor->timer->timerDmaSources &= ~motor->timerDmaSource;

    xDMA_Cmd(dmaRef, FALSE);
    xDMA_DeInit(dmaRef);

    dmaEnable(dmaIdentifier);
    dmaMuxEnable(dmaIdentifier, dmaMuxId);

    dma_init_type dmaInit;
    dma_default_para_init(&dmaInit);
    dmaInit.memory_base_addr = (uint32_t)motor->dmaBuffer;
    dmaInit.memory_inc_enable = TRUE;
    dmaInit.memory_data_width = DMA_MEMORY_DATA_WIDTH_WORD;
    dmaInit.loop_mode_enable = FALSE;
    dmaInit.buffer_size = (pwmProtocolType == PWM_TYPE_PROSHOT1000) ? PROSHOT_DMA_BUFFER_SIZE : DSHOT_DMA_BUFFER_SIZE;
    dmaInit.direction = DMA_DIR_MEMORY_TO_PERIPHERAL;
    dmaInit.peripheral_base_addr = (uint32_t)timerChCCR(timerHardware); // address of the channel's CxDT register
    dmaInit.peripheral_inc_enable = FALSE;
    dmaInit.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_WORD;
    dmaInit.priority = DMA_PRIORITY_HIGH;

    // Disable the preload buffer so that CxDT can be written immediately.
    timerOCPreloadConfig(timer, channel, FALSE);
    tmr_channel_enable(timer, toCHSelectType(channel, useNChannel), FALSE);

    // The AT32 SDK does not put the channel in output mode itself.
    switch (channel) {
    case 1:
        timer->cm1_output_bit.c1c = 0;
        break;
    case 2:
        timer->cm1_output_bit.c2c = 0;
        break;
    case 3:
        timer->cm2_output_bit.c3c = 0;
        break;
    case 4:
        timer->cm2_output_bit.c4c = 0;
        break;
    }

    timerOCInit(timer, channel, &ocConfig);
    tmr_channel_value_set(timer, toCHSelectType(channel, false), 0); // tmr_channel_value_set takes the normal channels only
    tmr_channel_enable(timer, toCHSelectType(channel, useNChannel), TRUE);
    timerOCPreloadConfig(timer, channel, TRUE);

    xDMA_Init(dmaRef, &dmaInit);

    // Interrupt when the transfer is complete
    xDMA_ITConfig(dmaRef, DMA_FDT_INT, TRUE);

    dmaSetHandler(dmaIdentifier, motor_DMA_IRQHandler, NVIC_BUILD_PRIORITY(1, 2), motorIndex);

    if (configureTimer) {
        // Changes to the period register are deferred until the next counter overflow
        tmr_period_buffer_enable(timer, TRUE);
        tmr_output_enable(timer, TRUE);
        tmr_counter_enable(timer, TRUE);
    }

    motor->configured = true;

    return true;
}

#endif
