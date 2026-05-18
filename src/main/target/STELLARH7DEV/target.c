/*
 * This file is part of EmuFlight. It is derived from Betaflight.
 *
 * This is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdint.h>
#include "platform.h"
#include "drivers/io.h"
#include "drivers/dma.h"
#include "drivers/timer.h"
#include "drivers/timer_def.h"

const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    // motors: TIM8 CH1-4, DShot via DMA1_S0-3; burst (TIM8_UP) via DMA2_S0 (upopt=8)
    DEF_TIM(TIM8, CH1, PC6, TIM_USE_MOTOR, 0, 0, 8), // motor 1
    DEF_TIM(TIM8, CH2, PC7, TIM_USE_MOTOR, 0, 1, 8), // motor 2
    DEF_TIM(TIM8, CH3, PC8, TIM_USE_MOTOR, 0, 2, 8), // motor 3
    DEF_TIM(TIM8, CH4, PC9, TIM_USE_MOTOR, 0, 3, 8), // motor 4
    DEF_TIM(TIM4, CH1, PD12, TIM_USE_SERVO, 0, 0, NONE),
    DEF_TIM(TIM4, CH2, PD13, TIM_USE_SERVO, 0, 0, NONE),
    DEF_TIM(TIM2, CH1, PA15, TIM_USE_LED, 0, 13, NONE), // led strip — DMA2_S5 (DMA2_S1-S3 reserved for ADC1-3)
    DEF_TIM(TIM3, CH1, PB4, TIM_USE_ANY, 0, 0, NONE),
};
