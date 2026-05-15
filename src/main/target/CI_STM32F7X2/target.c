/*
 * This file is part of EmuFlight. It is derived from Betaflight.
 *
 * This is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 * You should have received a copy of the GNU General Public
 * License along with this software.
 * If not, see <http://www.gnu.org/licenses/>.
 */

// Synthetic CI target — timer table copied from FOXEERF722V4 (valid F7X2 pins).

#include <stdint.h>
#include "platform.h"
#include "drivers/io.h"
#include "drivers/dma.h"
#include "drivers/timer.h"
#include "drivers/timer_def.h"

const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    DEF_TIM(TIM1, CH2, PA9,  TIM_USE_MOTOR, 0, 0),
    DEF_TIM(TIM1, CH1, PA8,  TIM_USE_MOTOR, 0, 0),
    DEF_TIM(TIM8, CH4, PC9,  TIM_USE_MOTOR, 0, 0),
    DEF_TIM(TIM8, CH3, PC8,  TIM_USE_MOTOR, 0, 0),
    DEF_TIM(TIM4, CH2, PB7,  TIM_USE_PPM,   0, 0),
    DEF_TIM(TIM8, CH1, PC6,  TIM_USE_ANY,   0, 0),
    DEF_TIM(TIM8, CH2, PC7,  TIM_USE_ANY,   0, 0),
    DEF_TIM(TIM2, CH1, PA15, TIM_USE_LED,   0, 0),
    DEF_TIM(TIM2, CH2, PB3,  TIM_USE_ANY,   0, 0),
};
