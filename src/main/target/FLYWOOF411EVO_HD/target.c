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
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

// This resource file generated using https://github.com/nerdCopter/target-convert
// Commit: 522aa66

#include <stdint.h>
#include "platform.h"
#include "drivers/io.h"
#include "drivers/dma.h"
#include "drivers/timer.h"
#include "drivers/timer_def.h"

const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    DEF_TIM(TIM1, CH1, PA8, TIM_USE_MOTOR, 0, 1), // motor 1
    DEF_TIM(TIM1, CH2, PA9, TIM_USE_MOTOR, 0, 1), // motor 2
    DEF_TIM(TIM1, CH3, PA10, TIM_USE_MOTOR, 0, 1), // motor 3
    DEF_TIM(TIM2, CH3, PB10, TIM_USE_MOTOR, 0, 0), // motor 4
    DEF_TIM(TIM9, CH1, PA2, TIM_USE_PPM, 0, 0), // ppm RX_PPM_PIN; dma 0 assumed, please verify
    DEF_TIM(TIM3, CH3, PB0, TIM_USE_ANY, 0, 0), // could not determine TIM_USE_xxxxx - please check
    DEF_TIM(TIM3, CH1, PB4, TIM_USE_ANY, 0, 0), // could not determine TIM_USE_xxxxx - please check
    DEF_TIM(TIM3, CH4, PB1, TIM_USE_PWM, 0, 0), // pwm RX_PWM1_PIN
    DEF_TIM(TIM3, CH2, PB5, TIM_USE_LED, 0, 0), // led
};

// notice - this file was programmatically generated and may be incomplete.
