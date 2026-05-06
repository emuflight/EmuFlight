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
// Commit: d84474d + 1 file changed, 24 deletions(-)

#include <stdint.h>
#include "platform.h"
#include "drivers/io.h"
#include "drivers/dma.h"
#include "drivers/timer.h"
#include "drivers/timer_def.h"

const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    DEF_TIM(TIM2, CH4, PA3, TIM_USE_MOTOR, 0, 1), // motor 1
    DEF_TIM(TIM3, CH3, PB0, TIM_USE_MOTOR, 0, 0), // motor 2
    DEF_TIM(TIM3, CH4, PB1, TIM_USE_MOTOR, 0, 0), // motor 3
    DEF_TIM(TIM2, CH3, PA2, TIM_USE_MOTOR, 0, 0), // motor 4
    DEF_TIM(TIM4, CH2, PB7, TIM_USE_LED, 0, 0), // led
};

// notice - this file was programmatically generated and may be incomplete.
