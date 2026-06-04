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
// Commit: 215ae87 + 1 file changed, 31 insertions(+), 8 deletions(-)

#include <stdint.h>
#include "platform.h"
#include "drivers/io.h"
#include "drivers/dma.h"
#include "drivers/timer.h"
#include "drivers/timer_def.h"

const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    DEF_TIM(TIM1, CH1, PE9, TIM_USE_MOTOR, 0, 0, NONE), // motor 1
    DEF_TIM(TIM1, CH2, PE11, TIM_USE_MOTOR, 0, 1, NONE), // motor 2
    DEF_TIM(TIM1, CH3, PE13, TIM_USE_MOTOR, 0, 2, NONE), // motor 3
    DEF_TIM(TIM1, CH4, PE14, TIM_USE_MOTOR, 0, 3, NONE), // motor 4
    DEF_TIM(TIM3, CH1, PA6, TIM_USE_MOTOR, 0, 5, NONE), // motor 5
    DEF_TIM(TIM3, CH2, PA7, TIM_USE_MOTOR, 0, 6, NONE), // motor 6
    DEF_TIM(TIM8, CH3, PC8, TIM_USE_MOTOR, 0, 7, NONE), // motor 7
    DEF_TIM(TIM8, CH4, PC9, TIM_USE_MOTOR, 0, 8, NONE), // motor 8
    DEF_TIM(TIM5, CH1, PA0, TIM_USE_LED, 0, 9, NONE), // led strip
    DEF_TIM(TIM2, CH1, PA5, TIM_USE_ANY, 0, 0, NONE), // could not determine TIM_USE_xxxxx - please check; dma -1 in config (input only)
};

// notice - this file was programmatically generated and may be incomplete.
