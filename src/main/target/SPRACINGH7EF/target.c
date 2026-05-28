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
// Commit: d7caf4a

#include <stdint.h>
#include "platform.h"
#include "drivers/io.h"
#include "drivers/dma.h"
#include "drivers/timer.h"
#include "drivers/timer_def.h"

const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    DEF_TIM(TIM5, CH1, PA0, TIM_USE_MOTOR, 0, 0, NONE), // motor 1
    DEF_TIM(TIM5, CH2, PA1, TIM_USE_MOTOR, 0, 1, NONE), // motor 2
    DEF_TIM(TIM5, CH3, PA2, TIM_USE_MOTOR, 0, 2, NONE), // motor 3
    DEF_TIM(TIM5, CH4, PA3, TIM_USE_MOTOR, 0, 3, NONE), // motor 4
    DEF_TIM(TIM3, CH1, PA6, TIM_USE_MOTOR, 0, 4, NONE), // motor 5
    DEF_TIM(TIM3, CH2, PA7, TIM_USE_MOTOR, 0, 5, NONE), // motor 6
    DEF_TIM(TIM3, CH3, PB0, TIM_USE_MOTOR, 0, 6, NONE), // motor 7
    DEF_TIM(TIM3, CH4, PB1, TIM_USE_MOTOR, 0, 7, NONE), // motor 8
    DEF_TIM(TIM16, CH1, PB8, TIM_USE_LED, 0, 8, NONE), // led strip
    DEF_TIM(TIM17, CH1, PB9, TIM_USE_CAMERA_CONTROL, 0, 9, NONE), // camera control — confirmed TIM17_CH1/PB9 per BF SPRACINGH7EF config.h
};

// notice - this file was programmatically generated and may be incomplete.
