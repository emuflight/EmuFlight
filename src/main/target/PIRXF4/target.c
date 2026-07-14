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

#include <stdint.h>

#include "platform.h"
#include "drivers/io.h"

#include "drivers/dma.h"
#include "drivers/timer.h"
#include "drivers/timer_def.h"

const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    DEF_TIM(TIM3, CH1, PB4,  TIM_USE_MOTOR, 0, 0 ),                  // S1_OUT - TIM3_UP - BURST
    DEF_TIM(TIM3, CH2, PB5,  TIM_USE_MOTOR, 0, 0 ),                  // S2_OUT - TIM3_UP - BURST
    DEF_TIM(TIM4, CH3, PB8,  TIM_USE_MOTOR, 0, 0 ),                  // S3_OUT - TIM2_UP - BURST
    DEF_TIM(TIM4, CH4, PB9,  TIM_USE_MOTOR, 0, 0 ),                  // S4_OUT - TIM2_UP - BURST
};
