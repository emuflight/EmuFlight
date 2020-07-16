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

    DEF_TIM(TIM2,  CH2,   PB3, TIM_USE_ANY  | TIM_USE_LED,     0, 0), // USE FOR CAMERA CONTROL

    DEF_TIM(TIM4,  CH1,  PB6, TIM_USE_MOTOR,               0, 0), // D1-ST0                   MOTOR1
    DEF_TIM(TIM4,  CH2,  PB7, TIM_USE_MOTOR,               0, 0), // D1-ST3                   MOTOR2
    DEF_TIM(TIM4,  CH3,  PB8, TIM_USE_MOTOR,               0, 0), // D1-ST7                   MOTOR3
    DEF_TIM(TIM4,  CH4,  PB9, TIM_USE_MOTOR,               0, 0), // NONE  TIM4_UP_D1-ST6     MOTOR4
    DEF_TIM(TIM5,  CH2,  PA1, TIM_USE_MOTOR,               0, 0), // D1-ST4                   MOTOR5
    DEF_TIM(TIM8,  CH3,  PC8, TIM_USE_MOTOR,               0, 0), // D2-ST2/D2-ST4            MOTOR6
    DEF_TIM(TIM8,  CH4,  PC9, TIM_USE_MOTOR,               0, 0), // D2-ST7                   MOTOR7

    DEF_TIM(TIM3,  CH4, PB1, TIM_USE_MOTOR | TIM_USE_LED,  0, 0), // D1-ST2                   LED/MOTOR5


};
