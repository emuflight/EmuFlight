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
// FILO arrangement for motor assignments, Motor 1 starts at 2nd DECLARATION
    DEF_TIM(TIM2,  CH2,   PB3, TIM_USE_ANY,                0, 0), // pin B03: TIM2 CH2 (AF1) // USE FOR CAMERA CONTROL

    DEF_TIM(TIM4,  CH1,  PB6, TIM_USE_MOTOR,               0, 0), // pin B06: TIM4 CH1 (AF2) // D1-ST0                   MOTOR1
    DEF_TIM(TIM4,  CH2,  PB7, TIM_USE_MOTOR,               0, 0), // pin B07: TIM4 CH2 (AF2) // D1-ST3                   MOTOR2
    DEF_TIM(TIM4,  CH3,  PB8, TIM_USE_MOTOR,               0, 0), // pin B08: TIM4 CH3 (AF2) // D1-ST7                   MOTOR3
    DEF_TIM(TIM8,  CH3,  PC8, TIM_USE_MOTOR,               0, 0), // pin C08: TIM8 CH3 (AF3) // D2-ST2/D2-ST4            MOTOR4
    DEF_TIM(TIM5,  CH2,  PA1, TIM_USE_MOTOR,               0, 0), // pin A01: TIM5 CH2 (AF2) // D1-ST4                   MOTOR5
    DEF_TIM(TIM4,  CH4,  PB9, TIM_USE_MOTOR,               0, 0), // pin B09: TIM4 CH4 (AF2) // NONE  TIM4_UP_D1-ST6     MOTOR6
    DEF_TIM(TIM8,  CH4,  PC9, TIM_USE_MOTOR,               0, 0), // pin C09: TIM8 CH4 (AF3) // D2-ST7                   MOTOR7

    DEF_TIM(TIM3,  CH4, PB1, TIM_USE_MOTOR | TIM_USE_LED,  0, 0), // pin B01: TIM3 CH4 (AF2) // D1-ST2                   LED/MOTOR5

    DEF_TIM(TIM9,  CH2,  PA3, TIM_USE_ANY,                 0, 0), // pin A03: TIM9 CH2 (AF3)
    DEF_TIM(TIM1,  CH2N, PB0, TIM_USE_ANY,                 0, 0), // pin B00: TIM1 CH2N (AF1)
    DEF_TIM(TIM5,  CH1,  PA0, TIM_USE_ANY,                 0, 0), // pin A00: TIM5 CH1 (AF2)
    DEF_TIM(TIM3,  CH1,  PC6, TIM_USE_ANY,                 0, 0), // pin C06: TIM3 CH1 (AF2)
    DEF_TIM(TIM8,  CH2,  PC7, TIM_USE_ANY,                 0, 0), // pin C07: TIM8 CH2 (AF3)
    DEF_TIM(TIM3,  CH2,  PB5, TIM_USE_ANY,                 0, 0), // pin B05: TIM3 CH2 (AF2)

};