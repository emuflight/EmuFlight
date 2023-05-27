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
 
#include <stdint.h>
#include "platform.h"
#include "drivers/io.h"
#include "drivers/dma.h"
#include "drivers/timer.h"
#include "drivers/timer_def.h"

const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
/* notice - incomplete */
// format : DEF_TIM(TIMxx, CHx, Pxx, TIM_USE_xxxxxxx, x, x), //comment
    DEF_TIM(TIM9, CH2, PA3, TIM_USE_PPM, 0, 0), //LED0

    DEF_TIM(TIM3, CH3, PB0, TIM_USE_MOTOR, 0, 0), // #define MOTOR1_PIN PB0
    DEF_TIM(TIM3, CH4, PB1, TIM_USE_MOTOR, 0, 0), // #define MOTOR2_PIN PB1
    DEF_TIM(TIM3, CH1, PB4, TIM_USE_MOTOR, 0, 0), // #define MOTOR3_PIN PB4
    DEF_TIM(TIM2, CH2, PB3, TIM_USE_MOTOR, 0, 0), // #define MOTOR4_PIN PB3
    DEF_TIM(TIM8, CH4, PC9, TIM_USE_MOTOR, 0, 0), // #define MOTOR5_PIN PC9
    DEF_TIM(TIM8, CH3, PC8, TIM_USE_MOTOR, 0, 0), // #define MOTOR6_PIN PC8

    DEF_TIM(TIM1, CH1, PA8, TIM_USE_LED, 0, 0), //LED_STRIP
    DEF_TIM(TIM4, CH3, PB8, TIM_USE_ANY, 0, 0), //CAM_CONTROL

};

// existing src/main/target/JHEF7DUAL contains TIM_USE_PPM, not TIM_USE_LED for TIM9-CH2-PA3

// TIM_USE options:
// TIM_USE_ANY
// TIM_USE_BEEPER
// TIM_USE_LED
// TIM_USE_MOTOR
// TIM_USE_NONE
// TIM_USE_PPM
// TIM_USE_PWM
// TIM_USE_SERVO
// TIM_USE_TRANSPONDER

// config.h resources:
// #define MOTOR1_PIN           PB0
// #define MOTOR2_PIN           PB1
// #define MOTOR3_PIN           PB4
// #define MOTOR4_PIN           PB3
// #define MOTOR5_PIN           PC9
// #define MOTOR6_PIN           PC8
// #define TIMER_PIN_MAPPING \
//     TIMER_PIN_MAP( 0, PA3 , 3, -1) \
//     TIMER_PIN_MAP( 1, PB0 , 2,  0) \
//     TIMER_PIN_MAP( 2, PB1 , 2,  0) \
//     TIMER_PIN_MAP( 3, PB4 , 1,  0) \
//     TIMER_PIN_MAP( 4, PB3 , 1,  0) \
//     TIMER_PIN_MAP( 5, PC9 , 2,  0) \
//     TIMER_PIN_MAP( 6, PC8 , 2,  0) \
//     TIMER_PIN_MAP( 7, PA8 , 1,  0) \
//     TIMER_PIN_MAP( 8, PB8 , 1,  0)


// unified-target:
// # timer
// timer A03 AF3
// # pin A03: TIM9 CH2 (AF3)

// timer B00 AF2
// # pin B00: TIM3 CH3 (AF2)
// timer B01 AF2
// # pin B01: TIM3 CH4 (AF2)
// timer B04 AF2
// # pin B04: TIM3 CH1 (AF2)
// timer B03 AF1
// # pin B03: TIM2 CH2 (AF1)
// timer C09 AF3
// # pin C09: TIM8 CH4 (AF3)
// timer C08 AF3
// # pin C08: TIM8 CH3 (AF3)

// timer A08 AF1
// # pin A08: TIM1 CH1 (AF1)
// timer B08 AF2
// # pin B08: TIM4 CH3 (AF2)
// 
// # dma
// dma ADC 3 1
// # ADC 3: DMA2 Stream 1 Channel 2
// dma pin B00 0
// # pin B00: DMA1 Stream 7 Channel 5
// dma pin B01 0
// # pin B01: DMA1 Stream 2 Channel 5
// dma pin B04 0
// # pin B04: DMA1 Stream 4 Channel 5
// dma pin B03 0
// # pin B03: DMA1 Stream 6 Channel 3
// dma pin C09 0
// # pin C09: DMA2 Stream 7 Channel 7
// dma pin C08 0
// # pin C08: DMA2 Stream 2 Channel 0
// dma pin A08 0
// # pin A08: DMA2 Stream 6 Channel 0
// dma pin B08 0
// # pin B08: DMA1 Stream 7 Channel 2
