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
    DEF_TIM(TIM2, CH2, PB3,  TIM_USE_LED,                  0, 0 ), // LED_STRIP,
    //DEF_TIM(TIM4, CH3, PB8,  TIM_USE_ANY,                  0, 0),  // baro/mag // no dps310 in EmuFlight,
    //DEF_TIM(TIM11, CH1, PB9,  TIM_USE_ANY,                  0, 0),  // baro/mag // no dps310 in EmuFlight,
    DEF_TIM(TIM1,  CH2, PA9,  TIM_USE_MOTOR,               0, 0 ), // M1
    DEF_TIM(TIM1,  CH1, PA8,  TIM_USE_MOTOR,               0, 0 ), // M2
    DEF_TIM(TIM8,  CH4, PC9,  TIM_USE_MOTOR,               0, 0 ), // M3
    DEF_TIM(TIM8,  CH3, PC8,  TIM_USE_MOTOR,               0, 0 ), // M4
    DEF_TIM(TIM3,  CH3, PB0,  TIM_USE_MOTOR,               0, 0 ), // M5
    DEF_TIM(TIM3,  CH4, PB1,  TIM_USE_MOTOR,               0, 0 ), // M6
};
// notice - this file was programmatically generated and may be incomplete.

// #define MOTOR1_PIN           PA9
// #define MOTOR2_PIN           PA8
// #define MOTOR3_PIN           PC9
// #define MOTOR4_PIN           PC8
// #define MOTOR5_PIN           PB0
// #define MOTOR6_PIN           PB1

// #define TIMER_PIN_MAPPING \
//     TIMER_PIN_MAP( 0, PB9 , 2, -1) \
//     TIMER_PIN_MAP( 1, PA9 , 1,  0) \ //m1
//     TIMER_PIN_MAP( 2, PA8 , 1,  0) \ //m2
//     TIMER_PIN_MAP( 3, PC9 , 2,  0) \ //m3
//     TIMER_PIN_MAP( 4, PC8 , 2,  0) \ //m4
//     TIMER_PIN_MAP( 5, PB0 , 2,  0) \ //m5
//     TIMER_PIN_MAP( 6, PB1 , 2,  0) \ //m6
//     TIMER_PIN_MAP( 7, PB8 , 1,  0) \ //I2C1_SCL_PIN //baro/mag?
//     TIMER_PIN_MAP( 8, PB3 , 1,  0) //LED_STRIP_PIN

// # timer
// timer B09 AF3
// # pin B09: TIM11 CH1 (AF3) //baro/mag
// timer A09 AF1
// # pin A09: TIM1 CH2 (AF1)  //m1
// timer A08 AF1
// # pin A08: TIM1 CH1 (AF1)  //m2
// timer C09 AF3
// # pin C09: TIM8 CH4 (AF3)  //m3
// timer C08 AF3
// # pin C08: TIM8 CH3 (AF3)  //m4
// timer B00 AF2
// # pin B00: TIM3 CH3 (AF2)  //m5
// timer B01 AF2
// # pin B01: TIM3 CH4 (AF2)  //m6
// timer B08 AF2
// # pin B08: TIM4 CH3 (AF2)   //baro/mag
// timer B03 AF1
// # pin B03: TIM2 CH2 (AF1) //led
// 
// # dma
// dma ADC 3 0
// # ADC 3: DMA2 Stream 0 Channel 2
// dma pin A09 0
// # pin A09: DMA2 Stream 6 Channel 0
// dma pin A08 0
// # pin A08: DMA2 Stream 6 Channel 0
// dma pin C09 0
// # pin C09: DMA2 Stream 7 Channel 7
// dma pin C08 0
// # pin C08: DMA2 Stream 2 Channel 0
// dma pin B00 0
// # pin B00: DMA1 Stream 7 Channel 5
// dma pin B01 0
// # pin B01: DMA1 Stream 2 Channel 5
// dma pin B08 0
// # pin B08: DMA1 Stream 7 Channel 2
// dma pin B03 0
// # pin B03: DMA1 Stream 6 Channel 3

