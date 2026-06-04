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
 * You should have received a copy of the GNU General Public
 * License along with this software.
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdint.h>
#include "platform.h"
#include "drivers/io.h"
#include "drivers/dma.h"
#include "drivers/timer.h"
#include "drivers/timer_def.h"

/*
 * Motor pin → timer mapping (TIM5 CH1-4, PA0-PA3):
 *   PA0 = TIM5_CH1, PA1 = TIM5_CH2, PA2 = TIM5_CH3, PA3 = TIM5_CH4
 * DShot dmaopt 0-3 → DMA1_Stream0-3.
 * upopt NONE: TIM5_UP burst DShot not used (DMAMUX request TBD for this target).
 *
 * LED strip TIM1_CH1 on PA8 (dmaopt uses DMA2).
 * Camera control: PE5 = TIM15_CH1 per BF SPRACINGH7NANO config (not PB9 — PB9 = I2C1_SDA).
 * Camera control entry not added here; needs schematic confirmation and target.h completion.
 */

const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    DEF_TIM(TIM5,  CH1, PA0, TIM_USE_MOTOR, 0, 0, NONE), // M1
    DEF_TIM(TIM5,  CH2, PA1, TIM_USE_MOTOR, 0, 1, NONE), // M2
    DEF_TIM(TIM5,  CH3, PA2, TIM_USE_MOTOR, 0, 2, NONE), // M3
    DEF_TIM(TIM5,  CH4, PA3, TIM_USE_MOTOR, 0, 3, NONE), // M4
    DEF_TIM(TIM1,  CH1, PA8, TIM_USE_LED,   0, 9, NONE), // LED strip — DMA2_S1
};
