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
// Commit: 3f33ae6 + 1 file changed, 24 deletions(-)

#include <stdint.h>
#include "platform.h"
#include "drivers/io.h"
#include "drivers/dma.h"
#include "drivers/timer.h"
#include "drivers/timer_def.h"

const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    DEF_TIM(TIM3, CH1, PB4, TIM_USE_MOTOR, 0, 0), // motor 1
    DEF_TIM(TIM3, CH2, PB5, TIM_USE_MOTOR, 0, 0), // motor 2
    DEF_TIM(TIM4, CH1, PB6, TIM_USE_MOTOR, 0, 0), // motor 3
    DEF_TIM(TIM4, CH2, PB7, TIM_USE_MOTOR, 0, 0), // motor 4
    // DEF_TIM(TIM2, CH2, PB3,  TIM_USE_MOTOR, 0, 0), // motor 5 disabled; PB3 used for SOFTSERIAL1 TX/RX — mutually exclusive (BF: MOTOR 5 B03 / SERIAL_TX|RX 11 B03)
    DEF_TIM(TIM2, CH2, PB3,  TIM_USE_ANY,   0, 0), // SOFTSERIAL1 TX/RX (half-duplex; shared with motor 5 — motor 5 disabled)
    // DEF_TIM(TIM2, CH3, PB10, TIM_USE_MOTOR, 0, 0), // motor 6 disabled; motor 5 disabled leaves 5 motor slots (odd count — invalid); motor 6 also disabled for even 4-motor count
    DEF_TIM(TIM9, CH2, PA3,  TIM_USE_PPM,   0, 0), // PPM; shared with UART2_RX — mutually exclusive (BF: PPM 1 A03 / SERIAL_RX 2 A03)
    DEF_TIM(TIM1, CH1, PA8,  TIM_USE_LED,   0, 0), // led
};

// notice - this file was programmatically generated and may be incomplete.
