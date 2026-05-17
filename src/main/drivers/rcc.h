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

#pragma once

#include "rcc_types.h"

enum rcc_reg {
    RCC_EMPTY = 0,   // make sure that default value (0) does not enable anything
    RCC_AHB,
    RCC_APB2,
    RCC_APB1,
    RCC_AHB1,
    RCC_AHB4
};

#define RCC_ENCODE(reg, mask) (((reg) << 5) | LOG2_32BIT(mask))
#define RCC_AHB(periph) RCC_ENCODE(RCC_AHB, RCC_AHBENR_ ## periph ## EN)
#define RCC_APB2(periph) RCC_ENCODE(RCC_APB2, RCC_APB2ENR_ ## periph ## EN)
#define RCC_APB1(periph) RCC_ENCODE(RCC_APB1, RCC_APB1ENR_ ## periph ## EN)
#define RCC_AHB1(periph) RCC_ENCODE(RCC_AHB1, RCC_AHB1ENR_ ## periph ## EN)
#define RCC_AHB4(periph) RCC_ENCODE(RCC_AHB4, RCC_AHB4ENR_ ## periph ## EN)
// H7 aliases: APB1 is split into APB1L/APB1H; RCC_APB1L maps directly to the _LENR registers
#define RCC_APB1L(periph) RCC_ENCODE(RCC_APB1, RCC_APB1LENR_ ## periph ## EN)

void RCC_ClockCmd(rccPeriphTag_t periphTag, FunctionalState NewState);
void RCC_ResetCmd(rccPeriphTag_t periphTag, FunctionalState NewState);
