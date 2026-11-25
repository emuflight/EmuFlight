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

/*
 * Author: Giles Burgess (giles@multiflite.co.uk)
 *
 * This source code is provided as is and can be used/modified so long
 * as this header is maintained with the file at all times.
 */

#pragma once

#include <stdint.h>

#define VTX_RTC6705_BAND_COUNT      5
#define VTX_RTC6705_CHANNEL_COUNT   8
#if defined(RTC6705_EX_POWER_1_PIN) && defined(RTC6705_EX_POWER_2_PIN)
#define RTC6705_DYNAMIC_POWER_CTRL
#endif

#if defined(RTC6705_DYNAMIC_POWER_CTRL) && defined(RTC6705_POWER_PIN)
#error "RTC6705_DYNAMIC_POWER_CTRL and RTC6705_POWER_PIN cannot both be defined"
#endif

#ifdef RTC6705_DYNAMIC_POWER_CTRL
#define VTX_RTC6705_POWER_COUNT     4   // index 0 unused; 1 to 3 = external codes 00, 01, 10 (code 11 is undefined and unsafe)
#else
#define VTX_RTC6705_POWER_COUNT     3
#endif
#define VTX_RTC6705_DEFAULT_POWER   1

#if defined(RTC6705_POWER_PIN)
#define VTX_RTC6705_MIN_POWER   0
#else
#define VTX_RTC6705_MIN_POWER   1
#endif

#define VTX_RTC6705_FREQ_MIN    5600
#define VTX_RTC6705_FREQ_MAX    5950

#define VTX_RTC6705_BOOT_DELAY  350 // milliseconds

void rtc6705IOInit(void);
void rtc6705SetFrequency(uint16_t freq);
void rtc6705SetRFPower(uint8_t rf_power);
void rtc6705Disable(void);
void rtc6705Enable(void);
