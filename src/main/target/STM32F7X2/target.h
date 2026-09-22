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

// Treat the target as generic, and expect manufacturer id / board name
// to be supplied when the board is configured for the first time
#define GENERIC_TARGET

#define TARGET_BOARD_IDENTIFIER "S7X2"

#define USBD_PRODUCT_STRING     "S7X2"

#define USE_BEEPER

// No IMU/gyro selected: this file carries no board-specific sensor wiring, and
// EF has no CLI/resource path to add one at runtime (no OWNER_MPU_CS/OWNER_MPU_EXTI
// resourceTable entries). To use this target with a real board's gyro, add its
// USE_*_SPI_*/USE_*_I2C_* chip macro plus the matching *_CS_PIN/*_SPI_BUS/*_EXTI_PIN
// (or GYRO_1_* equivalents) here, matching that board's actual wiring.
#define USE_EXTI

#define USE_MAG
#define USE_MAG_HMC5883
#define USE_MAG_QMC5883
#define MAG_HMC5883_ALIGN CW270_DEG_FLIP

#define USE_BARO
#define USE_BARO_MS5611
#define USE_BARO_BMP280
#define USE_BARO_LPS

#define USE_SDCARD

#define SDCARD_SPI_INITIALIZATION_CLOCK_DIVIDER 256 // 328kHz
#define SDCARD_SPI_FULL_SPEED_CLOCK_DIVIDER     4 // 21MHz

#define USE_I2C
#define USE_I2C_DEVICE_1
#define USE_I2C_DEVICE_2
#define USE_I2C_DEVICE_3
#define I2C_FULL_RECONFIGURABILITY

#define USE_VCP

#define USE_UART1
#define USE_UART2
#define USE_UART3
#define USE_UART4
#define USE_UART5
#define USE_UART6
#define USE_SOFTSERIAL1
#define USE_SOFTSERIAL2

#define SERIAL_PORT_COUNT       9

#define USE_SPI
#define USE_SPI_DEVICE_1
#define USE_SPI_DEVICE_2
#define USE_SPI_DEVICE_3
#define SPI_FULL_RECONFIGURABILITY

#define USE_ADC


#define USABLE_TIMER_CHANNEL_COUNT 70
#define USED_TIMERS ( TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(4) | TIM_N(5) | TIM_N(8) | TIM_N(9) | TIM_N(10) | TIM_N(11) | TIM_N(12) | TIM_N(13) | TIM_N(14) )

// USE_TIMER_MGMT intentionally not defined: this generic target's pin count exceeds
// MAX_TIMER_PINMAP_COUNT, so it cannot carry a TIMER_PIN_MAPPING without truncating the
// set of pins available for runtime CLI configuration.
