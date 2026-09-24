/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
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

#pragma once

#ifndef TARGET_BOARD_IDENTIFIER
#define TARGET_BOARD_IDENTIFIER         "S446"
#endif

#ifndef USBD_PRODUCT_STRING
#define USBD_PRODUCT_STRING             "STM32F446"
#endif

#define USE_I2C_DEVICE_1
#define USE_I2C_DEVICE_2
#define USE_I2C_DEVICE_3

#define USE_VCP

#define USE_SOFTSERIAL

#define UNIFIED_SERIAL_PORT_COUNT       3

#define USE_UART1
#define USE_UART2
// #define USE_UART6

#define SERIAL_PORT_COUNT       (UNIFIED_SERIAL_PORT_COUNT + 2)

#define USE_INVERTER

#define USE_SPI_DEVICE_1
#define USE_SPI_DEVICE_2
#define USE_SPI_DEVICE_3

#define USE_I2C
#define I2C_FULL_RECONFIGURABILITY

#define USE_DSHOT_BITBAND

#ifdef USE_SDCARD
#define USE_SDCARD_SPI
#define USE_SDCARD_SDIO
#endif

#define USE_SPI
#define SPI_FULL_RECONFIGURABILITY
#define USE_SPI_DMA_ENABLE_EARLY

#define USE_USB_DETECT

#define USE_ADC

#define USE_EXTI

#define FLASH_PAGE_SIZE                 ((uint32_t)0x4000) // 16K sectors

// compile-sanity target only -- no real motor/timer pins.
// USE_TIMER_MGMT pulls USED_TIMERS/FULL_TIMER_CHANNEL_COUNT fleet defaults from
// common_defaults_post.h; USABLE_TIMER_CHANNEL_COUNT/timerHardware[] (target.c) still cover
// timerioTagGetByUsage()'s legacy scan, which isn't USE_TIMER_MGMT-gated.
#define USE_TIMER_MGMT
#define USABLE_TIMER_CHANNEL_COUNT 0
