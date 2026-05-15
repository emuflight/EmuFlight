/*
 * This file is part of EmuFlight. It is derived from Betaflight.
 *
 * This is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 * You should have received a copy of the GNU General Public
 * License along with this software.
 * If not, see <http://www.gnu.org/licenses/>.
 */

// Synthetic CI target — STM32F303 family compile gate.
// GPIO pins copied from WORMFC (valid for F303; not a real product).
// .hex output is NOT flashable. See make/targets.mk UNSUPPORTED_TARGETS.

#pragma once

#define TARGET_MANUFACTURER_IDENTIFIER "CIST"
#define USBD_PRODUCT_STRING "CI_STM32F3"

#define FC_TARGET_MCU     STM32F303     // not used in EmuF
#define TARGET_BOARD_IDENTIFIER "CIF3"  // generic ID

#define USE_GYRO
#define USE_GYRO_SPI_MPU6000
#define USE_ACC
#define USE_ACC_SPI_MPU6000
#define USE_BARO
#define USE_BARO_BMP280

#define USE_VCP
#define USE_OSD

#define USE_LED
#define LED0_PIN             PB3
#define USE_BEEPER
#define BEEPER_PIN           PC15

#define USE_SPI
#define USE_SPI_DEVICE_1
#define SPI1_SCK_PIN         PA5
#define SPI1_MISO_PIN        PA6
#define SPI1_MOSI_PIN        PA7

#define USE_EXTI
#define MPU_INT_EXTI         PC13
#define USE_MPU_DATA_READY_SIGNAL

#define GYRO_1_ALIGN         CW0_DEG
#define GYRO_1_CS_PIN        PA4
#define GYRO_1_EXTI_PIN      PC13
#define GYRO_1_SPI_BUS  SPIDEV_1

#define USE_SPI_GYRO
#define MPU6000_CS_PIN       PA4
#define MPU6000_SPI_BUS SPIDEV_1

#define USE_UART1
#define UART1_TX_PIN         PA9
#define UART1_RX_PIN         PA10

#define USE_UART2
#define UART2_TX_PIN         PA2
#define UART2_RX_PIN         PA3

#define SERIAL_PORT_COUNT    3 // VCP + UART1 + UART2

#define USE_I2C
#define USE_I2C_DEVICE_1
#define I2C_DEVICE           (I2CDEV_1)
#define I2C1_SCL             PB6
#define I2C1_SDA             PB7

#define USE_ADC
#define ADC_INSTANCE         ADC1
#define VBAT_ADC_PIN         PA0
#define CURRENT_METER_ADC_PIN PA1

#define DEFAULT_RX_FEATURE   FEATURE_RX_SERIAL
#define SERIALRX_PROVIDER    SERIALRX_SBUS

#define TARGET_IO_PORTA 0xffff
#define TARGET_IO_PORTB 0xffff
#define TARGET_IO_PORTC 0xffff
#define TARGET_IO_PORTF 0x001f

#define USABLE_TIMER_CHANNEL_COUNT 6
#define USED_TIMERS (TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(4))
