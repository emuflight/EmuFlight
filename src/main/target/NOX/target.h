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

// This resource file generated using https://github.com/nerdCopter/target-convert
// Commit: d84474d + 1 file changed, 24 deletions(-)

#pragma once

#define BOARD_NAME        NOX
#define MANUFACTURER_ID   AIRB
#define TARGET_BOARD_IDENTIFIER "S411"  // generic ID
#define FC_TARGET_MCU     STM32F411     // not used in EmuF

#define USE_GYRO
#define USE_GYRO_SPI_MPU6000
#define USE_GYRO_SPI_MPU6500
#define USE_ACC
#define USE_ACC_SPI_MPU6000
#define USE_ACC_SPI_MPU6500
#define USE_BARO_SPI_BMP280
#define USE_FLASH
#define USE_FLASH_M25P16
#define USE_MAX7456
#define USE_BARO
//TODO #define USE_UNSYNCED_PWM OFF

#define USE_VCP
#define USE_FLASHFS
#define USE_FLASH_M25P16    // 16MB Micron M25P16 driver; drives all unless QSPI
#define USE_OSD

#define USE_LED
#define LED0_PIN             PA4
#define LED_STRIP_PIN        PA0
#define USE_BEEPER
#define BEEPER_PIN           PC13
#define BEEPER_INVERTED

#define USE_SPI
#define USE_SPI_DEVICE_1
#define SPI1_SCK_PIN         PB3
#define SPI1_MISO_PIN        PB4
#define SPI1_MOSI_PIN        PB5
#define USE_SPI_DEVICE_2
#define SPI2_SCK_PIN         PB13
#define SPI2_MISO_PIN        PB14
#define SPI2_MOSI_PIN        PB15

#define USE_SPI_GYRO
#define USE_EXTI
#define USE_GYRO_EXTI

#define USE_MPU_DATA_READY_SIGNAL

#define MPU_INT_EXTI         NONE

#define ACC_MPU6000_ALIGN        CW0_DEG
#define GYRO_MPU6000_ALIGN       CW0_DEG
#define MPU6000_CS_PIN           PB12
#define MPU6000_SPI_INSTANCE     SPI2

#define ACC_MPU6500_ALIGN        CW0_DEG
#define GYRO_MPU6500_ALIGN       CW0_DEG
#define MPU6500_CS_PIN           PB12
#define MPU6500_SPI_INSTANCE     SPI2

#define USE_UART1
#define UART1_TX_PIN         PB6
#define UART1_RX_PIN         PB7
#define USE_UART2
#define UART2_TX_PIN         PA2
#define UART2_RX_PIN         PA3
#define INVERTER_PIN_UART2   PC14
#define SERIAL_PORT_COUNT 3

#define BARO_CS_PIN          PA9
#define BARO_SPI_INSTANCE SPI2
#define BMP280_CS_PIN       PA9
#define BMP280_SPI_INSTANCE SPI2

#define FLASH_CS_PIN         PA15
#define FLASH_SPI_INSTANCE SPI1
#define ENABLE_BLACKBOX_LOGGING_ON_SPIFLASH_BY_DEFAULT

#define MAX7456_SPI_CS_PIN   PA10
#define MAX7456_SPI_INSTANCE SPI2

#define USE_ADC
#define VBAT_ADC_PIN PA5
#define ADC1_DMA_OPT        1
#define ADC1_DMA_STREAM DMA2_Stream4 //# ADC 1: DMA2 Stream 4 Channel 0
#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ESC
#define DEFAULT_CURRENT_METER_SOURCE CURRENT_METER_ESC

#define USE_ESCSERIAL
#define ESCSERIAL_TIMER_TX_PIN PB10


#define TARGET_IO_PORTA 0xffff
#define TARGET_IO_PORTB 0xffff
#define TARGET_IO_PORTC 0xffff
// notice - masks were programmatically generated - please verify last port group for 0xffff or (BIT(2))

#define DEFAULT_FEATURES       (FEATURE_OSD | FEATURE_TELEMETRY | FEATURE_AIRMODE | FEATURE_RX_SERIAL)
#define DEFAULT_RX_FEATURE     FEATURE_RX_SERIAL

#define USABLE_TIMER_CHANNEL_COUNT 8
#define USED_TIMERS ( TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(4) | TIM_N(5) | TIM_N(9) )

// notice - this file was programmatically generated and may be incomplete.
