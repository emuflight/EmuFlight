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

#define BOARD_NAME        AIRBOTF4
#define MANUFACTURER_ID   AIRB
#define TARGET_BOARD_IDENTIFIER "S405"  // generic ID
#define FC_TARGET_MCU     STM32F405     // not used in EmuF

#define USE_ACC
#define USE_ACC_SPI_MPU6500
#define USE_GYRO
#define USE_GYRO_SPI_MPU6500
#define USE_ACC_SPI_MPU6000
#define USE_GYRO_SPI_MPU6000
#define USE_BARO_SPI_BMP280
#define USE_MAX7456
#define USE_FLASH
#define USE_FLASH_M25P16
#define USE_BARO_BMP280
#define USE_BARO_BMP085
#define USE_BARO_MS5611
#define USE_BARO

#define USE_VCP
#define USE_FLASHFS
#define USE_FLASH_M25P16    // 16MB Micron M25P16 driver; drives all unless QSPI
#define USE_OSD

#define USE_LED
#define LED0_PIN             PB5
#define LED_STRIP_PIN        PB6
#define USE_BEEPER
#define BEEPER_PIN           PB4
#define BEEPER_INVERTED
#define USE_USB_DETECT

#define USE_SPI
#define USE_SPI_DEVICE_1
#define SPI1_SCK_PIN         PA5
#define SPI1_MISO_PIN        PA6
#define SPI1_MOSI_PIN        PA7
#define USE_SPI_DEVICE_3
#define SPI3_SCK_PIN         PC10
#define SPI3_MISO_PIN        PC11
#define SPI3_MOSI_PIN        PC12

#define USE_SPI_GYRO
#define USE_EXTI
#define USE_GYRO_EXTI

#define USE_MPU_DATA_READY_SIGNAL

#define MPU_INT_EXTI         PC4

#define ACC_MPU6000_ALIGN        CW270_DEG
#define GYRO_MPU6000_ALIGN       CW270_DEG
#define MPU6000_CS_PIN           PA4
#define MPU6000_SPI_INSTANCE     SPI1

#define ACC_MPU6500_ALIGN        CW270_DEG
#define GYRO_MPU6500_ALIGN       CW270_DEG
#define MPU6500_CS_PIN           PA4
#define MPU6500_SPI_INSTANCE     SPI1

#define USE_UART1
#define UART1_TX_PIN         PA9
#define UART1_RX_PIN         PA10
#define USE_UART3
#define UART3_TX_PIN         PB10
#define UART3_RX_PIN         PB11
#define USE_UART4
#define UART4_TX_PIN         PA0
#define UART4_RX_PIN         PA1
#define USE_UART6
#define UART6_TX_PIN         PC6
#define UART6_RX_PIN         PC7
#define INVERTER_PIN_UART1   PC0
#define SERIAL_PORT_COUNT 5

#define BARO_CS_PIN          PC13
#define BARO_SPI_INSTANCE SPI1
#define BMP280_CS_PIN       PC13
#define BMP280_SPI_INSTANCE SPI1
#define USE_I2C
#define MAG_I2C_INSTANCE (I2CDEV_2)
#define DASHBOARD_I2C_INSTANCE (I2CDEV_2)

#define FLASH_CS_PIN         PB3
#define FLASH_SPI_INSTANCE SPI3


#define USE_ADC
#define VBAT_ADC_PIN PC2
#define CURRENT_METER_ADC_PIN PC1
#define ADC1_DMA_OPT        1
#define ADC1_DMA_STREAM DMA2_Stream4 //# ADC 1: DMA2 Stream 4 Channel 0

#define USE_ESCSERIAL
#define ESCSERIAL_TIMER_TX_PIN PB14

#define PINIO1_PIN           PC8

#define TARGET_IO_PORTA 0xffff
#define TARGET_IO_PORTB 0xffff
#define TARGET_IO_PORTC 0xffff
// notice - masks were programmatically generated - please verify last port group for 0xffff or (BIT(2))

#define DEFAULT_FEATURES       (FEATURE_OSD | FEATURE_TELEMETRY | FEATURE_AIRMODE | FEATURE_RX_SERIAL)
#define DEFAULT_RX_FEATURE     FEATURE_RX_SERIAL

#define USABLE_TIMER_CHANNEL_COUNT 13
#define USED_TIMERS ( TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(4) | TIM_N(5) | TIM_N(8) | TIM_N(12) )

// notice - this file was programmatically generated and may be incomplete.
