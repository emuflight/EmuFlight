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
// Commit: 4cb7ad1 + manual edits for SDCard, UART5, etc.

#pragma once

#define TARGET_MANUFACTURER_IDENTIFIER "SPBE"
#define USBD_PRODUCT_STRING "SPEEDYBEEF7V3"

#define FC_TARGET_MCU     STM32F7X2     // not used in EmuF
#define TARGET_BOARD_IDENTIFIER "S7X2"  // generic ID

#define USE_GYRO
#define USE_ACC
#define USE_ACCGYRO_BMI270
#define USE_BARO_BMP280
#define USE_MAX7456
#define USE_BARO
#define USE_SPI_GYRO

#define USE_VCP
#define USE_OSD

#define USE_LED
#define LED0_PIN             PA14
#define LED1_PIN             PA13
#define LED_STRIP_PIN        PC8
#define USE_BEEPER
#define BEEPER_PIN           PC13
#define BEEPER_INVERTED
#define CAMERA_CONTROL_PIN   PA0

#define USE_SPI
#define USE_SPI_DEVICE_1
#define SPI1_SCK_PIN         PA5
#define SPI1_MISO_PIN        PA6
#define SPI1_MOSI_PIN        PA7
#define USE_SPI_DEVICE_2
#define SPI2_SCK_PIN         PB13
#define SPI2_MISO_PIN        PB14
#define SPI2_MOSI_PIN        PB15
#define USE_SPI_DEVICE_3
#define SPI3_SCK_PIN         PC10
#define SPI3_MISO_PIN        PC11
#define SPI3_MOSI_PIN        PC12

#define GYRO_1_ALIGN         CW270_DEG_FLIP
#define ACC_1_ALIGN          CW270_DEG_FLIP
#define GYRO_1_CS_PIN        PB2
#define GYRO_1_EXTI_PIN      PC4
#define GYRO_1_SPI_INSTANCE  SPI1
#define MPU_INT_EXTI         PC4

#define USE_DUAL_GYRO
#define GYRO_CONFIG_USE_GYRO_DEFAULT GYRO_CONFIG_USE_GYRO_2

#define GYRO_2_ALIGN         CW0_DEG_FLIP
#define ACC_2_ALIGN          CW0_DEG_FLIP
#define GYRO_2_CS_PIN        PC15
#define GYRO_2_EXTI_PIN      PC3
#define GYRO_2_SPI_INSTANCE  SPI1

#define USE_EXTI // notice - REQUIRED when USE_GYRO_EXTI
#define USE_GYRO_EXTI

#define USE_SPI_GYRO
#define ACC_BMI270_ALIGN     CW270_DEG_FLIP
#define GYRO_BMI270_ALIGN    CW270_DEG_FLIP
#define BMI270_CS_PIN        PB2
#define BMI270_SPI_INSTANCE  SPI1

#define USE_UART1
#define UART1_TX_PIN         PA9
#define UART1_RX_PIN         PA10
#define USE_UART2
#define UART2_TX_PIN         PA2
#define UART2_RX_PIN         PA3
#define USE_UART3
#define UART3_TX_PIN         PB10
#define UART3_RX_PIN         PB11
#define USE_UART4
#define UART4_RX_PIN         PA1
#define USE_UART6
#define UART6_TX_PIN         PC6
#define UART6_RX_PIN         PC7
// note: UART5 NONE, but still need to define USE_UART5 and add +1 to serial-count.
#define USE_UART5
#define SERIAL_PORT_COUNT 7

#define USE_I2C
#define USE_I2C_DEVICE_1
#define I2C_DEVICE        (I2CDEV_1)
#define MAG_I2C_INSTANCE  (I2CDEV_1)
#define BARO_I2C_INSTANCE (I2CDEV_1)
#define I2C1_SCL          PB8
#define I2C1_SDA          PB9

//#define USE_FLASH //breaks CLI resources (need support in .mk? or not valid at all?)
#define FLASH_CS_PIN          PD2
#define FLASH_SPI_INSTANCE    SPI3

#define USE_SDCARD
#define USE_SDCARD_SPI
#define USE_SDCARD_SDIO
#define SDCARD_SPI_CS_PIN     PD2
#define SDCARD_SPI_INSTANCE   SPI3
#define SDCARD_DMA_CHANNEL    0
#define SDCARD_DMA_CHANNEL_TX DMA1_Stream5 //# SPI_MOSI 3: DMA1 Stream 5 Channel 0
#define ENABLE_BLACKBOX_LOGGING_ON_SDCARD_BY_DEFAULT
#define DEFAULT_BLACKBOX_DEVICE     BLACKBOX_DEVICE_SDCARD
#define SDCARD_SPI_FULL_SPEED_CLOCK_DIVIDER     4
#define SDCARD_SPI_INITIALIZATION_CLOCK_DIVIDER 256

#define MAX7456_SPI_CS_PIN   PB12
#define MAX7456_SPI_INSTANCE SPI2

#define USE_ADC
#define ADC_INSTANCE          ADC1
#define VBAT_ADC_PIN          PC2
#define CURRENT_METER_ADC_PIN PC1
#define RSSI_ADC_PIN          PC0
#define ADC1_DMA_STREAM       DMA2_Stream0 //# ADC 1: DMA2 Stream 0 Channel 0
#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define DEFAULT_CURRENT_METER_SOURCE CURRENT_METER_ADC
#define DEFAULT_CURRENT_METER_SCALE 490

#define ENABLE_DSHOT_DMAR true

#define PINIO1_PIN           PC9
#define PINIO1_CONFIG        129
#define PINIO1_BOX           0

#define TARGET_IO_PORTA 0xffff
#define TARGET_IO_PORTB 0xffff
#define TARGET_IO_PORTC 0xffff
#define TARGET_IO_PORTD 0xffff

#define DEFAULT_FEATURES       (FEATURE_OSD | FEATURE_TELEMETRY | FEATURE_AIRMODE | FEATURE_RX_SERIAL)
#define DEFAULT_RX_FEATURE     FEATURE_RX_SERIAL

#define USABLE_TIMER_CHANNEL_COUNT 11
#define USED_TIMERS ( TIM_N(2) | TIM_N(3) | TIM_N(4) | TIM_N(5) | TIM_N(8) | TIM_N(9) )

// notice - this file was programmatically generated and may be incomplete.

