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
// Commit: 3f33ae6 + 1 file changed, 24 deletions(-)

#pragma once

#define BOARD_NAME        DARWINF411
#define MANUFACTURER_ID   DAFP
#define TARGET_BOARD_IDENTIFIER "S411"  // generic ID
#define FC_TARGET_MCU     STM32F411     // not used in EmuF

#define USE_GYRO
#define USE_GYRO_SPI_MPU6500
#define USE_ACC
#define USE_ACC_SPI_MPU6500
#define USE_GYRO_SPI_ICM20689
#define USE_ACC_SPI_ICM20689
#define USE_ACCGYRO_BMI270
#define USE_BARO
#define USE_BARO_BMP280
#define USE_MAX7456
#define USE_SDCARD
#define USE_SDCARD_SPI

#define USE_VCP
#define USE_OSD

#define USE_LED
#define LED0_PIN             PC13
#define LED1_PIN             PC14
#define LED_STRIP_PIN        PA8
#define USE_BEEPER
#define BEEPER_PIN           PB2
#define BEEPER_INVERTED
#define USE_USB_DETECT

#define USE_SPI
#define USE_SPI_DEVICE_1
#define SPI1_SCK_PIN         PA5
#define SPI1_MISO_PIN        PA6
#define SPI1_MOSI_PIN        PA7
#define USE_SPI_DEVICE_2
#define SPI2_SCK_PIN         PB13
#define SPI2_MISO_PIN        PB14
#define SPI2_MOSI_PIN        PB15

#define USE_SPI_GYRO
#define USE_EXTI
#define USE_GYRO_EXTI

#define USE_MPU_DATA_READY_SIGNAL

#define MPU_INT_EXTI         PA1

#define ACC_MPU6500_ALIGN        CW90_DEG_FLIP
#define GYRO_MPU6500_ALIGN       CW90_DEG_FLIP
#define MPU6500_CS_PIN           PA4
#define MPU6500_SPI_INSTANCE     SPI1

#define ACC_ICM20689_ALIGN       CW90_DEG_FLIP
#define GYRO_ICM20689_ALIGN      CW90_DEG_FLIP
#define ICM20689_CS_PIN          PA4
#define ICM20689_SPI_INSTANCE    SPI1

#define ACC_BMI270_ALIGN         CW90_DEG_FLIP
#define GYRO_BMI270_ALIGN        CW90_DEG_FLIP
#define BMI270_CS_PIN            PA4
#define BMI270_SPI_INSTANCE      SPI1

#define USE_UART1
#define UART1_TX_PIN         PA9
#define UART1_RX_PIN         PA10
#define USE_UART2
#define UART2_TX_PIN         PA2
#define UART2_RX_PIN         PA3
#define USE_SOFTSERIAL1
#define SOFTSERIAL1_TX_PIN   PB3
#define SOFTSERIAL1_RX_PIN   PB3
#define SERIAL_PORT_COUNT 4

#define USE_I2C
#define USE_I2C_DEVICE_1
#define I2C_DEVICE_1      (I2CDEV_1)
#define MAG_I2C_INSTANCE (I2CDEV_1)
#define BARO_I2C_INSTANCE (I2CDEV_1)
#define I2C1_SCL PB8
#define I2C1_SDA PB9

#define USE_SDCARD_SDIO
#define SDCARD_SPI_CS_PIN    PA0
#define SDCARD_SPI_INSTANCE SPI2
#define SDCARD_DMA_CHANNEL          6            //# pin A00: DMA1 Stream 2 Channel 6
#define SDCARD_DMA_CHANNEL_TX       DMA1_Stream2 //# pin A00: DMA1 Stream 2 Channel 6
//notice - other sdcard defines maybe needed (rare?): SDCARD_DMA_STREAM_TX_FULL, SDCARD_DMA_STREAM_TX, SDCARD_DMA_CLK, SDCARD_DMA_CHANNEL_TX_COMPLETE_FLAG
#define ENABLE_BLACKBOX_LOGGING_ON_SDCARD_BY_DEFAULT
#define SDCARD_SPI_FULL_SPEED_CLOCK_DIVIDER     4    //notice - needs validation. these are hardware dependent. known options: 2, 4, 8.
#define SDCARD_SPI_INITIALIZATION_CLOCK_DIVIDER 256  //notice - needs validation. these are hardware dependent. known options: 128, 256

#define MAX7456_SPI_CS_PIN   PB12
#define MAX7456_SPI_INSTANCE SPI2

#define USE_ADC
#define VBAT_ADC_PIN PB0
#define CURRENT_METER_ADC_PIN PB1
#define ADC1_DMA_OPT        1
#define ADC1_DMA_STREAM DMA2_Stream4 //# ADC 1: DMA2 Stream 4 Channel 0
#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define DEFAULT_CURRENT_METER_SOURCE CURRENT_METER_ADC
#define DEFAULT_CURRENT_METER_SCALE 125

#define TARGET_IO_PORTA 0xffff
#define TARGET_IO_PORTB 0xffff
#define TARGET_IO_PORTC 0xffff
// notice - masks were programmatically generated - please verify last port group for 0xffff or (BIT(2))

#define DEFAULT_FEATURES       (FEATURE_OSD | FEATURE_TELEMETRY | FEATURE_AIRMODE | FEATURE_RX_SERIAL)
#define DEFAULT_RX_FEATURE     FEATURE_RX_SERIAL

#define USABLE_TIMER_CHANNEL_COUNT 10
#define USED_TIMERS ( TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(4) | TIM_N(5) | TIM_N(9) )

// notice - this file was programmatically generated and may be incomplete.
