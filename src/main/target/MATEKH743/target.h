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
// Commit: 215ae87 + 1 file changed, 31 insertions(+), 8 deletions(-)

#pragma once

#define BOARD_NAME        MATEKH743
#define MANUFACTURER_ID   MTKS
#define TARGET_BOARD_IDENTIFIER "SH74"  // generic ID
#define FC_TARGET_MCU     STM32H743     // not used in EmuF

#define USE_ACC
#define USE_ACC_SPI_MPU6000
#define USE_ACC_SPI_MPU6500
#define USE_ACC_SPI_ICM42605
#define USE_ACC_SPI_ICM42688P
#define USE_GYRO
#define USE_GYRO_SPI_MPU6000
#define USE_GYRO_SPI_MPU6500
#define USE_GYRO_SPI_ICM42605
#define USE_GYRO_SPI_ICM42688P
#define USE_BARO
#define USE_BARO_BMP280
#define USE_BARO_DPS310
#define USE_MAX7456
#define USE_SDCARD

#define USE_VCP
#define USE_OSD

#define USE_LED
#define LED0_PIN             PE3
#define LED1_PIN             PE4
#define LED_STRIP_PIN        PA8
#define USE_BEEPER
#define BEEPER_PIN           PA15
#define BEEPER_INVERTED

#define USE_SPI
#define USE_SPI_DEVICE_1
#define SPI1_SCK_PIN         PA5
#define SPI1_MISO_PIN        PA6
#define SPI1_MOSI_PIN        PD7
#define USE_SPI_DEVICE_2
#define SPI2_SCK_PIN         PB13
#define SPI2_MISO_PIN        PB14
#define SPI2_MOSI_PIN        PB15
#define USE_SPI_DEVICE_3
#define SPI3_SCK_PIN         PB3
#define SPI3_MISO_PIN        PB4
#define SPI3_MOSI_PIN        PB5
#define USE_SPI_DEVICE_4
#define SPI4_SCK_PIN         PE12
#define SPI4_MISO_PIN        PE13
#define SPI4_MOSI_PIN        PE14

#define USE_SPI_GYRO
#define USE_EXTI
#define USE_GYRO_EXTI

#define USE_MPU_DATA_READY_SIGNAL
#define ENSURE_MPU_DATA_READY_IS_LOW

#define ACC_1_ALIGN          CW0_DEG_FLIP
#define GYRO_1_ALIGN         CW0_DEG_FLIP
#define GYRO_1_CS_PIN        PC15
#define GYRO_1_EXTI_PIN      PB2
#define GYRO_1_SPI_BUS SPIDEV_1

#define USE_DUAL_GYRO

#define ACC_2_ALIGN          CW90_DEG_FLIP
#define GYRO_2_ALIGN         CW90_DEG_FLIP
#define GYRO_2_CS_PIN        PC13
#define GYRO_2_EXTI_PIN      PE15
#define GYRO_2_SPI_BUS SPIDEV_4

#define USE_UART1
#define UART1_TX_PIN         PA9
#define UART1_RX_PIN         PA10
#define USE_UART2
#define UART2_TX_PIN         PD5
#define UART2_RX_PIN         PD6
#define USE_UART3
#define UART3_TX_PIN         PD8
#define UART3_RX_PIN         PD9
#define USE_UART4
#define UART4_TX_PIN         PB9
#define UART4_RX_PIN         PB8
#define USE_UART6
#define UART6_TX_PIN         PC6
#define UART6_RX_PIN         PC7
#define USE_UART7
#define UART7_TX_PIN         PE8
#define UART7_RX_PIN         PE7
#define USE_UART8
#define UART8_TX_PIN         PE1
#define UART8_RX_PIN         PE0
#define SERIALRX_UART SERIAL_PORT_USART6
#define SERIAL_PORT_COUNT 8

#define USE_I2C
#define USE_I2C_DEVICE_1
#define I2C_DEVICE_1      (I2CDEV_1)
#define MAG_I2C_INSTANCE I2CDEV_1
#define I2C1_SCL PB6
#define I2C1_SDA PB7
#define USE_I2C_DEVICE_2
#define I2C_DEVICE_2      (I2CDEV_2)
#define BARO_I2C_INSTANCE I2CDEV_2
#define I2C2_SCL PB10
#define I2C2_SDA PB11

#define USE_SDCARD_SDIO
//notice - NEED: #define SDCARD_DMA_CHANNEL          X            // please verify
//notice - NEED: #define SDCARD_DMA_CHANNEL_TX       DMAx_StreamX // please verify
//notice - other sdcard defines maybe needed (rare?): SDCARD_DMA_STREAM_TX_FULL, SDCARD_DMA_STREAM_TX, SDCARD_DMA_CLK, SDCARD_DMA_CHANNEL_TX_COMPLETE_FLAG
#define ENABLE_BLACKBOX_LOGGING_ON_SDCARD_BY_DEFAULT
#define SDCARD_SPI_FULL_SPEED_CLOCK_DIVIDER     4    //notice - needs validation. these are hardware dependent. known options: 2, 4, 8.
#define SDCARD_SPI_INITIALIZATION_CLOCK_DIVIDER 256  //notice - needs validation. these are hardware dependent. known options: 128, 256

#define MAX7456_SPI_CS_PIN   PB12
#define MAX7456_SPI_INSTANCE SPI2

#define USE_ADC
#define VBAT_ADC_PIN PC0
#define CURRENT_METER_ADC_PIN PC1
#define RSSI_ADC_PIN PC5
#define ADC1_DMA_OPT        9
#define ADC3_DMA_OPT        10
#define ADC1_DMA_STREAM DMA2_Stream1 // ADC1 opt9
#define ADC3_DMA_STREAM DMA2_Stream2 // ADC3 opt10
#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define DEFAULT_CURRENT_METER_SOURCE CURRENT_METER_ADC
#define DEFAULT_CURRENT_METER_SCALE 250

#define PINIO1_PIN           PD10
#define PINIO2_PIN           PD11
#define PINIO1_BOX 40
#define PINIO2_BOX 41

#define TARGET_IO_PORTA 0xffff
#define TARGET_IO_PORTB 0xffff
#define TARGET_IO_PORTC 0xffff
#define TARGET_IO_PORTD 0xffff
#define TARGET_IO_PORTE 0xffff
// notice - port masks derived from config.h; single-pin ports use exact mask, multi-pin ports use 0xffff

#define DEFAULT_FEATURES       (FEATURE_OSD | FEATURE_TELEMETRY | FEATURE_AIRMODE | FEATURE_RX_SERIAL)
#define DEFAULT_RX_FEATURE     FEATURE_RX_SERIAL

#define USABLE_TIMER_CHANNEL_COUNT 13
#define USED_TIMERS ( TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(4) | TIM_N(5) | TIM_N(8) | TIM_N(15) )

// notice - this file was programmatically generated and may be incomplete.
