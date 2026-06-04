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

#define BOARD_NAME        AEROH7DIGITAL
#define MANUFACTURER_ID   AERO
#define TARGET_BOARD_IDENTIFIER "SH74"  // generic ID
#define FC_TARGET_MCU     STM32H743     // not used in EmuF

#define USE_ACC
#define USE_ACC_SPI_ICM42688P
#define USE_GYRO
#define USE_GYRO_SPI_ICM42688P
// #define USE_GYRO_CLKIN // not supported in EmuFlight
#define USE_BARO
#define USE_BARO_DPS310
#define USE_SDCARD

#define USE_VCP

#define USE_LED
#define LED0_PIN             PD8  // Blue
#define LED1_PIN             PB15 // Green
#define LED2_PIN             PB14 // Amber

#define USE_SPI
#define USE_SPI_DEVICE_1
#define SPI1_SCK_PIN         PA5
#define SPI1_MISO_PIN        PA6
#define SPI1_MOSI_PIN        PA7

#define USE_SPI_GYRO
#define USE_EXTI
#define USE_GYRO_EXTI

#define MPU_INT_EXTI         PB1

#define ACC_ICM42688P_ALIGN      CW90_DEG
#define GYRO_ICM42688P_ALIGN     CW90_DEG
#define ICM42688P_CS_PIN         PB0
#define ICM42688P_SPI_BUS        SPIDEV_1

#define USE_UART2
#define UART2_TX_PIN         PA2
#define UART2_RX_PIN         PA3
#define USE_UART3
#define UART3_RX_PIN         PD9 // ESC TELEMETRY
#define USE_UART4
#define UART4_TX_PIN         PD1
#define UART4_RX_PIN         PD0
#define USE_UART5
#define UART5_TX_PIN         PB13
#define UART5_RX_PIN         PB12
#define USE_UART7
#define UART7_RX_PIN         PE7 // SBUS DJI
#define USE_UART8
#define UART8_TX_PIN         PE1
#define UART8_RX_PIN         PE0
#define MSP_DISPLAYPORT_UART         SERIAL_PORT_USART2
#define ESC_SENSOR_UART              SERIAL_PORT_USART3
#define SERIAL_PORT_COUNT 7

#define USE_I2C
#define USE_I2C_DEVICE_1
#define I2C_DEVICE_1      (I2CDEV_1)
#define BARO_I2C_INSTANCE            I2CDEV_1
#define I2C1_SCL PB8
#define I2C1_SDA PB9
#define USE_I2C_DEVICE_4
#define I2C_DEVICE_4      (I2CDEV_4)
#define MAG_I2C_INSTANCE             I2CDEV_4
#define I2C4_SCL PB6
#define I2C4_SDA PB7

#define USE_SDCARD_SDIO
//notice - NEED: #define SDCARD_DMA_CHANNEL          X            // please verify
//notice - NEED: #define SDCARD_DMA_CHANNEL_TX       DMAx_StreamX // please verify
//notice - other sdcard defines maybe needed (rare?): SDCARD_DMA_STREAM_TX_FULL, SDCARD_DMA_STREAM_TX, SDCARD_DMA_CLK, SDCARD_DMA_CHANNEL_TX_COMPLETE_FLAG
#define ENABLE_BLACKBOX_LOGGING_ON_SDCARD_BY_DEFAULT
#define SDCARD_SPI_FULL_SPEED_CLOCK_DIVIDER     4    //notice - needs validation. these are hardware dependent. known options: 2, 4, 8.
#define SDCARD_SPI_INITIALIZATION_CLOCK_DIVIDER 256  //notice - needs validation. these are hardware dependent. known options: 128, 256

#define USE_ADC
#define VBAT_ADC_PIN PA0
#define CURRENT_METER_ADC_PIN PA1
#define ADC1_DMA_OPT                 8
#define ADC3_DMA_OPT                 9
#define ADC1_DMA_STREAM DMA2_Stream0 // ADC1 opt8
#define ADC3_DMA_STREAM DMA2_Stream1 // ADC3 opt9
#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define DEFAULT_CURRENT_METER_SOURCE CURRENT_METER_ADC

#define PINIO1_PIN           PE2
#define PINIO1_BOX                   40
#define PINIO1_CONFIG                129

#define TARGET_IO_PORTA 0xffff
#define TARGET_IO_PORTB 0xffff
#define TARGET_IO_PORTC 0xffff
#define TARGET_IO_PORTD 0xffff
#define TARGET_IO_PORTE 0xffff
// notice - port masks derived from config.h; single-pin ports use exact mask, multi-pin ports use 0xffff

#define DEFAULT_FEATURES       (FEATURE_OSD | FEATURE_TELEMETRY | FEATURE_AIRMODE | FEATURE_RX_SERIAL)
#define DEFAULT_RX_FEATURE     FEATURE_RX_SERIAL

#define USABLE_TIMER_CHANNEL_COUNT 11
#define USED_TIMERS ( TIM_N(1) | TIM_N(3) | TIM_N(4) | TIM_N(15) )

// notice - this file was programmatically generated and may be incomplete.
