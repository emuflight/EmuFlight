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

#define BOARD_NAME        FF_FORTINIF4_REV03
#define MANUFACTURER_ID   FFPV
#define TARGET_BOARD_IDENTIFIER "S405"  // generic ID
#define FC_TARGET_MCU     STM32F405     // not used in EmuF

#define USE_ACC
#define USE_ACC_SPI_ICM20689
#define USE_ACC_SPI_MPU6500
#define USE_ACC_SPI_MPU6000
#define USE_GYRO
#define USE_GYRO_SPI_ICM20689
#define USE_GYRO_SPI_MPU6500
#define USE_GYRO_SPI_MPU6000
#define USE_MAX7456

#define USE_VCP
#define USE_OSD

#define USE_LED
#define LED0_PIN             PB5
#define LED1_PIN             PB4
#define LED_STRIP_PIN        PB6
#define USE_BEEPER
#define BEEPER_PIN           PB3
#define BEEPER_INVERTED
#define CAMERA_CONTROL_PIN   PA10

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

#define MPU_INT_EXTI         PC4

#define ACC_MPU6000_ALIGN        CW180_DEG
#define GYRO_MPU6000_ALIGN       CW180_DEG
#define MPU6000_CS_PIN           PA4
#define MPU6000_SPI_INSTANCE     SPI1

#define ACC_MPU6500_ALIGN        CW180_DEG
#define GYRO_MPU6500_ALIGN       CW180_DEG
#define MPU6500_CS_PIN           PA4
#define MPU6500_SPI_INSTANCE     SPI1

#define ACC_ICM20689_ALIGN       CW180_DEG
#define GYRO_ICM20689_ALIGN      CW180_DEG
#define ICM20689_CS_PIN          PA4
#define ICM20689_SPI_INSTANCE    SPI1

#define USE_UART1
#define UART1_TX_PIN         PA9
#define UART1_RX_PIN         PB7
#define USE_UART2
#define UART2_TX_PIN         PA2
#define UART2_RX_PIN         PA3
#define USE_UART3
#define UART3_TX_PIN         PC10
#define UART3_RX_PIN         PC11
#define USE_UART4
#define UART4_TX_PIN         PA0
#define UART4_RX_PIN         PA1
#define USE_UART5
#define UART5_TX_PIN         PC12
#define UART5_RX_PIN         PD2
#define USE_UART6
#define UART6_TX_PIN         PC6
#define UART6_RX_PIN         PC7
#define INVERTER_PIN_UART3   PC15
#define SERIAL_PORT_COUNT 7

#define USE_I2C
#define USE_I2C_DEVICE_3
#define I2C_DEVICE_3      (I2CDEV_3)
#define DASHBOARD_I2C_INSTANCE (I2CDEV_3)
#define I2C3_SCL PA8
#define I2C3_SDA PC9

#define MAX7456_SPI_CS_PIN   PB12
#define MAX7456_SPI_INSTANCE SPI2

#define USE_ADC
#define VBAT_ADC_PIN PC2
#define CURRENT_METER_ADC_PIN PC1
#define ADC1_DMA_OPT        1
#define ADC1_DMA_STREAM DMA2_Stream4 //# ADC 1: DMA2 Stream 4 Channel 0
#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define DEFAULT_CURRENT_METER_SOURCE CURRENT_METER_ADC

#define USE_ESCSERIAL
#define ESCSERIAL_TIMER_TX_PIN PB0


#define TARGET_IO_PORTA 0xffff
#define TARGET_IO_PORTB 0xffff
#define TARGET_IO_PORTC 0xffff
#define TARGET_IO_PORTD 0xffff
// notice - masks were programmatically generated - please verify last port group for 0xffff or (BIT(2))

#define DEFAULT_FEATURES       (FEATURE_OSD | FEATURE_TELEMETRY | FEATURE_AIRMODE | FEATURE_RX_SERIAL)
#define DEFAULT_RX_FEATURE     FEATURE_RX_SERIAL

#define USABLE_TIMER_CHANNEL_COUNT 7
#define USED_TIMERS ( TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(4) )

// notice - this file was programmatically generated and may be incomplete.
