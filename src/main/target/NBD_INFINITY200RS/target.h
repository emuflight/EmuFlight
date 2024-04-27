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

#define BOARD_NAME NBD_INFINITY200RS
#define MANUFACTURER_ID NEBD
#define TARGET_BOARD_IDENTIFIER "S745"  // generic ID
#define FC_TARGET_MCU     STM32F745     // not used in EmuF

#define USE_GYRO
#define USE_GYRO_SPI_MPU6000
#define USE_ACC
#define USE_ACC_SPI_MPU6000
#define USE_ACCGYRO_BMI270
#define USE_FLASH
#define USE_FLASH_W25N01G
#define USE_MAX7456

#define USE_VCP
#define USE_FLASHFS
#define USE_FLASH_M25P16    // 16MB Micron M25P16 driver; drives all unless QSPI
#define USE_OSD

#define USE_LED
#define LED0_PIN PC0
#define LED_STRIP_PIN PA9

#define USE_SPI
#define USE_SPI_DEVICE_1
#define SPI1_SCK_PIN PA5
#define SPI1_MISO_PIN        PA6
#define SPI1_MOSI_PIN        PA7
#define USE_SPI_DEVICE_2
#define SPI2_SCK_PIN PB13
#define SPI2_MISO_PIN        PB14
#define SPI2_MOSI_PIN        PB15
#define USE_SPI_DEVICE_3
#define SPI3_SCK_PIN PB3
#define SPI3_MISO_PIN        PB4
#define SPI3_MOSI_PIN        PD6
#define USE_SPI_DEVICE_4
#define SPI4_SCK_PIN PE12
#define SPI4_MISO_PIN        PE13
#define SPI4_MOSI_PIN        PE14

#define USE_SPI_GYRO
#define USE_EXTI
#define USE_GYRO_EXTI

#define USE_MPU_DATA_READY_SIGNAL

#define ACC_1_ALIGN          CW90_DEG
#define GYRO_1_ALIGN         CW90_DEG
#define GYRO_1_CS_PIN PE11
#define GYRO_1_EXTI_PIN PB1
#define GYRO_1_SPI_INSTANCE SPI4

#define USE_DUAL_GYRO

#define ACC_2_ALIGN          CW0_DEG
#define GYRO_2_ALIGN         CW0_DEG
#define GYRO_2_CS_PIN PB12
#define GYRO_2_EXTI_PIN PE9
#define GYRO_2_SPI_INSTANCE SPI2

#define USE_UART1
#define UART1_TX_PIN PB6
#define UART1_RX_PIN PB7
#define USE_UART2
#define UART2_TX_PIN PA2
#define UART2_RX_PIN PA3
#define USE_UART3
#define UART3_TX_PIN PB10
#define USE_UART5
#define UART5_RX_PIN PD2
#define USE_UART7
#define UART7_TX_PIN PE8
#define UART7_RX_PIN PE7
#define USE_UART8
#define UART8_TX_PIN PE1
#define UART8_RX_PIN PE0
#define VTX_SMARTAUDIO_UART SERIAL_PORT_USART2
#define SERIAL_PORT_COUNT 7

#define USE_I2C
#define USE_I2C_DEVICE_1
#define I2C_DEVICE_1      (I2CDEV_1)
#define I2C1_SCL PB8
#define I2C1_SDA PB9

#define FLASH_CS_PIN PB0
#define FLASH_SPI_INSTANCE SPI1
#define ENABLE_BLACKBOX_LOGGING_ON_SPIFLASH_BY_DEFAULT

#define MAX7456_SPI_CS_PIN PA15
#define MAX7456_SPI_INSTANCE SPI3

#define USE_ADC
#define VBAT_ADC_PIN PC1
#define CURRENT_METER_ADC_PIN PC2
#define ADC1_DMA_OPT 0
#define ADC1_DMA_STREAM DMA2_Stream0 //# ADC 1: DMA2 Stream 0 Channel 0
#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define DEFAULT_CURRENT_METER_SOURCE CURRENT_METER_ADC
#define DEFAULT_CURRENT_METER_SCALE 230

#define ENABLE_DSHOT_DMAR true


#define TARGET_IO_PORTA 0xffff
#define TARGET_IO_PORTB 0xffff
#define TARGET_IO_PORTC 0xffff
#define TARGET_IO_PORTD 0xffff
#define TARGET_IO_PORTE 0xffff
// notice - masks were programmatically generated - please verify last port group for 0xffff or (BIT(2))

#define DEFAULT_FEATURES       (FEATURE_OSD | FEATURE_TELEMETRY | FEATURE_AIRMODE | FEATURE_RX_SERIAL)
#define DEFAULT_RX_FEATURE     FEATURE_RX_SERIAL

#define USABLE_TIMER_CHANNEL_COUNT 6
#define USED_TIMERS ( TIM_N(1) | TIM_N(2) | TIM_N(3) )

// notice - this file was programmatically generated and may be incomplete.
