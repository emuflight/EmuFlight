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

#define BOARD_NAME        HDZERO_HALO
#define MANUFACTURER_ID   HDZO
#define TARGET_BOARD_IDENTIFIER "SH74"  // generic ID
#define FC_TARGET_MCU     STM32H743     // not used in EmuF

#define USE_GYRO
#define USE_GYRO_SPI_MPU6000
#define USE_GYRO_SPI_ICM42688P
#define USE_ACC
#define USE_ACC_SPI_MPU6000
#define USE_ACC_SPI_ICM42688P
#define USE_FLASH
#define USE_FLASH_W25Q128FV
#define USE_OSD_HD
#define USE_LED_STRIP

#define USE_VCP
#define USE_FLASHFS
#define USE_FLASH_M25P16    // 16MB Micron M25P16 driver; drives all unless QSPI

#define USE_LED
#define LED0_PIN PE2
#define LED_STRIP_PIN PA10
#define USE_BEEPER
#define BEEPER_PIN PD12
#define BEEPER_INVERTED

#define USE_SPI
#define USE_SPI_DEVICE_1
#define SPI1_SCK_PIN PA5
#define SPI1_MISO_PIN        PA6
#define SPI1_MOSI_PIN        PA7
#define USE_SPI_DEVICE_2
#define SPI2_SCK_PIN PB13
#define SPI2_MISO_PIN        PB14
#define SPI2_MOSI_PIN        PB15

#define USE_SPI_GYRO
#define USE_EXTI
#define USE_GYRO_EXTI

#define USE_MPU_DATA_READY_SIGNAL

#define ACC_1_ALIGN          CW0_DEG
#define GYRO_1_ALIGN         CW0_DEG
#define GYRO_1_CS_PIN PB4
#define GYRO_1_EXTI_PIN PC13
#define GYRO_1_SPI_BUS             SPIDEV_1

#define USE_DUAL_GYRO

#define ACC_2_ALIGN          CW0_DEG
#define GYRO_2_ALIGN         CW0_DEG
#define GYRO_2_CS_PIN PB3
#define GYRO_2_EXTI_PIN PA15
#define GYRO_2_SPI_BUS             SPIDEV_1

#define USE_UART1
#define UART1_TX_PIN PB6
#define UART1_RX_PIN PB7
#define USE_UART2
#define UART2_TX_PIN PA2
#define UART2_RX_PIN PA3
#define USE_UART3
#define UART3_RX_PIN PB11
#define USE_UART4
#define UART4_RX_PIN PA1
#define USE_UART5
#define UART5_TX_PIN PC12
#define UART5_RX_PIN PD2
#define USE_UART7
#define UART7_TX_PIN PE8
#define UART7_RX_PIN PE7
#define USE_UART8
#define UART8_TX_PIN PE1
#define UART8_RX_PIN PE0
#define SERIALRX_UART                   SERIAL_PORT_USART1
#define SERIAL_PORT_COUNT 8

#define USE_I2C
#define USE_I2C_DEVICE_1
#define I2C_DEVICE_1      (I2CDEV_1)
#define I2C1_SCL PB8
#define I2C1_SDA PB9

#define FLASH_CS_PIN PB12
#define FLASH_SPI_INSTANCE              SPI2
#define ENABLE_BLACKBOX_LOGGING_ON_SPIFLASH_BY_DEFAULT

#define USE_ADC
#define VBAT_ADC_PIN PC0
#define CURRENT_METER_ADC_PIN PC2
#define RSSI_ADC_PIN PC1
#define ADC1_DMA_OPT 8
#define ADC2_DMA_OPT 9
#define ADC3_DMA_OPT 10
#define ADC1_DMA_STREAM DMA2_Stream0 // ADC1 opt8
#define ADC2_DMA_STREAM DMA2_Stream1 // ADC2 opt9
#define ADC3_DMA_STREAM DMA2_Stream2 // ADC3 opt10
#define DEFAULT_VOLTAGE_METER_SOURCE    VOLTAGE_METER_ADC
#define DEFAULT_CURRENT_METER_SOURCE    CURRENT_METER_ADC
#define DEFAULT_CURRENT_METER_SCALE 100


#define TARGET_IO_PORTA 0xffff
#define TARGET_IO_PORTB 0xffff
#define TARGET_IO_PORTC 0xffff
#define TARGET_IO_PORTD 0xffff
#define TARGET_IO_PORTE 0xffff
// notice - port masks derived from config.h; single-pin ports use exact mask, multi-pin ports use 0xffff

#define DEFAULT_FEATURES       (FEATURE_OSD | FEATURE_TELEMETRY | FEATURE_AIRMODE | FEATURE_RX_SERIAL)
#define DEFAULT_RX_FEATURE     FEATURE_RX_SERIAL

#define USABLE_TIMER_CHANNEL_COUNT 6
#define USED_TIMERS ( TIM_N(1) | TIM_N(3) | TIM_N(4) )

// notice - this file was programmatically generated and may be incomplete.
