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

#define BOARD_NAME        WPLV2
#define MANUFACTURER_ID   WRPL
#define TARGET_BOARD_IDENTIFIER "SH74"  // generic ID
#define FC_TARGET_MCU     STM32H743     // not used in EmuF

#define USE_GYRO
#define USE_GYRO_SPI_ICM42688P
// #define USE_GYRO_CLKIN // not supported in EmuFlight
#define USE_ACC
#define USE_ACC_SPI_ICM42688P
#define USE_FLASH
#define USE_FLASH_W25Q128FV

#define USE_VCP
#define USE_FLASHFS
#define USE_FLASH_M25P16    // 16MB Micron M25P16 driver; drives all unless QSPI

#define USE_LED
#define LED0_PIN PE2
#define LED_STRIP_PIN PA15
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

#define MPU_INT_EXTI         PC13

#define ACC_ICM42688P_ALIGN      CW0_DEG
#define GYRO_ICM42688P_ALIGN     CW0_DEG
#define ICM42688P_CS_PIN         PB4
#define ICM42688P_SPI_BUS        SPIDEV_1

#define USE_UART1
#define UART1_TX_PIN PB6
#define UART1_RX_PIN PB7
#define USE_UART2
#define UART2_TX_PIN PD5
#define UART2_RX_PIN PD6
#define USE_UART3
#define UART3_RX_PIN PD9
#define USE_UART4
#define UART4_RX_PIN PD0
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
#define MSP_DISPLAYPORT_UART            SERIAL_PORT_USART2
#define SERIAL_PORT_COUNT 8

#define USE_I2C
#define USE_I2C_DEVICE_1
#define I2C_DEVICE_1      (I2CDEV_1)
#define MAG_I2C_INSTANCE I2CDEV_1
#define BARO_I2C_INSTANCE I2CDEV_1
#define I2C1_SCL PB8
#define I2C1_SDA PB9

#define FLASH_CS_PIN PB12
#define FLASH_SPI_INSTANCE              SPI2
#define ENABLE_BLACKBOX_LOGGING_ON_SPIFLASH_BY_DEFAULT

#define USE_ADC
#define VBAT_ADC_PIN PC0
#define CURRENT_METER_ADC_PIN PC2
#define ADC1_DMA_OPT   11
#define ADC2_DMA_OPT   12
#define ADC3_DMA_OPT   13
#define ADC1_DMA_STREAM DMA2_Stream3 // ADC1 opt11
#define ADC2_DMA_STREAM DMA2_Stream4 // ADC2 opt12
#define ADC3_DMA_STREAM DMA2_Stream5 // ADC3 opt13
#define DEFAULT_VOLTAGE_METER_SOURCE    VOLTAGE_METER_ADC
#define DEFAULT_CURRENT_METER_SOURCE    CURRENT_METER_ADC

#define VTX_ENABLE_PIN PE3
#define PINIO1_PIN VTX_ENABLE_PIN
#define PINIO1_BOX 40
#define PINIO1_CONFIG 129
#define PINIO2_PIN PE4                  
#define PINIO2_BOX 41
#define PINIO2_CONFIG 1

#define TARGET_IO_PORTA 0xffff
#define TARGET_IO_PORTB 0xffff
#define TARGET_IO_PORTC 0xffff
#define TARGET_IO_PORTD 0xffff
#define TARGET_IO_PORTE 0xffff
// notice - port masks derived from config.h; single-pin ports use exact mask, multi-pin ports use 0xffff

#define DEFAULT_FEATURES       (FEATURE_OSD | FEATURE_TELEMETRY | FEATURE_AIRMODE | FEATURE_RX_SERIAL)
#define DEFAULT_RX_FEATURE     FEATURE_RX_SERIAL

#define USABLE_TIMER_CHANNEL_COUNT 12
#define USED_TIMERS ( TIM_N(1) | TIM_N(2) | TIM_N(5) | TIM_N(8) | TIM_N(15) )

// notice - this file was programmatically generated and may be incomplete.
