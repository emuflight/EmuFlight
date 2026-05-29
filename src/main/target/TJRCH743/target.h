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

#define BOARD_NAME           TJRCH743
#define MANUFACTURER_ID      TJRC
#define TARGET_BOARD_IDENTIFIER "SH74"  // generic ID
#define FC_TARGET_MCU        STM32H743     // not used in EmuF

#define USE_ACC
#define USE_ACC_SPI_ICM42688P
#define USE_GYRO
#define USE_GYRO_SPI_ICM42688P
#define USE_FLASH
#define USE_FLASH_M25P16
#define USE_BARO
#define USE_BARO_DPS310
// #define USE_GYRO_CLKIN // not supported in EmuFlight

#define USE_VCP
#define USE_FLASHFS
#define USE_FLASH_M25P16    // 16MB Micron M25P16 driver; drives all unless QSPI

#define USE_LED
#define LED0_PIN             PE3
#define LED1_PIN             PE4
#define LED_STRIP_PIN        PA8          // TIM1_CH1
#define USE_BEEPER
#define BEEPER_PIN           PE9
#define BEEPER_INVERTED

#define USE_SPI
#define USE_SPI_DEVICE_1
#define SPI1_SCK_PIN         PA5
#define SPI1_MISO_PIN        PA6
#define SPI1_MOSI_PIN        PD7
#define USE_SPI_DEVICE_3
#define SPI3_SCK_PIN         PB3
#define SPI3_MISO_PIN        PB4
#define SPI3_MOSI_PIN        PB5

#define USE_SPI_GYRO
#define USE_EXTI
#define USE_GYRO_EXTI

#define MPU_INT_EXTI         PB2

#define ACC_ICM42688P_ALIGN      CW270_DEG
#define GYRO_ICM42688P_ALIGN     CW270_DEG
#define ICM42688P_CS_PIN         PC15
#define ICM42688P_SPI_BUS        SPIDEV_1

#define USE_UART1
#define UART1_RX_PIN         PA10
#define USE_UART2
#define UART2_RX_PIN         PD6
#define USE_UART3
#define UART3_TX_PIN         PD8
#define UART3_RX_PIN         PD9
#define USE_UART4
#define UART4_TX_PIN         PB9
#define UART4_RX_PIN         PB8
#define USE_UART7
#define UART7_TX_PIN         PE8
#define UART7_RX_PIN         PE7
#define USE_UART8
#define UART8_TX_PIN         PE1
#define UART8_RX_PIN         PE0
#define SERIAL_PORT_COUNT 7

#define USE_I2C
#define USE_I2C_DEVICE_1
#define I2C_DEVICE_1      (I2CDEV_1)
#define MAG_I2C_INSTANCE                    I2CDEV_1
#define I2C1_SCL PB6
#define I2C1_SDA PB7
#define USE_I2C_DEVICE_2
#define I2C_DEVICE_2      (I2CDEV_2)
#define BARO_I2C_INSTANCE                   I2CDEV_2
#define I2C2_SCL PB10
#define I2C2_SDA PB11

#define FLASH_CS_PIN         PE2
#define FLASH_SPI_INSTANCE                  SPI3
#define ENABLE_BLACKBOX_LOGGING_ON_SPIFLASH_BY_DEFAULT

#define USE_ADC
#define VBAT_ADC_PIN PC0
#define CURRENT_METER_ADC_PIN PC1
#define ADC1_DMA_OPT                        9
#define ADC2_DMA_OPT                        10
#define ADC3_DMA_OPT                        11
#define ADC1_DMA_STREAM DMA2_Stream1 // ADC1 opt9
#define ADC2_DMA_STREAM DMA2_Stream2 // ADC2 opt10
#define ADC3_DMA_STREAM DMA2_Stream3 // ADC3 opt11
#define DEFAULT_VOLTAGE_METER_SOURCE        VOLTAGE_METER_ADC
#define DEFAULT_CURRENT_METER_SOURCE        CURRENT_METER_ADC
#define DEFAULT_CURRENT_METER_SCALE 350

#define PINIO1_PIN           PD10
#define PINIO1_CONFIG                       129
#define PINIO1_BOX                          40

#define TARGET_IO_PORTA 0xffff
#define TARGET_IO_PORTB 0xffff
#define TARGET_IO_PORTC 0xffff
#define TARGET_IO_PORTD 0xffff
#define TARGET_IO_PORTE 0xffff
// notice - port masks derived from config.h; single-pin ports use exact mask, multi-pin ports use 0xffff

#define DEFAULT_FEATURES       (FEATURE_OSD | FEATURE_TELEMETRY | FEATURE_AIRMODE | FEATURE_RX_SERIAL)
#define DEFAULT_RX_FEATURE     FEATURE_RX_SERIAL

#define USABLE_TIMER_CHANNEL_COUNT 7
#define USED_TIMERS ( TIM_N(1) | TIM_N(2) | TIM_N(5) | TIM_N(16) )

// notice - this file was programmatically generated and may be incomplete.
