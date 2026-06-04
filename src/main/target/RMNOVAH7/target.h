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

#define BOARD_NAME RMNOVAH7
#define MANUFACTURER_ID RDMS
#define TARGET_BOARD_IDENTIFIER "SH74"  // generic ID
#define FC_TARGET_MCU STM32H743     // not used in EmuF

#define USE_GYRO
#define USE_GYRO_SPI_ICM42688P
// #define USE_GYRO_CLKIN // not supported in EmuFlight
#define USE_ACC
#define USE_ACC_SPI_ICM42688P
#define USE_BARO
#define USE_BARO_DPS310
#define USE_FLASH
#define USE_FLASH_M25P16
#define USE_MAX7456

#define USE_VCP
#define USE_FLASHFS
#define USE_FLASH_M25P16    // 16MB Micron M25P16 driver; drives all unless QSPI
#define USE_OSD

#define USE_LED
#define  LED0_PIN            PC4
#define  LED1_PIN            PC5
#define USE_BEEPER
#define BEEPER_INVERTED
#define USE_USB_DETECT
#define  USB_DETECT_PIN      PA10

#define USE_SPI
#define USE_SPI_DEVICE_1
#define  SPI1_SCK_PIN        PB3
#define SPI1_MISO_PIN        PB4
#define SPI1_MOSI_PIN        PB5
#define USE_SPI_DEVICE_2
#define  SPI2_SCK_PIN        PB13
#define SPI2_MISO_PIN        PB14
#define SPI2_MOSI_PIN        PB15
#define USE_SPI_DEVICE_3
#define  SPI3_SCK_PIN        PC10
#define SPI3_MISO_PIN        PC11
#define SPI3_MOSI_PIN        PC12
#define USE_SPI_DEVICE_4
#define  SPI4_SCK_PIN        PE2
#define SPI4_MISO_PIN        PE5
#define SPI4_MOSI_PIN        PE6

#define USE_SPI_GYRO
#define USE_EXTI
#define USE_GYRO_EXTI

#define MPU_INT_EXTI         PD7

#define ACC_ICM42688P_ALIGN      CW0_DEG
#define GYRO_ICM42688P_ALIGN     CW0_DEG
#define ICM42688P_CS_PIN         PA15
#define ICM42688P_SPI_BUS        SPIDEV_1

#define USE_UART1
#define USE_UART2
#define USE_UART3
#define USE_UART4
#define USE_UART5
#define USE_UART6
#define USE_UART7
#define USE_UART8
#define SERIALRX_UART                   SERIAL_PORT_USART6
#define SERIAL_PORT_COUNT 9

#define USE_I2C
#define USE_I2C_DEVICE_1
#define I2C_DEVICE_1      (I2CDEV_1)
#define BARO_I2C_INSTANCE               I2CDEV_1
#define I2C1_SCL PB8
#define I2C1_SDA PB9
#define USE_I2C_DEVICE_2
#define I2C_DEVICE_2      (I2CDEV_2)
#define MAG_I2C_INSTANCE                I2CDEV_2
#define I2C2_SCL PB10
#define I2C2_SDA PB11

#define  FLASH_CS_PIN        PA8
#define FLASH_SPI_INSTANCE              SPI3
#define ENABLE_BLACKBOX_LOGGING_ON_SPIFLASH_BY_DEFAULT

#define  MAX7456_SPI_CS_PIN  PB12
#define MAX7456_SPI_INSTANCE            SPI2

#define USE_ADC
#define VBAT_ADC_PIN PC0
#define CURRENT_METER_ADC_PIN PC1
#define ADC1_DMA_OPT 10
#define ADC3_DMA_OPT 11
#define ADC1_DMA_STREAM DMA2_Stream2 // ADC1 opt10
#define ADC3_DMA_STREAM DMA2_Stream3 // ADC3 opt11
#define DEFAULT_VOLTAGE_METER_SOURCE    VOLTAGE_METER_ADC
#define DEFAULT_CURRENT_METER_SOURCE    CURRENT_METER_ADC

#define  PINIO1_PIN          PA3
#define  PINIO2_PIN          PA2
#define  PINIO3_PIN          PA1
#define PINIO1_BOX                      40
#define PINIO2_BOX                      41
#define PINIO2_CONFIG                   129
#define PINIO3_BOX                      42

#define TARGET_IO_PORTA 0xffff
#define TARGET_IO_PORTB 0xffff
#define TARGET_IO_PORTC 0xffff
#define TARGET_IO_PORTD 0xffff
#define TARGET_IO_PORTE 0xffff
// notice - port masks derived from config.h; single-pin ports use exact mask, multi-pin ports use 0xffff

#define DEFAULT_FEATURES       (FEATURE_OSD | FEATURE_TELEMETRY | FEATURE_AIRMODE | FEATURE_RX_SERIAL)
#define DEFAULT_RX_FEATURE     FEATURE_RX_SERIAL

#define USABLE_TIMER_CHANNEL_COUNT 10
#define USED_TIMERS ( TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(5) | TIM_N(8) )

// notice - this file was programmatically generated and may be incomplete.
