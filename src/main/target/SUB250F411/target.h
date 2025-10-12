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

#define BOARD_NAME                      SUB250F411
#define MANUFACTURER_ID                 SU25
#define TARGET_BOARD_IDENTIFIER "S411"  // generic ID
#define FC_TARGET_MCU                   STM32F411     // not used in EmuF

#define USE_ACC
#define USE_ACC_SPI_ICM42688P
#define USE_ACCGYRO_BMI270
#define USE_GYRO
#define USE_GYRO_SPI_ICM42688P
#define USE_BARO
#define USE_BARO_DPS310
#define USE_BARO_BMP280
#define USE_FLASH
#define USE_FLASH_W25Q128FV
#define USE_MAX7456

#define USE_VCP
#define USE_FLASHFS
#define USE_FLASH_M25P16    // 16MB Micron M25P16 driver; drives all unless QSPI
#define USE_OSD

#define USE_LED
#define LED0_PIN                        PC13
#define LED1_PIN                        PC14
#define LED_STRIP_PIN                   PA8
#define USE_BEEPER
#define BEEPER_PIN                      PB2
#define BEEPER_INVERTED

#define USE_SPI
#define USE_SPI_DEVICE_1
#define SPI1_SCK_PIN                    PA5
#define SPI1_MISO_PIN        PA6
#define SPI1_MOSI_PIN        PA7
#define USE_SPI_DEVICE_2
#define SPI2_SCK_PIN                    PB13
#define SPI2_MISO_PIN        PB14
#define SPI2_MOSI_PIN        PB15

#define USE_SPI_GYRO
#define USE_EXTI
#define USE_GYRO_EXTI

#define MPU_INT_EXTI         PA1

#define ACC_ICM42688P_ALIGN      CW270_DEG
#define GYRO_ICM42688P_ALIGN     CW270_DEG
#define ICM42688P_CS_PIN         PA4
#define ICM42688P_SPI_INSTANCE   SPI1

#define ACC_BMI270_ALIGN         CW270_DEG
#define GYRO_BMI270_ALIGN        CW270_DEG
#define BMI270_CS_PIN            PA4
#define BMI270_SPI_INSTANCE      SPI1

#define USE_UART1
#define UART1_TX_PIN                    PA9
#define UART1_RX_PIN                    PA10
#define USE_UART2
#define UART2_TX_PIN                    PA2
#define UART2_RX_PIN                    PA3
#define INVERTER_PIN_UART2              PC15
#define SERIALRX_UART                   SERIAL_PORT_USART2
#define SERIAL_PORT_COUNT 3

#define USE_I2C
#define USE_I2C_DEVICE_1
#define I2C_DEVICE_1      (I2CDEV_1)
#define BARO_I2C_INSTANCE               (I2CDEV_1)
#define I2C1_SCL PB8
#define I2C1_SDA PB9

#define FLASH_CS_PIN                    PA0
#define FLASH_SPI_INSTANCE              SPI2
#define ENABLE_BLACKBOX_LOGGING_ON_SPIFLASH_BY_DEFAULT

#define MAX7456_SPI_CS_PIN              PB12
#define MAX7456_SPI_INSTANCE            SPI2

#define USE_ADC
#define VBAT_ADC_PIN PB0
#define CURRENT_METER_ADC_PIN PB1
#define ADC1_DMA_OPT                    0
#define ADC1_DMA_STREAM DMA2_Stream0 //# ADC 1: DMA2 Stream 0 Channel 0
#define DEFAULT_VOLTAGE_METER_SOURCE    VOLTAGE_METER_ADC
#define DEFAULT_CURRENT_METER_SOURCE    CURRENT_METER_ADC
#define DEFAULT_CURRENT_METER_SCALE     500


#define PINIO1_PIN                      PB10
#define PINIO1_BOX                      40

#define TARGET_IO_PORTA 0xffff
#define TARGET_IO_PORTB 0xffff
#define TARGET_IO_PORTC 0xffff
// notice - masks were programmatically generated - please verify last port group for 0xffff or (BIT(2))

#define DEFAULT_FEATURES       (FEATURE_OSD | FEATURE_TELEMETRY | FEATURE_AIRMODE | FEATURE_RX_SERIAL)
#define DEFAULT_RX_FEATURE     FEATURE_RX_SERIAL

#define USABLE_TIMER_CHANNEL_COUNT 6
#define USED_TIMERS ( TIM_N(1) | TIM_N(3) | TIM_N(4) | TIM_N(9) )

// notice - this file was programmatically generated and may be incomplete.
