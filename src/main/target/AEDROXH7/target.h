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

#define BOARD_NAME                      AEDROXH7
#define MANUFACTURER_ID                 AEDR
#define TARGET_BOARD_IDENTIFIER "SH74"  // generic ID
#define FC_TARGET_MCU                   STM32H743     // not used in EmuF

#define USE_GYRO
#define USE_GYRO_SPI_ICM42688P
#define USE_ACC
#define USE_ACC_SPI_ICM42688P
// #define USE_GYRO_CLKIN // not supported in EmuFlight
#define USE_FLASH
#define USE_FLASH_W25Q128FV
#define USE_FLASH_M25P16
#define USE_BARO
#define USE_BARO_DPS310

#define USE_VCP
#define USE_FLASHFS
#define USE_FLASH_M25P16    // 16MB Micron M25P16 driver; drives all unless QSPI

#define USE_LED
#define LED0_PIN                        PE5
#define LED1_PIN                        PE4
#define LED_STRIP_PIN                   PA5
#define USE_BEEPER
#define BEEPER_PIN                      PA7
#define BEEPER_INVERTED

#define USE_SPI
#define USE_SPI_DEVICE_2
#define SPI2_SCK_PIN                    PB13
#define SPI2_MISO_PIN        PB14
#define SPI2_MOSI_PIN        PB15
#define USE_SPI_DEVICE_3
#define SPI3_SCK_PIN                    PC10
#define SPI3_MISO_PIN        PC11
#define SPI3_MOSI_PIN        PB2

#define USE_SPI_GYRO
#define USE_EXTI
#define USE_GYRO_EXTI

#define MPU_INT_EXTI         PC4

#define ACC_ICM42688P_ALIGN      CW90_DEG
#define GYRO_ICM42688P_ALIGN     CW90_DEG
#define ICM42688P_CS_PIN         PA4
#define ICM42688P_SPI_BUS        SPIDEV_2

#define USE_UART1
#define UART1_TX_PIN                    PA9
#define UART1_RX_PIN                    PA10
#define USE_UART2
#define UART2_TX_PIN                    PD5
#define UART2_RX_PIN                    PD6
#define USE_UART3
#define UART3_TX_PIN                    PD8
#define UART3_RX_PIN                    PD9
#define USE_UART4
#define UART4_TX_PIN                    PD1
#define UART4_RX_PIN                    PD0
#define USE_UART7
#define UART7_RX_PIN                    PE7
#define USE_UART8
#define UART8_TX_PIN                    PE1
#define UART8_RX_PIN                    PE0
#define GPS_UART                        SERIAL_PORT_USART2
#define SERIALRX_UART                   SERIAL_PORT_USART3
#define ESC_SENSOR_UART                 SERIAL_PORT_USART7
#define MSP_DISPLAYPORT_UART            SERIAL_PORT_USART8
#define SERIAL_PORT_COUNT 7

#define USE_I2C
#define USE_I2C_DEVICE_1
#define I2C_DEVICE_1      (I2CDEV_1)
#define MAG_I2C_INSTANCE                I2CDEV_1
#define I2C1_SCL PB6
#define I2C1_SDA PB7
#define USE_I2C_DEVICE_2
#define I2C_DEVICE_2      (I2CDEV_2)
#define BARO_I2C_INSTANCE               I2CDEV_2
#define I2C2_SCL PB10
#define I2C2_SDA PB11

#define FLASH_CS_PIN                    PA15
#define FLASH_SPI_INSTANCE              SPI3
#define ENABLE_BLACKBOX_LOGGING_ON_SPIFLASH_BY_DEFAULT

#define USE_ADC
#define VBAT_ADC_PIN PC0
#define CURRENT_METER_ADC_PIN PC1
#define RSSI_ADC_PIN PC5
#define ADC1_DMA_OPT                    8
#define ADC3_DMA_OPT                    9
#define ADC1_DMA_STREAM DMA2_Stream0 // ADC1 opt8
#define ADC3_DMA_STREAM DMA2_Stream1 // ADC3 opt9
#define DEFAULT_VOLTAGE_METER_SOURCE    VOLTAGE_METER_ADC
#define DEFAULT_CURRENT_METER_SOURCE    CURRENT_METER_ADC

#define PINIO1_PIN                      PA2
#define PINIO2_PIN                      PA3
#define PINIO3_PIN                      PB12
#define PINIO1_BOX                      40
#define PINIO2_BOX                      41
#define PINIO3_BOX                      42
#define PINIO3_CONFIG                   129

#define DEFAULT_FEATURES       (FEATURE_OSD | FEATURE_TELEMETRY | FEATURE_AIRMODE | FEATURE_RX_SERIAL)
#define DEFAULT_RX_FEATURE     FEATURE_RX_SERIAL

#define USABLE_TIMER_CHANNEL_COUNT 11
#define USE_TIMER_MGMT
#define TIMER_PIN_MAPPING \
    TIMER_PIN_MAP( 0, PC7 , 2,  0) \
    TIMER_PIN_MAP( 1, PC6 , 2,  1) \
    TIMER_PIN_MAP( 2, PC9 , 2,  2) \
    TIMER_PIN_MAP( 3, PC8 , 2,  3) \
    TIMER_PIN_MAP( 4, PE9 , 1,  4) \
    TIMER_PIN_MAP( 5, PE11 , 1,  5) \
    TIMER_PIN_MAP( 6, PE13 , 1,  6) \
    TIMER_PIN_MAP( 7, PE14 , 1,  7) \
    TIMER_PIN_MAP( 8, PA5 , 1,  10) \
    TIMER_PIN_MAP( 9, PD13 , 1,  0) \
    TIMER_PIN_MAP( 10, PA7 , 2,  0)

#define MOTOR1_PIN              PC7
#define MOTOR2_PIN              PC6
#define MOTOR3_PIN              PC9
#define MOTOR4_PIN              PC8
#define MOTOR5_PIN              PE9
#define MOTOR6_PIN              PE11
#define MOTOR7_PIN              PE13
#define MOTOR8_PIN              PE14

// notice - this file was programmatically generated and may be incomplete.
