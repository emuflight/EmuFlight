/*
 * This file is part of EmuFlight. It is derived from Betaflight.
 *
 * This is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

// This resource file generated using https://github.com/nerdCopter/target-convert
// Commit: c4eda1b

#pragma once

#define TARGET_MANUFACTURER_IDENTIFIER "RUSH"
#define USBD_PRODUCT_STRING "BLADE_F7"

#define FC_TARGET_MCU     STM32F7X2     // not used in EmuF
#define TARGET_BOARD_IDENTIFIER "S7X2"  // generic ID

#define USE_ACC
#define USE_ACC_SPI_MPU6000
#define USE_GYRO
#define USE_GYRO_SPI_MPU6000
#define USE_GYRO_SPI_ICM42688P
#define USE_ACC_SPI_ICM42688P
#define USE_BARO
#define USE_BARO_DPS310
#define USE_BARO_BMP280
#define USE_FLASH
#define USE_FLASH_W25Q128FV
#define USE_MAX7456
#define USE_SPI_GYRO
#define USE_BARO

#define USE_VCP
#define USE_FLASHFS
#define USE_FLASH_M25P16    // 16MB Micron M25P16 and others (ref: https://github.com/betaflight/betaflight/blob/master/src/main/drivers/flash_m25p16.c)
#define USE_OSD

#define USE_LED
#define LED0_PIN             PB10
#define LED_STRIP_PIN        PA15
#define USE_BEEPER
#define BEEPER_PIN           PB2
#define BEEPER_INVERTED
#define BEEPER_PWM_HZ        1100
#define CAMERA_CONTROL_PIN   PA8

#define USE_SPI
#define USE_SPI_DEVICE_1
#define SPI1_SCK_PIN         PA5
#define SPI1_MISO_PIN        PA6
#define SPI1_MOSI_PIN        PA7
#define USE_SPI_DEVICE_2
#define SPI2_SCK_PIN         PB13
#define SPI2_MISO_PIN        PB14
#define SPI2_MOSI_PIN        PB15
#define USE_SPI_DEVICE_3
#define SPI3_SCK_PIN         PB3
#define SPI3_MISO_PIN        PB4
#define SPI3_MOSI_PIN        PB5

#define USE_EXTI // notice - REQUIRED when USE_GYRO_EXTI
#define USE_GYRO_EXTI

#define USE_MPU_DATA_READY_SIGNAL

#define GYRO_1_ALIGN             CW270_DEG
#define ACC_1_ALIGN              CW270_DEG
#define GYRO_1_CS_PIN            PC4
#define GYRO_1_EXTI_PIN          PA4
#define GYRO_1_SPI_BUS      SPIDEV_1
#define MPU_INT_EXTI             PA4

#define ACC_MPU6000_ALIGN        CW270_DEG
#define GYRO_MPU6000_ALIGN       CW270_DEG
#define MPU6000_CS_PIN           PC4
#define MPU6000_SPI_BUS     SPIDEV_1

#define ACC_ICM42688P_ALIGN      CW270_DEG
#define GYRO_ICM42688P_ALIGN     CW270_DEG
#define ICM42688P_CS_PIN         PC4
#define ICM42688P_SPI_BUS   SPIDEV_1

#define USE_UART1
#define UART1_TX_PIN         PA9
#define UART1_RX_PIN         PA10
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
#define SERIAL_PORT_COUNT 6
//#define USE_SOFTSERIAL1   //old defines had softserial, maybe in error
//#define USE_SOFTSERIAL2
//#define SERIAL_PORT_COUNT 8

#define USE_I2C
#define USE_I2C_DEVICE_1
#define I2C_DEVICE        (I2CDEV_1)
#define BARO_I2C_INSTANCE (I2CDEV_1)
#define I2C1_SCL          PB8
#define I2C1_SDA          PB9

#define FLASH_CS_PIN         PB12
#define FLASH_SPI_INSTANCE   SPI2
#define ENABLE_BLACKBOX_LOGGING_ON_SPIFLASH_BY_DEFAULT

#define MAX7456_SPI_CS_PIN   PB11
#define MAX7456_SPI_INSTANCE SPI2

#define USE_ADC
#define VBAT_ADC_PIN                 PC0
#define CURRENT_METER_ADC_PIN        PC1
#define ADC1_DMA_OPT                 1
#define ADC1_DMA_STREAM              DMA2_Stream4 //# ADC 1: DMA2 Stream 4 Channel 0
#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define DEFAULT_CURRENT_METER_SOURCE CURRENT_METER_ADC
#define DEFAULT_CURRENT_METER_SCALE  179

#define ENABLE_DSHOT_DMAR true

#define DEFAULT_FEATURES       (FEATURE_OSD | FEATURE_TELEMETRY | FEATURE_AIRMODE | FEATURE_RX_SERIAL)
#define DEFAULT_RX_FEATURE     FEATURE_RX_SERIAL

#define USABLE_TIMER_CHANNEL_COUNT 10
#define USE_TIMER_MGMT
#define TIMER_PIN_MAPPING \
    TIMER_PIN_MAP( 0, PC8 , 2,  1) \
    TIMER_PIN_MAP( 1, PC9 , 2,  0) \
    TIMER_PIN_MAP( 2, PB1 , 2,  0) \
    TIMER_PIN_MAP( 3, PB0 , 2,  0) \
    TIMER_PIN_MAP( 4, PC7 , 2,  1) \
    TIMER_PIN_MAP( 5, PC6 , 2,  1) \
    TIMER_PIN_MAP( 6, PB6 , 1,  0) \
    TIMER_PIN_MAP( 7, PB7 , 1,  0) \
    TIMER_PIN_MAP( 8, PA15 , 1,  0) \
    TIMER_PIN_MAP( 9, PA8 , 1,  0)

#define MOTOR1_PIN              PC8
#define MOTOR2_PIN              PC9
#define MOTOR3_PIN              PB1
#define MOTOR4_PIN              PB0
#define MOTOR5_PIN              PC7
#define MOTOR6_PIN              PC6
#define MOTOR7_PIN              PB6
#define MOTOR8_PIN              PB7

// notice - this file was programmatically generated and may be incomplete.
