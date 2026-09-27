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
// Commit: bacae61

#pragma once

#define TARGET_MANUFACTURER_IDENTIFIER "TURC"
#define USBD_PRODUCT_STRING "TUNERCF405"

#define FC_TARGET_MCU     STM32F405     // not used in EmuF
#define TARGET_BOARD_IDENTIFIER "S405"  // generic ID

#define USE_FLASH
#define USE_FLASH_W25Q128FV
#define USE_GYRO
#define USE_ACC
#define USE_ACCGYRO_BMI270
#define USE_MAX7456

#define USE_VCP
#define USE_FLASHFS
#define USE_FLASH_M25P16    // 16MB Micron M25P16 and others (ref: https://github.com/betaflight/betaflight/blob/master/src/main/drivers/flash_m25p16.c)
#define USE_OSD

#define USE_LED
#define LED0_PIN             PB9
#define LED_STRIP_PIN        PB1
#define USE_BEEPER
#define BEEPER_PIN           PB2
#define BEEPER_INVERTED

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

#define GYRO_1_ALIGN         CW180_DEG
#define ACC_1_ALIGN          CW180_DEG
#define GYRO_1_CS_PIN        PA4
#define GYRO_1_EXTI_PIN      PC4
#define GYRO_1_SPI_BUS  SPIDEV_1
#define MPU_INT_EXTI         PC4

#define USE_SPI_GYRO
#define ACC_BMI270_ALIGN     CW180_DEG
#define GYRO_BMI270_ALIGN    CW180_DEG
#define BMI270_CS_PIN        PA4
#define BMI270_SPI_BUS  SPIDEV_1

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
#define INVERTER_PIN_UART4   PC14
#define SERIAL_PORT_COUNT 6

#define USE_I2C
#define USE_I2C_DEVICE_2
#define I2C_DEVICE        (I2CDEV_2)
#define I2C2_SCL PB10
#define I2C2_SDA PB11

#define FLASH_CS_PIN         PB6
#define FLASH_SPI_INSTANCE   SPI3
#define ENABLE_BLACKBOX_LOGGING_ON_SPIFLASH_BY_DEFAULT

#define MAX7456_SPI_CS_PIN   PB12
#define MAX7456_SPI_INSTANCE SPI2

#define USE_ADC
#define VBAT_ADC_PIN          PC0
#define CURRENT_METER_ADC_PIN PC1
#define ADC1_DMA_OPT        0
#define ADC3_DMA_OPT        1
#define ADC1_DMA_STREAM DMA2_Stream0 //# ADC 1: DMA2 Stream 0 Channel 0
#define ADC3_DMA_STREAM DMA2_Stream1 //# ADC 3: DMA2 Stream 1 Channel 2
#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define DEFAULT_CURRENT_METER_SOURCE CURRENT_METER_ADC
#define DEFAULT_CURRENT_METER_SCALE 453

#define ENABLE_DSHOT_DMAR true

#define DEFAULT_FEATURES       (FEATURE_OSD | FEATURE_TELEMETRY | FEATURE_AIRMODE | FEATURE_RX_SERIAL)
#define DEFAULT_RX_FEATURE     FEATURE_RX_SERIAL

#define USABLE_TIMER_CHANNEL_COUNT 9
#define USE_TIMER_MGMT
#define TIMER_PIN_MAPPING \
    TIMER_PIN_MAP( 0, PB0 , 2,  0) \
    TIMER_PIN_MAP( 1, PA8 , 1,  0) \
    TIMER_PIN_MAP( 2, PB8 , 1,  0) \
    TIMER_PIN_MAP( 3, PA15 , 1,  0) \
    TIMER_PIN_MAP( 4, PC9 , 2,  0) \
    TIMER_PIN_MAP( 5, PC8 , 2,  0) \
    TIMER_PIN_MAP( 6, PC7 , 1,  0) \
    TIMER_PIN_MAP( 7, PC6 , 1,  0) \
    TIMER_PIN_MAP( 8, PB1 , 2,  0)

#define MOTOR1_PIN              PB0
#define MOTOR2_PIN              PA8
#define MOTOR3_PIN              PB8
#define MOTOR4_PIN              PA15
#define MOTOR5_PIN              PC9
#define MOTOR6_PIN              PC8
#define MOTOR7_PIN              PC7
#define MOTOR8_PIN              PC6

// notice - this file was programmatically generated and may be incomplete.
