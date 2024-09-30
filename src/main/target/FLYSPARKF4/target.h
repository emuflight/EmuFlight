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
// Commit: 3f33ae6 + 1 file changed, 1 insertion(+), 25 deletions(-)

#pragma once

#define BOARD_NAME        FLYSPARKF4
#define MANUFACTURER_ID   FLYS
#define TARGET_BOARD_IDENTIFIER "S405"  // generic ID
#define FC_TARGET_MCU     STM32F405     // not used in EmuF

#define USE_ACC
#define USE_GYRO
#define USE_GYRO_SPI_ICM42688P
#define USE_ACC_SPI_ICM42688P
#define USE_SDCARD
#define USE_MAX7456
#define USE_BARO
#define USE_BARO_DPS310
#define USE_SDCARD_SPI

#define USE_VCP
#define USE_OSD

#define USE_LED
#define LED0_PIN PC13
#define LED_STRIP_PIN PA8
#define USE_BEEPER
#define BEEPER_PIN PC15
#define BEEPER_INVERTED
#define CAMERA_CONTROL_PIN PB14

#define USE_SPI
#define USE_SPI_DEVICE_1
#define SPI1_SCK_PIN         PA5
#define SPI1_MISO_PIN        PA6
#define SPI1_MOSI_PIN        PA7
#define USE_SPI_DEVICE_2
#define SPI2_SCK_PIN         PB13
#define SPI2_MISO_PIN        PC2
#define SPI2_MOSI_PIN        PC3
#define USE_SPI_DEVICE_3
#define SPI3_SCK_PIN         PB3
#define SPI3_MISO_PIN        PB4
#define SPI3_MOSI_PIN        PB5

#define USE_SPI_GYRO
#define USE_EXTI
#define USE_GYRO_EXTI

#define MPU_INT_EXTI         PC4

#define ACC_ICM42688P_ALIGN      CW90_DEG
#define GYRO_ICM42688P_ALIGN     CW90_DEG
#define ICM42688P_CS_PIN         PA4
#define ICM42688P_SPI_INSTANCE   SPI1

#define USE_UART1
#define UART1_TX_PIN PA9
#define UART1_RX_PIN PA10
#define USE_UART2
#define UART2_TX_PIN PA2
#define UART2_RX_PIN PA3
#define USE_UART3
#define UART3_TX_PIN PC10
#define UART3_RX_PIN PC11
#define USE_UART4
#define UART4_TX_PIN PA0
#define UART4_RX_PIN PA1
#define USE_UART5
#define UART5_RX_PIN PD2
#define USE_UART6
#define UART6_TX_PIN PC6
#define UART6_RX_PIN PC7
#define SERIALRX_UART SERIAL_PORT_USART2
#define SERIAL_PORT_COUNT 7

#define USE_I2C
#define USE_I2C_DEVICE_1
#define I2C_DEVICE_1      (I2CDEV_1)
#define MAG_I2C_INSTANCE (I2CDEV_1)
#define BARO_I2C_INSTANCE (I2CDEV_1)
#define I2C1_SCL PB8
#define I2C1_SDA PB9

#define USE_SDCARD_SDIO
#define SDCARD_SPI_CS_PIN PC14
#define SDCARD_SPI_INSTANCE SPI3
#define SDCARD_DMA_CHANNEL          0
#define SDCARD_DMA_CHANNEL_TX       DMA1_Stream5
//notice - other sdcard defines maybe needed (rare?): SDCARD_DMA_STREAM_TX_FULL, SDCARD_DMA_STREAM_TX, SDCARD_DMA_CLK, SDCARD_DMA_CHANNEL_TX_COMPLETE_FLAG
#define ENABLE_BLACKBOX_LOGGING_ON_SDCARD_BY_DEFAULT
#define SDCARD_SPI_FULL_SPEED_CLOCK_DIVIDER     4    //notice - needs validation. these are hardware dependent. known options: 2, 4, 8.
#define SDCARD_SPI_INITIALIZATION_CLOCK_DIVIDER 256  //notice - needs validation. these are hardware dependent. known options: 128, 256
#define SDCARD_DETECT_INVERTED

#define MAX7456_SPI_CS_PIN PB12
#define MAX7456_SPI_INSTANCE SPI2

#define USE_ADC
#define VBAT_ADC_PIN PC0
#define CURRENT_METER_ADC_PIN PC1
#define RSSI_ADC_PIN PC5
#define ADC1_DMA_OPT    1
#define ADC1_DMA_STREAM DMA2_Stream0 //# ADC 1: DMA2 Stream 0 Channel 0
#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define DEFAULT_CURRENT_METER_SOURCE CURRENT_METER_ADC
#define DEFAULT_CURRENT_METER_SCALE 400

#define ENABLE_DSHOT_DMAR false
#define ESC_SENSOR_UART SERIAL_PORT_UART5

#define PINIO1_PIN PB11
#define PINIO1_CONFIG 129
#define PINIO1_BOX 40

#define TARGET_IO_PORTA 0xffff
#define TARGET_IO_PORTB 0xffff
#define TARGET_IO_PORTC 0xffff
#define TARGET_IO_PORTD 0xffff
// notice - masks were programmatically generated - please verify last port group for 0xffff or (BIT(2))

#define DEFAULT_FEATURES       (FEATURE_OSD | FEATURE_TELEMETRY | FEATURE_AIRMODE | FEATURE_RX_SERIAL)
#define DEFAULT_RX_FEATURE     FEATURE_RX_SERIAL

#define USABLE_TIMER_CHANNEL_COUNT 12
#define USED_TIMERS ( TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(4) | TIM_N(8) | TIM_N(9) | TIM_N(12) )

// notice - this file was programmatically generated and may be incomplete.
