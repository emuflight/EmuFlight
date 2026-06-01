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

#define BOARD_NAME        DAKEFPVH743_SLIM
#define MANUFACTURER_ID   DAKE
#define TARGET_BOARD_IDENTIFIER "SH74"  // generic ID
#define FC_TARGET_MCU     STM32H743     // not used in EmuF

#define USE_ACC
#define USE_GYRO
#define USE_ACC_SPI_MPU6000
#define USE_ACC_SPI_ICM42688P
#define USE_GYRO_SPI_MPU6000
#define USE_GYRO_SPI_ICM42688P
// #define USE_ACCGYRO_LSM6DSV16X // not supported in EmuFlight
// #define USE_ACCGYRO_LSM6DSK320X // not supported in EmuFlight
#define USE_BARO
#define USE_BARO_BMP280
#define USE_BARO_DPS310
#define USE_FLASH
#define USE_FLASH_M25P16
#define USE_MAX7456
#define USE_SDCARD
#define USE_SDCARD_SDIO
// #define USE_GYRO_CLKIN // not supported in EmuFlight

#define USE_VCP
#define USE_FLASHFS
#define USE_FLASH_M25P16    // 16MB Micron M25P16 driver; drives all unless QSPI
#define USE_OSD

#define USE_LED
#define LED0_PIN             PD10
#define LED1_PIN             PD11
#define LED2_PIN             PA8
#define LED_STRIP_PIN        PB0  // TIM3 CH3
#define USE_BEEPER
#define BEEPER_PIN           PE10 
#define BEEPER_INVERTED
#define CAMERA_CONTROL_PIN   PC8  // TIM8 CH3

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
#define SPI3_SCK_PIN         PC10
#define SPI3_MISO_PIN        PC11
#define SPI3_MOSI_PIN        PC12
#define USE_SPI_DEVICE_4
#define SPI4_SCK_PIN         PE12
#define SPI4_MISO_PIN        PE5
#define SPI4_MOSI_PIN        PE6

#define USE_SPI_GYRO
#define USE_EXTI
#define USE_GYRO_EXTI

#define USE_MPU_DATA_READY_SIGNAL

#define ACC_1_ALIGN          CW0_DEG_FLIP
#define GYRO_1_ALIGN         CW0_DEG_FLIP
#define GYRO_1_CS_PIN        PC9
#define GYRO_1_EXTI_PIN      PD4 
#define GYRO_1_SPI_BUS SPIDEV_1

#define USE_DUAL_GYRO

#define GYRO_CONFIG_USE_GYRO_DEFAULT GYRO_CONFIG_USE_GYRO_BOTH
#define ACC_2_ALIGN          CW90_DEG_FLIP
#define GYRO_2_ALIGN         CW90_DEG_FLIP
#define GYRO_2_CS_PIN        PB1
#define GYRO_2_EXTI_PIN      PB2
#define GYRO_2_SPI_BUS SPIDEV_4

#define USE_UART1
#define UART1_TX_PIN         PA9
#define UART1_RX_PIN         PA10
#define USE_UART2
#define UART2_TX_PIN         PD5
#define UART2_RX_PIN         PD6
#define USE_UART3
#define UART3_TX_PIN         PD8
#define UART3_RX_PIN         PD9
#define USE_UART4
#define UART4_TX_PIN         PB9
#define UART4_RX_PIN         PB8
#define USE_UART5
#define UART5_TX_PIN         PB6
#define UART5_RX_PIN         PB5
#define USE_UART6
#define UART6_TX_PIN         PC6
#define UART6_RX_PIN         PC7
#define USE_UART7
#define UART7_TX_PIN         PE8
#define UART7_RX_PIN         PE7
#define USE_UART8
#define UART8_TX_PIN         PE1
#define UART8_RX_PIN         PE0
#define ESC_SENSOR_UART         SERIAL_PORT_USART3 // ESC
#define MSP_UART                SERIAL_PORT_USART2 // BLUETOOTH
#define SERIAL_PORT_COUNT 9

#define USE_I2C
#define USE_I2C_DEVICE_2
#define I2C_DEVICE_2      (I2CDEV_2)
#define BARO_I2C_INSTANCE I2CDEV_2
#define MAG_I2C_INSTANCE I2CDEV_2
#define I2C2_SCL PB10
#define I2C2_SDA PB11

#define FLASH_CS_PIN         PA15
#define FLASH_SPI_INSTANCE SPI3

//notice - NEED: #define SDCARD_DMA_CHANNEL          X            // please verify
//notice - NEED: #define SDCARD_DMA_CHANNEL_TX       DMAx_StreamX // please verify
//notice - other sdcard defines maybe needed (rare?): SDCARD_DMA_STREAM_TX_FULL, SDCARD_DMA_STREAM_TX, SDCARD_DMA_CLK, SDCARD_DMA_CHANNEL_TX_COMPLETE_FLAG
#define ENABLE_BLACKBOX_LOGGING_ON_SDCARD_BY_DEFAULT
#define SDCARD_SPI_FULL_SPEED_CLOCK_DIVIDER     4    //notice - needs validation. these are hardware dependent. known options: 2, 4, 8.
#define SDCARD_SPI_INITIALIZATION_CLOCK_DIVIDER 256  //notice - needs validation. these are hardware dependent. known options: 128, 256

#define MAX7456_SPI_CS_PIN   PB12
#define MAX7456_SPI_INSTANCE SPI2

#define USE_ADC
#define VBAT_ADC_PIN PA4
#define CURRENT_METER_ADC_PIN PC0
#define RSSI_ADC_PIN PC5
#define ADC1_DMA_OPT        9
#define ADC3_DMA_OPT        10
#define ADC1_DMA_STREAM DMA2_Stream1 // ADC1 opt9
#define ADC3_DMA_STREAM DMA2_Stream2 // ADC3 opt10
#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define DEFAULT_VOLTAGE_METER_SCALE 160
#define DEFAULT_CURRENT_METER_SOURCE CURRENT_METER_ADC

#define PINIO1_PIN           PE2
#define PINIO2_PIN           PE3
#define PINIO3_PIN           PE4
#define PINIO4_PIN           PD2
#define PINIO1_CONFIG 129
#define PINIO1_BOX 40
#define PINIO2_CONFIG 129
#define PINIO2_BOX 41
#define PINIO3_CONFIG 129
#define PINIO3_BOX 42
#define PINIO4_CONFIG 129
#define PINIO4_BOX 0

#define TARGET_IO_PORTA 0xffff
#define TARGET_IO_PORTB 0xffff
#define TARGET_IO_PORTC 0xffff
#define TARGET_IO_PORTD 0xffff
#define TARGET_IO_PORTE 0xffff
// notice - port masks derived from config.h; single-pin ports use exact mask, multi-pin ports use 0xffff

#define DEFAULT_FEATURES       (FEATURE_OSD | FEATURE_TELEMETRY | FEATURE_AIRMODE | FEATURE_RX_SERIAL)
#define DEFAULT_RX_FEATURE     FEATURE_RX_SERIAL

#define USABLE_TIMER_CHANNEL_COUNT 14
#define USED_TIMERS ( TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(4) | TIM_N(8) )

// notice - this file was programmatically generated and may be incomplete.
