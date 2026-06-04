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

#define BOARD_NAME                      DRONEERH743DUAL
#define MANUFACTURER_ID                 DRNR
#define TARGET_BOARD_IDENTIFIER "SH74"  // generic ID
#define FC_TARGET_MCU                   STM32H743     // not used in EmuF

#define USE_BARO
#define USE_BARO_DPS310
#define USE_ACC
#define USE_GYRO
#define USE_ACC_SPI_ICM42688P
#define USE_GYRO_SPI_ICM42688P
// #define USE_GYRO_CLKIN // not supported in EmuFlight
#define USE_FLASH
#define USE_FLASH_W25Q128FV
#define USE_MAX7456
#define USE_SWDIO_PIN                   PA13
#define USE_SWCLK_PIN                   PA14

#define USE_VCP
#define USE_FLASHFS
#define USE_FLASH_M25P16    // 16MB Micron M25P16 driver; drives all unless QSPI
#define USE_OSD

#define USE_LED
#define LED0_PIN                        PC4  //ADC12_INP4
#define LED1_PIN                        PC5  //ADC12_INN4/ADC12_INP8
#define LED_STRIP_PIN                   PA0  //TIM2_CH1/TIM5_CH1/UART4_TX/ADC1_INP16
#define USE_BEEPER
#define BEEPER_PIN                      PD14 //TIM4_CH3
#define BEEPER_INVERTED
#define CAMERA_CONTROL_PIN              PA1  //TIM2_CH2/TIM5_CH2/TIM15_CH1N/UART4_RX/ADC12_INP1
#define USE_USB_DETECT
#define USB_DETECT_PIN                  PA10 //TIM1_CH3/USART1_RX

#define USE_SPI
#define USE_SPI_DEVICE_1
#define SPI1_SCK_PIN                    PB3  //TIM2_CH2/SPI1_SCK/SPI3_SCK/SPI6_SCK/UART7_RX
#define SPI1_MISO_PIN        PB4
#define SPI1_MOSI_PIN        PB5
#define USE_SPI_DEVICE_2
#define SPI2_SCK_PIN                    PB13 //TIM1_CH1N/SPI2_SCK/USART3_NSS/UART5_TX
#define SPI2_MISO_PIN        PB14
#define SPI2_MOSI_PIN        PB15
#define USE_SPI_DEVICE_3
#define SPI3_SCK_PIN                    PC10 //SPI3_SCK/USART3_TX/UART4_TX/SDMMC1_D2
#define SPI3_MISO_PIN        PC11
#define SPI3_MOSI_PIN        PC12
#define USE_SPI_DEVICE_4
#define SPI4_SCK_PIN                    PE2  //SPI4_SCK
#define SPI4_MISO_PIN        PE5
#define SPI4_MOSI_PIN        PE6
#define USE_SPI_DEVICE_6

#define USE_SPI_GYRO
#define USE_EXTI
#define USE_GYRO_EXTI

#define ACC_1_ALIGN          CW0_DEG
#define GYRO_1_ALIGN         CW0_DEG
#define GYRO_1_CS_PIN                   PA15 //TIM2_CH1/SPI1_NSS/SPI3_NSS/SPI6_NSS/UART7_TX
#define GYRO_1_EXTI_PIN                 PD7  //SPI1_MOSI
#define GYRO_1_SPI_BUS             SPIDEV_1

#define USE_DUAL_GYRO

#define ACC_2_ALIGN          CW0_DEG
#define GYRO_2_ALIGN         CW0_DEG
#define GYRO_2_CS_PIN                   PE3  //SPI4_NSS
#define GYRO_2_EXTI_PIN                 PE4  //TIM15_CH1N/SPI4_NSS
#define GYRO_2_SPI_BUS             SPIDEV_4

#define USE_UART1
#define UART1_TX_PIN                    PA9  //TIM1_CH2/I2C3_SDA/SPI2_SCK/USART1_TX
#define UART1_RX_PIN                    PB7  //TIM17_CH1N/TIM4_CH2/I2C1_SDA/I2C4_SDA/USART1_RX
#define USE_UART2
#define UART2_TX_PIN                    PD5  //USART2_TX
#define UART2_RX_PIN                    PD6  //SPI3_MOSI/USART2_RX
#define USE_UART3
#define UART3_TX_PIN                    PD8  //USART3_TX
#define UART3_RX_PIN                    PD9  //USART3_RX
#define USE_UART4
#define UART4_TX_PIN                    PD1  //UART4_TX
#define UART4_RX_PIN                    PD0  //UART4_RX
#define USE_UART5
#define UART5_TX_PIN                    PB6
#define UART5_RX_PIN                    PD2  //UART5_RX/SDMMC1_CMD
#define USE_UART6
#define UART6_TX_PIN                    PC6  //TIM3_CH1/TIM8_CH1/USART6_TX
#define UART6_RX_PIN                    PC7  //TIM3_CH2/TIM8_CH2/USART6_RX
#define PINIO1_PIN                      PA2  //TIM2_CH3/TIM5_CH3/TIM15_CH1/USART2_TX/ADC12_INP14
#define PINIO2_PIN                      PA3  //TIM2_CH4/TIM5_CH4/TIM15_CH2/USART2_RX/ADC12_INP15
#define SERIALRX_UART                   SERIAL_PORT_USART2
#define GPS_UART                        SERIAL_PORT_USART6
#define SPI3_SDI_PIN                    PC11 //SPI3_MISO/USART3_RX/UART4_RX/SDMMC1_D3
#define SPI2_SDI_PIN                    PB14 //TIM1_CH2N/TIM12_CH1/TIM8_CH2N/USART1_TX/SPI2_MISO
#define SPI2_SDO_PIN                    PB15 //TIM1_CH3N/TIM12_CH2/TIM8_CH3N/USART1_RX/SPI2_MOSI
#define SERIAL_PORT_COUNT 7

#define USE_I2C
#define USE_I2C_DEVICE_1
#define I2C_DEVICE_1      (I2CDEV_1)
#define BARO_I2C_INSTANCE               I2CDEV_1
#define I2C1_SCL PB8
#define I2C1_SDA PB9
#define USE_I2C_DEVICE_2
#define I2C_DEVICE_2      (I2CDEV_2)
#define USE_I2C_DEVICE_3
#define I2C_DEVICE_3      (I2CDEV_3)
#define USE_I2C_DEVICE_4
#define I2C_DEVICE_4      (I2CDEV_4)
#define MAG_I2C_INSTANCE                I2CDEV_4
#define I2C4_SCL PD12
#define I2C4_SDA PD13

#define FLASH_CS_PIN                    PA8  //TIM1_CH1/I2C3_SCL/UART7_RX
#define FLASH_SPI_INSTANCE              SPI3
#define ENABLE_BLACKBOX_LOGGING_ON_SPIFLASH_BY_DEFAULT

#define MAX7456_SPI_CS_PIN              PB12 //I2C2_SMBA/SPI2_NSS/UART5_RX
#define MAX7456_SPI_INSTANCE            SPI2

#define USE_ADC
#define VBAT_ADC_PIN PC0
#define CURRENT_METER_ADC_PIN PC1
#define ADC1_DMA_OPT                    8
#define ADC3_DMA_OPT                    10
#define ADC1_DMA_STREAM DMA2_Stream0 // ADC1 opt8
#define ADC3_DMA_STREAM DMA2_Stream2 // ADC3 opt10
#define DEFAULT_VOLTAGE_METER_SOURCE    VOLTAGE_METER_ADC
#define DEFAULT_VOLTAGE_METER_SCALE 110
#define DEFAULT_CURRENT_METER_SOURCE    CURRENT_METER_ADC

#define PINIO1_CONFIG                   129
#define PINIO1_BOX                      40
#define PINIO2_CONFIG                   1
#define PINIO2_BOX                      41

#define TARGET_IO_PORTA 0xffff
#define TARGET_IO_PORTB 0xffff
#define TARGET_IO_PORTC 0xffff
#define TARGET_IO_PORTD 0xffff
#define TARGET_IO_PORTE 0xffff
// notice - port masks derived from config.h; single-pin ports use exact mask, multi-pin ports use 0xffff

#define DEFAULT_FEATURES       (FEATURE_OSD | FEATURE_TELEMETRY | FEATURE_AIRMODE | FEATURE_RX_SERIAL)
#define DEFAULT_RX_FEATURE     FEATURE_RX_SERIAL

#define USABLE_TIMER_CHANNEL_COUNT 13
#define USED_TIMERS ( TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(4) | TIM_N(5) | TIM_N(8) | TIM_N(15) )

// notice - this file was programmatically generated and may be incomplete.
