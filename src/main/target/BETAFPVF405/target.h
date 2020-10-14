/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#define TARGET_BOARD_IDENTIFIER "BEFH"
#define USBD_PRODUCT_STRING     "BetaFPV f405"

#define USE_BEEPER
#define BEEPER_PIN              PB4
#define BEEPER_INVERTED

#define USE_SONAR
#define SONAR_TRIGGER_PIN       PC9
#define SONAR_ECHO_PIN          PA8

#define USE_VCP
#define USE_USB_DETECT
#define USB_DETECT_PIN          PC5

#define USE_UART1
#define UART1_RX_PIN            PA10
#define UART1_TX_PIN            PA9

#define USE_UART3
#define UART3_RX_PIN            PB11
#define UART3_TX_PIN            PB10

#define USE_UART4
#define UART4_RX_PIN            PA1
#define UART4_TX_PIN            PA0

#define USE_UART5
#define UART5_RX_PIN            PD2

#define USE_UART6
#define UART6_RX_PIN            PC7
#define UART6_TX_PIN            PC6

#define SERIAL_PORT_COUNT       6 //VCP, USART1, USART2, USART3, USART6, SOFTSERIAL1, SOFTSERIAL2

#define INVERTER_PIN_UART3      PC9
#define INVERTER_PIN_UART6      PC8

#define LED0_PIN                PB5

#define USE_SPI
#define USE_SPI_DEVICE_1
#define SPI1_SCK_PIN            PA5
#define SPI1_MISO_PIN           PA6
#define SPI1_MOSI_PIN           PA7

#define USE_SPI_DEVICE_2
#define SPI2_SCK_PIN            PB13
#define SPI2_MISO_PIN           PB14
#define SPI2_MOSI_PIN           PB15

#define USE_SPI_DEVICE_3
#define SPI3_SCK_PIN            PC10
#define SPI3_MISO_PIN           PC11
#define SPI3_MOSI_PIN           PC12

#define USE_ESCSERIAL
#define ESCSERIAL_TIMER_TX_PIN  PB8 // (Hardware=0, PPM)

#define USE_ADC
#define ADC_INSTANCE            ADC2 //test 1 for ADC1
#define ADC2_DMA_STREAM         DMA2_Stream3
#define RSSI_ADC_PIN            PC0
#define CURRENT_METER_ADC_PIN   PC1
#define VBAT_ADC_PIN            PC2

#define BARO_CS_PIN             PB3

#define USE_MAX7456
#define MAX7456_SPI_INSTANCE    SPI3
#define MAX7456_SPI_CS_PIN      PA15
#define MAX7456_SPI_CLK         (SPI_CLOCK_STANDARD) // 10MHz
#define MAX7456_RESTORE_CLK     (SPI_CLOCK_FAST)

#define USE_GYRO
#define USE_ACC
#define USE_ACC_SPI_MPU6000
#define GYRO_MPU6000_ALIGN      CW270_DEG
#define USE_GYRO_SPI_MPU6000
#define ACC_MPU6000_ALIGN       CW270_DEG

// MPU6000 interrupts
#define USE_EXTI
#define MPU_INT_EXTI            PC4
#define USE_MPU_DATA_READY_SIGNAL
#define ENSURE_MPU_DATA_READY_IS_LOW

#define MPU6000_CS_PIN          PA4
#define MPU6000_SPI_INSTANCE    SPI1

#define USE_BARO
#define USE_BARO_BMP280
#define USE_BARO_MS5611

#define USE_MAG
#define USE_MAG_HMC5883
#define USE_MAG_QMC5883

#define USE_SDCARD
#define SDCARD_DETECT_PIN             PB7
#define SDCARD_DETECT_INVERTED
#define SDCARD_SPI_INSTANCE           SPI2
#define SDCARD_SPI_CS_PIN             PB12
#define SDCARD_SPI_INITIALIZATION_CLOCK_DIVIDER  256 // 328kHz
#define SDCARD_SPI_FULL_SPEED_CLOCK_DIVIDER  4 // 21MHz
#define SDCARD_DMA_CHANNEL_TX                DMA1_Stream4
#define SDCARD_DMA_CHANNEL                   0

#define FLASH_CS_PIN            PB12
#define FLASH_SPI_INSTANCE      SPI2
#define USE_FLASHFS
#define USE_FLASH_M25P16
#define ENABLE_BLACKBOX_LOGGING_ON_SPIFLASH_BY_DEFAULT

#define DEFAULT_RX_FEATURE      FEATURE_RX_SERIAL
#define SERIALRX_PROVIDER       SERIALRX_SBUS
#define SERIALRX_UART           SERIAL_PORT_USART6
//#define SBUS_TELEMETRY_UART     SERIAL_PORT_USART2

#define DEFAULT_FEATURES        ( FEATURE_TELEMETRY | FEATURE_OSD )

#define TARGET_IO_PORTA 0xffff
#define TARGET_IO_PORTB 0xffff
#define TARGET_IO_PORTC 0xffff
#define TARGET_IO_PORTD 0xffff

#define USABLE_TIMER_CHANNEL_COUNT 10

#define USED_TIMERS             ( TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(4) | TIM_N(8) | TIM_N(10) )
