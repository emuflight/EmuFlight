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

#define TARGET_BOARD_IDENTIFIER "NEBD"
#define USBD_PRODUCT_STRING "NBD CRICKETF7"

#define USE_TARGET_CONFIG
#define MSP_OVER_CLI

// *************** SPI *****************************
#define USE_SPI
#define USE_SPI_DEVICE_1
#define SPI1_SCK_PIN              PA5
#define SPI1_MISO_PIN             PA6
#define SPI1_MOSI_PIN             PA7

#define USE_SPI_DEVICE_2
#define SPI2_SCK_PIN              PB13
#define SPI2_MISO_PIN             PB14
#define SPI2_MOSI_PIN             PB15

#define USE_SPI_DEVICE_3
#define SPI3_SCK_PIN              PC10
#define SPI3_MISO_PIN             PC11
#define SPI3_MOSI_PIN             PC12

// *************** Gyro & ACC **********************
#define USE_ACC
#define USE_ACC_SPI_MPU6000
#define USE_GYRO
#define USE_GYRO_SPI_MPU6000

#define MPU6000_CS_PIN            PB12
#define MPU6000_SPI_INSTANCE      SPI2

#define GYRO_MPU6000_ALIGN        CW90_DEG
#define ACC_MPU6000_ALIGN         CW90_DEG

#define USE_EXTI
#define MPU_INT_EXTI              PC4
#define USE_MPU_DATA_READY_SIGNAL

// *************** OSD *****************************
#define USE_MAX7456
#define MAX7456_SPI_INSTANCE      SPI3
#define MAX7456_SPI_CS_PIN        PC13
#define MAX7456_SPI_CLK           (SPI_CLOCK_STANDARD)
#define MAX7456_RESTORE_CLK       (SPI_CLOCK_FAST)


// *************** UART *****************************
#define USE_VCP


#define USE_UART1
#define UART1_RX_PIN              PA10
#define UART1_TX_PIN              PA9

#define USE_UART2
#define UART2_RX_PIN              PA3
#define UART2_TX_PIN              PA2

#define USE_UART3
#define UART3_RX_PIN              PB11
#define UART3_TX_PIN              PB10

#define USE_UART4
#define UART4_RX_PIN              PA0
#define UART4_TX_PIN              PA1

#define USE_UART6
#define UART6_RX_PIN              PC7
#define UART6_TX_PIN              PC6

#define USE_SOFTSERIAL1

#define SERIAL_PORT_COUNT        7 //VCP, USART1, USART3,USART4, USART6, SOFT_SERIAL1
#define USE_MSP_UART

// *************** ADC *****************************
#define USE_ADC
#define ADC_INSTANCE              ADC1    // Default added
#define VBAT_ADC_PIN              PC0
#define ADC1_DMA_STREAM           DMA2_Stream0
#define ADC1_DMA_OPT              0       // DMA 2 Stream 0 Channel 0
#define CURRENT_METER_ADC_PIN     PC1
#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define DEFAULT_CURRENT_METER_SOURCE CURRENT_METER_ADC
#define CURRENT_METER_SCALE_DEFAULT 400
#define VBAT_SCALE 110

// *************** RX ******************************
#define DEFAULT_RX_FEATURE        FEATURE_RX_SERIAL

// *************** OTHERS **************************
#define DEFAULT_FEATURES          ( FEATURE_OSD  )

#define USE_ESCSERIAL
#define ESCSERIAL_TIMER_TX_PIN    NONE

#define LED0_PIN                  PA15

#define USE_BEEPER
#define BEEPER_PIN                PC15
#define BEEPER_INVERTED

#define ENABLE_DSHOT_DMAR        true

//#define ENABLE_BLACKBOX_LOGGING_ON_SPIFLASH_BY_DEFAULT
//#define USE_FLASHFS
//#define USE_FLASH_M25P16
#define FLASH_CS_PIN              PA4
#define FLASH_SPI_INSTANCE        SPI1

#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         0xffff
#define TARGET_IO_PORTD         (BIT(2))

#define USABLE_TIMER_CHANNEL_COUNT 5
#define USED_TIMERS  (  TIM_N(1) | TIM_N(2) | TIM_N(3))
