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

#define TARGET_BOARD_IDENTIFIER "S405"
#define USBD_PRODUCT_STRING "NBD INFINITYF4"

#define USE_TARGET_CONFIG

#define LED0_PIN                  PC0

#define USE_BEEPER
#define BEEPER_PIN                PB6
#define BEEPER_INVERTED

#define USE_PINIO
#define PINIO1_PIN                PC5
#define USE_PINIOBOX

#define ENABLE_DSHOT_DMAR        true

#define CAMERA_CONTROL_PIN        PB9
#define INVERTER_PIN_UART1        PA10


#define USE_EXTI
#define MPU_INT_EXTI              PC13
#define USE_MPU_DATA_READY_SIGNAL

//  MPU 6000

#define MPU6000_CS_PIN            PB12
#define MPU6000_SPI_INSTANCE      SPI2
#define USE_ACC
#define USE_ACC_SPI_MPU6000
#define USE_GYRO
#define USE_GYRO_SPI_MPU6000
#define GYRO_MPU6000_ALIGN        CW90_DEG
#define ACC_MPU6000_ALIGN         CW90_DEG

#define USE_MAX7456
#define MAX7456_SPI_INSTANCE      SPI1
#define MAX7456_SPI_CS_PIN        PA4
#define MAX7456_SPI_CLK           (SPI_CLOCK_STANDARD)
#define MAX7456_RESTORE_CLK       (SPI_CLOCK_FAST)

#define ENABLE_BLACKBOX_LOGGING_ON_SPIFLASH_BY_DEFAULT
#define USE_FLASHFS
#define USE_FLASH_M25P16
#define FLASH_CS_PIN              PA15
#define FLASH_SPI_INSTANCE        SPI3

#define USE_VCP
//#define USE_USB_DETECT
//#define USB_DETECT_PIN            PC5

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
#define UART4_RX_PIN              PA1
#define UART4_TX_PIN              PA0

#define USE_UART5
#define UART5_RX_PIN              PD2
#define UART5_TX_PIN              PC12

#define USE_UART6
#define UART6_RX_PIN              PC7
#define UART6_TX_PIN              PC6

#define USE_SOFTSERIAL1

#define SERIAL_PORT_COUNT        8 //VCP, USART1, USART3,USART4, USART6, SOFT_SERIAL1

#define USE_ESCSERIAL
#define ESCSERIAL_TIMER_TX_PIN    PD2

#define USE_SPI
#define USE_SPI_DEVICE_1
//#define SPI1_NSS_PIN              PA4
#define SPI1_SCK_PIN              PA5
#define SPI1_MISO_PIN             PA6
#define SPI1_MOSI_PIN             PA7

#define USE_SPI_DEVICE_2
//#define SPI2_NSS_PIN              PB12
#define SPI2_SCK_PIN              PB13
#define SPI2_MISO_PIN             PB14
#define SPI2_MOSI_PIN             PB15

#define USE_SPI_DEVICE_3
//#define SPI3_NSS_PIN              PA15
#define SPI3_SCK_PIN              PB3
#define SPI3_MISO_PIN             PB4
#define SPI3_MOSI_PIN             PB5

#define USE_ADC
#define CURRENT_METER_ADC_PIN     PC3
#define VBAT_ADC_PIN              PC4
// #define RSSI_ADC_PIN              PC5  --undefined in resource list
#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define DEFAULT_CURRENT_METER_SOURCE CURRENT_METER_ADC

#define DEFAULT_RX_FEATURE        FEATURE_RX_SERIAL
#define DEFAULT_FEATURES          ( FEATURE_OSD  )
#define CURRENT_METER_SCALE_DEFAULT 250

#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         0xffff
#define TARGET_IO_PORTD         (BIT(2))

#define USABLE_TIMER_CHANNEL_COUNT 7
#define USED_TIMERS  (  TIM_N(1) | TIM_N(3) | TIM_N(4) | TIM_N(8) | TIM_N(11))
