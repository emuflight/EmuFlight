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

#define TARGET_BOARD_IDENTIFIER "TTRH"
#define USBD_PRODUCT_STRING     "Transtec f405 HD"

#define USE_BEEPER
#define BEEPER_PIN              PB4
#define BEEPER_INVERTED

#define USE_VCP
#define USE_USB_DETECT
#define USB_DETECT_PIN          PB12

#define USE_UART1
#define UART1_RX_PIN            PA10
#define UART1_TX_PIN            PA9

#define USE_UART2
#define UART2_RX_PIN            PA3
#define UART2_TX_PIN            PA2

#define USE_UART3
#define UART3_RX_PIN            PB11
#define UART3_TX_PIN            PB10

#define USE_UART4
#define UART4_RX_PIN            PA1
#define UART4_TX_PIN            PA0

#define USE_UART5
#define UART5_RX_PIN            PD2
#define UART5_TX_PIN            PC12

#define USE_SOFTSERIAL1
#define USE_SOFTSERIAL2

#define SERIAL_PORT_COUNT       8 //VCP, USART1, USART2, USART3, USART6, SOFTSERIAL1, SOFTSERIAL2

#define INVERTER_PIN_UART1      PB8

#define LED0_PIN                PB9

#define USE_SPI
#define USE_SPI_DEVICE_1
#define SPI1_SCK_PIN            PA5
#define SPI1_MISO_PIN           PA6
#define SPI1_MOSI_PIN           PA7

#define USE_ESCSERIAL
#define USE_ADC
#define ADC_INSTANCE            ADC1 //test 1 for ADC1
#define ADC1_DMA_STREAM         DMA2_Stream0
#define RSSI_ADC_PIN            PB1
#define CURRENT_METER_ADC_PIN   PC4
#define VBAT_ADC_PIN            PC5

#define USE_GYRO
#define USE_ACC
#define USE_ACC_SPI_MPU6000
#define GYRO_MPU6000_ALIGN      CW0_DEG
#define USE_GYRO_SPI_MPU6000
#define ACC_MPU6000_ALIGN       CW0_DEG

// MPU6000 interrupts
#define USE_EXTI
#define MPU_INT_EXTI            PC3
#define USE_MPU_DATA_READY_SIGNAL
#define ENSURE_MPU_DATA_READY_IS_LOW

#define MPU6000_CS_PIN          PC2
#define MPU6000_SPI_INSTANCE    SPI1

#define DEFAULT_RX_FEATURE      FEATURE_RX_SERIAL
#define SERIALRX_PROVIDER       SERIALRX_SBUS
#define SERIALRX_UART           SERIAL_PORT_USART6
//#define SBUS_TELEMETRY_UART     SERIAL_PORT_USART2

#define ENABLE_DSHOT_DMAR       true
#define DEFAULT_FEATURES        ( FEATURE_TELEMETRY | FEATURE_OSD )

#define TARGET_IO_PORTA 0xffff
#define TARGET_IO_PORTB 0xffff
#define TARGET_IO_PORTC 0xffff
#define TARGET_IO_PORTD 0xffff

#define USABLE_TIMER_CHANNEL_COUNT 5
#define USED_TIMERS             ( TIM_N(3) | TIM_N(4) | TIM_N(8) )
