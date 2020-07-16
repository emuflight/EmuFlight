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

#define USE_TARGET_CONFIG
#define TARGET_MANUFACTURER_IDENTIFIER      "TTRH"
#define TARGET_BOARD_IDENTIFIER             "TT41"
#if defined (TRANSTECF411HD)
#define USBD_PRODUCT_STRING                 "TransTECF411HD"
#else
#define USBD_PRODUCT_STRING                 "TransTECF411"

// *************** OSD *****************************
#define USE_SPI_DEVICE_2
#define SPI2_SCK_PIN                        PB13
#define SPI2_MISO_PIN                       PB14
#define SPI2_MOSI_PIN                       PB15

#define USE_MAX7456
#define MAX7456_SPI_INSTANCE                SPI2
#define MAX7456_SPI_CS_PIN                  PB12
#endif

#define LED0_PIN                            PA14

#define USE_BEEPER
#define BEEPER_PIN                          PB5
#define BEEPER_INVERTED

#define ENABLE_DSHOT_DMAR                                true

#define USE_PINIO
#define PINIO1_PIN                          PB6         //VTX Power Switch
#define USE_PINIOBOX

// *************** Gyro & ACC **********************
#define USE_SPI
#define USE_SPI_DEVICE_1

#define SPI1_SCK_PIN                        PA5
#define SPI1_MISO_PIN                       PA6
#define SPI1_MOSI_PIN                       PA7

#define MPU6000_CS_PIN                      PA4
#define MPU6000_SPI_INSTANCE                SPI1

#define USE_EXTI
#define MPU_INT_EXTI                        PA1
#define USE_MPU_DATA_READY_SIGNAL

#define USE_GYRO
#define USE_GYRO_SPI_MPU6000
#define GYRO_MPU6000_ALIGN                  CW90_DEG

#define USE_ACC
#define USE_ACC_SPI_MPU6000

// *************** UART *****************************
#define USE_VCP
#define USB_DETECT_PIN                      PC15
#define USE_USB_DETECT

#define USE_UART1
#define UART1_RX_PIN                        PA10
#define UART1_TX_PIN                        PA9

#define USE_UART2
#define UART2_RX_PIN                        PA3
#define UART2_TX_PIN                        PA2

#define SERIAL_PORT_COUNT                   3

#define DEFAULT_RX_FEATURE                  FEATURE_RX_SERIAL
#define SERIALRX_PROVIDER                   SERIALRX_SBUS
#define SERIALRX_UART                       SERIAL_PORT_USART1

#define INVERTER_PIN_UART1                  PC13

// *************** ADC *****************************
#define USE_ADC
#define ADC_INSTANCE                        ADC1
#define ADC1_DMA_OPT                        0

#define VBAT_ADC_PIN                        PA0
#define CURRENT_METER_ADC_PIN               PB4

#define USE_ESCSERIAL

#define DEFAULT_FEATURES                    (FEATURE_OSD | FEATURE_AIRMODE)
#define DEFAULT_VOLTAGE_METER_SOURCE        VOLTAGE_METER_ADC
#define DEFAULT_CURRENT_METER_SOURCE        CURRENT_METER_ADC

#define TARGET_IO_PORTA                     0xffff
#define TARGET_IO_PORTB                     0xffff
#define TARGET_IO_PORTC                     0xffff
#define TARGET_IO_PORTD                     (BIT(2))

#define USABLE_TIMER_CHANNEL_COUNT          5
#define USED_TIMERS                         ( TIM_N(1)|TIM_N(2)|TIM_N(3)|TIM_N(4) )
