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
#define TARGET_BOARD_IDENTIFIER                          "S411"
#define USBD_PRODUCT_STRING                              "HGLRCF411"
#define TARGET_MANUFACTURER_IDENTIFIER                   "HGLR"

#define ENABLE_DSHOT_DMAR                                true

//Aux
#define LED0_PIN                                         PC13

#define USE_BEEPER
#define BEEPER_PIN                                       PB2
#define BEEPER_INVERTED
//define camera control
#define CAMERA_CONTROL_PIN                               PB10

//MPU-6000
#define USE_GYRO
#define USE_ACC
#define USE_ACC_SPI_MPU6000
#define USE_GYRO_SPI_MPU6000
#define USE_EXTI
#define USE_MPU_DATA_READY_SIGNAL

#define MPU_INT_EXTI                                     PA1                    //MPU_INT_EXTI
#define MPU6000_CS_PIN                                   PA4                    //GYRO_1_CS_PIN
#define MPU6000_SPI_INSTANCE                             SPI1                   //GYRO_1_SPI_INSTANCE
#define GYRO_MPU6000_ALIGN                               CW180_DEG
#define ACC_MPU6000_ALIGN                                CW180_DEG

// OSD
#define USE_MAX7456
#define MAX7456_SPI_INSTANCE                             SPI2
#define MAX7456_SPI_CS_PIN                               PB12
#define MAX7456_SPI_CLK                                  (SPI_CLOCK_STANDARD)   // 10MHz
#define MAX7456_RESTORE_CLK                              (SPI_CLOCK_FAST)

// Uarts
#define USE_UART1
#define UART1_RX_PIN                                     PA10
#define UART1_TX_PIN                                     PA9

#define USE_UART2
#define UART2_RX_PIN                                     PA3
#define UART2_TX_PIN                                     PA2
#define USE_SOFTSERIAL1
#define SERIAL_PORT_COUNT                                4                      //VCP, USART1, USART2,USART3,USART4,USART5,USART6

// ESC
#define USE_ESCSERIAL
#define USE_ADC
#define DEFAULT_VOLTAGE_METER_SOURCE                     VOLTAGE_METER_ADC
#define DEFAULT_CURRENT_METER_SOURCE                     CURRENT_METER_ADC
#define ESCSERIAL_TIMER_TX_PIN                           PB8                    // (Hardware=0, PPM)
#define CURRENT_METER_ADC_PIN                            PB1
#define VBAT_ADC_PIN                                     PB0
#define CURRENT_METER_SCALE_DEFAULT                      250                    // 3.3/120A  = 25mv/A

// SPI devices
#define USE_SPI
#define USE_SPI_DEVICE_1
#define USE_SPI_DEVICE_2
#define SPI1_MISO_PIN                                   PA6
#define SPI1_MOSI_PIN                                   PA7
#define SPI1_NSS_PIN                                    PA4
#define SPI1_SCK_PIN                                    PA5
#define SPI2_MISO_PIN                                   PB14
#define SPI2_MOSI_PIN                                   PB15
//#define SPI2_NSS_PIN                                    PB12
#define SPI2_SCK_PIN                                    PB13

//Flash
#define ENABLE_BLACKBOX_LOGGING_ON_SPIFLASH_BY_DEFAULT
#define USE_FLASHFS
#define USE_FLASH_M25P16
#define FLASH_CS_PIN                                    PA15
#define FLASH_SPI_INSTANCE                              SPI2

// USB
#define USE_VCP
#define USE_USB_DETECT
#define USB_DETECT_PIN                                  PC15
#define DEFAULT_RX_FEATURE                              FEATURE_RX_SERIAL
#define SERIALRX_PROVIDER                               SERIALRX_SBUS
#define SERIALRX_UART                                   SERIAL_PORT_USART2

// IO Ports
#define TARGET_IO_PORTA                                 0xffff
#define TARGET_IO_PORTB                                 0xffff
#define TARGET_IO_PORTC                                 0xffff
#define TARGET_IO_PORTD                                 (BIT(2))

// timers
#define USABLE_TIMER_CHANNEL_COUNT                      9                       //updated timer count to compensate for Nf Motor 4
#define USED_TIMERS ( TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(4)  | TIM_N(5) | TIM_N(9)   ) //update based on update CLRACINGF7 Target BF4.1+
