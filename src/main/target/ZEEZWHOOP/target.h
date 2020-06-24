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

#define TARGET_BOARD_IDENTIFIER "ZEEZ"
#define USBD_PRODUCT_STRING     "ZEEZWHOOP"

#define LED0_PIN                PC13

// *************** Gyro & ACC **********************
#define USE_SPI
#define USE_SPI_DEVICE_1
#define USE_SPI_DEVICE_3

#define SPI1_SCK_PIN            PA5
#define SPI1_MISO_PIN           PA6
#define SPI1_MOSI_PIN           PA7

#define SPI3_SCK_PIN            PB3
#define SPI3_MISO_PIN           PB4
#define SPI3_MOSI_PIN           PB5

#define MPU6000_CS_PIN          PA15
#define MPU6000_SPI_INSTANCE    SPI3

#define USE_EXTI
#define MPU_INT_EXTI            PC15
#define USE_MPU_DATA_READY_SIGNAL

#define USE_GYRO
#define USE_GYRO_SPI_MPU6000
#define GYRO_MPU6000_ALIGN      CW180_DEG
#define USE_ACC
#define USE_ACC_SPI_MPU6000
#define ACC_MPU6000_ALIGN       CW180_DEG

// *************** Baro **************************
#define USE_I2C

#define USE_I2C_DEVICE_1
#define I2C_DEVICE              (I2CDEV_1)
#define I2C1_SCL                PB8        // SCL pad
#define I2C1_SDA                PB9        // SDA pad


// *************** UART *****************************
#define USE_VCP
#define USB_DETECT_PIN          PC15
#define USE_USB_DETECT

#define USE_UART1
#define UART1_RX_PIN            PA10
#define UART1_TX_PIN            PA9

#define USE_UART2
#define UART2_RX_PIN            PA3
#define UART2_TX_PIN            PA2

#define USE_SOFTSERIAL1
#define USE_SOFTSERIAL2

#define SERIAL_PORT_COUNT       5

#define DEFAULT_RX_FEATURE      FEATURE_RX_SERIAL
#define SERIALRX_PROVIDER       SERIALRX_SBUS
#define SERIALRX_UART           SERIAL_PORT_USART1

// *************** OSD *****************************
#define USE_MAX7456
#define MAX7456_SPI_INSTANCE    SPI1
#define MAX7456_SPI_CS_PIN      PB10

// *************** ADC *****************************

#define USE_ADC
#define DEFAULT_VOLTAGE_METER_SOURCE                     VOLTAGE_METER_ADC
#define DEFAULT_CURRENT_METER_SOURCE                     CURRENT_METER_ADC
#define CURRENT_METER_ADC_PIN                            PA4
#define VBAT_ADC_PIN                                     PB1



#define DEFAULT_FEATURES                (FEATURE_OSD | FEATURE_TELEMETRY | FEATURE_SOFTSERIAL)


#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         0xffff
#define TARGET_IO_PORTD         (BIT(2))

#define USABLE_TIMER_CHANNEL_COUNT 6
#define USED_TIMERS             ( TIM_N(1)|TIM_N(2)|TIM_N(4)|TIM_N(9) )
