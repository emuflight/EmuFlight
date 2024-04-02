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

#define TARGET_BOARD_IDENTIFIER "TEBS"
#define USBD_PRODUCT_STRING     "PODRACERAIO"

// ******* LEDs and BEEPER ********

#define LED0_PIN                PC13


#define USE_BEEPER
#define BEEPER_PIN              PB2
#define BEEPER_INVERTED

#define ENABLE_DSHOT_DMAR       true

// ******* INVERTER PIN ********

//#define INVERTER_PIN_UART1      PB10

// ******* GYRO and ACC ********

#define USE_EXTI
#define MPU_INT_EXTI            PC15
#define USE_MPU_DATA_READY_SIGNAL

#define USE_GYRO

#define USE_ACC

#define MPU6000_CS_PIN           PA15
#define MPU6000_SPI_INSTANCE     SPI3

#define USE_GYRO_SPI_MPU6000
#define GYRO_MPU6000_ALIGN      CW90_DEG

#define USE_ACC_SPI_MPU6000
#define ACC_MPU6000_ALIGN       CW90_DEG

// *************** Baro **************************
#define USE_I2C

#define USE_I2C_DEVICE_1
#define I2C_DEVICE              (I2CDEV_1)
#define I2C1_SCL                PB8        // SCL pad
#define I2C1_SDA                PB9        // SDA pad
#define BARO_I2C_INSTANCE       (I2CDEV_1)

#define USE_BARO
#define USE_BARO_BMP280
#define USE_BARO_MS5611
#define USE_BARO_BMP085

// ******* OSD ********

#define USE_MAX7456
#define MAX7456_SPI_INSTANCE    SPI1
#define MAX7456_SPI_CS_PIN      PB10

// ******* SERIAL ********

#define USE_VCP

#define USE_UART1
#define UART1_TX_PIN            PA9
#define UART1_RX_PIN            PA10

#define USE_UART2
#define UART2_TX_PIN            PA2
#define UART2_RX_PIN            PA3

#define USE_SOFTSERIAL1
#define SOFTSERIAL1_RX_PIN      PA9
#define SOFTSERIAL1_TX_PIN      PA8
//#define USE_SOFTSERIAL2

#define SERIAL_PORT_COUNT       4 //VCP, USART1, USART2, SOFTSERIAL

#define USE_ESCSERIAL
//#define ESCSERIAL_TIMER_TX_PIN  PB9  // (HARDARE=0,PPM)

// ******* SPI ********

#define USE_SPI

#define USE_SPI_DEVICE_1
//#define SPI1_NSS_PIN            PB10
#define SPI1_SCK_PIN            PA5
#define SPI1_MISO_PIN           PA6
#define SPI1_MOSI_PIN           PA7

#define USE_SPI_DEVICE_3
//#define SPI3_NSS_PIN            PA15
#define SPI3_SCK_PIN            PB3
#define SPI3_MISO_PIN           PB4
#define SPI3_MOSI_PIN           PB5

// ******* ADC ********

#define USE_ADC
#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define DEFAULT_CURRENT_METER_SOURCE    CURRENT_METER_ADC
#define ADC_INSTANCE            ADC1  // Default added
#define ADC1_DMA_STREAM         DMA2_Stream0
#define VBAT_ADC_PIN            PB1
//#define RSSI_ADC_PIN            PB1
#define CURRENT_METER_ADC_PIN   PA4


// ******* FEATURES ********

#define DEFAULT_RX_FEATURE      FEATURE_RX_SERIAL
#define SERIALRX_UART           SERIAL_PORT_USART2
#define SERIALRX_PROVIDER       SERIALRX_CRSF

#define DEFAULT_FEATURES                (FEATURE_OSD | FEATURE_TELEMETRY | FEATURE_SOFTSERIAL)
#define CURRENT_METER_SCALE_DEFAULT                      250                    // 3.3/120A  = 25mv/A

#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         0xffff
#define TARGET_IO_PORTD         (BIT(2))

#define USABLE_TIMER_CHANNEL_COUNT 8
#define USED_TIMERS             ( TIM_N(1) | TIM_N(2) | TIM_N(4) | TIM_N(9) )
