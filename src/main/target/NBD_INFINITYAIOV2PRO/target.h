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

#define USBD_PRODUCT_STRING "NBD_INFINITYAIOV2PRO"

#define LED0_PIN   PC0

#define USE_BEEPER
#define BEEPER_PIN PD13
#define BEEPER_INVERTED

#define USE_SPI_GYRO
#define USE_ACCGYRO_BMI270
#define BMI270_CS_PIN           PE11
#define BMI270_SPI_INSTANCE     SPI4
#define ACC_BMI270_ALIGN        CW270_DEG
#define GYRO_BMI270_ALIGN       CW270_DEG

// dual gyro
#define USE_DUAL_GYRO

// gyro 1
#define GYRO_1_CS_PIN           PE11
#define GYRO_1_SPI_INSTANCE     SPI4
#define GYRO_1_EXTI_PIN         PB1
#define GYRO_1_ALIGN            CW270_DEG
#define ACC_1_ALIGN             CW270_DEG

// gyro 2
#define GYRO_2_CS_PIN           PB12
#define GYRO_2_SPI_INSTANCE     SPI2
#define GYRO_2_EXTI_PIN         PD0
#define GYRO_2_ALIGN            CW270_DEG
#define ACC_2_ALIGN             CW270_DEG

#define USE_VCP
#define USE_USB_DETECT

#define USE_UART1
#define UART1_RX_PIN PB7

#define USE_UART2
#define UART2_RX_PIN PA3
#define UART2_TX_PIN PA2

#define USE_UART3
#define UART3_RX_PIN PB11
#define UART3_TX_PIN PB10

#define USE_UART5
#define UART5_RX_PIN PD2

#define USE_UART7
#define UART7_TX_PIN PE8

#define USE_UART8
#define UART8_RX_PIN PE0
#define UART8_TX_PIN PE1

#define SERIAL_PORT_COUNT 7 //VCP, USART1, USART2, USART3, UART5, USART7, UART8

#define USE_ESC_SENSOR

#define USE_SPI
#define USE_SPI_DEVICE_1
#define USE_SPI_DEVICE_2
#define USE_SPI_DEVICE_3
#define USE_SPI_DEVICE_4

#define SPI1_SCK_PIN   PA5
#define SPI1_MISO_PIN  PA6
#define SPI1_MOSI_PIN  PA7

#define SPI2_SCK_PIN   PB13
#define SPI2_MISO_PIN  PB14
#define SPI2_MOSI_PIN  PB15

#define SPI3_SCK_PIN   PB3
#define SPI3_MISO_PIN  PB4
#define SPI3_MOSI_PIN  PD6

#define SPI4_SCK_PIN   PE12
#define SPI4_MISO_PIN  PE13
#define SPI4_MOSI_PIN  PE14

#define USE_MAX7456
#define MAX7456_SPI_INSTANCE    SPI3
#define MAX7456_SPI_CS_PIN      PA15
#define MAX7456_SPI_CLK         (SPI_CLOCK_STANDARD) // 10MHz
#define MAX7456_RESTORE_CLK     (SPI_CLOCK_FAST)

//#define ENABLE_BLACKBOX_LOGGING_ON_SDCARD_BY_DEFAULT
#define ENABLE_BLACKBOX_LOGGING_ON_SPIFLASH_BY_DEFAULT
#define USE_FLASHFS
#define USE_FLASH_W25Q128FV     //official
#define FLASH_CS_PIN            PB0
#define FLASH_SPI_INSTANCE      SPI1

#define USE_FLASH_W25Q          //testing
#define USE_FLASH_W25M512       //testing
#define USE_FLASH_M25P16        //testing

#define USE_I2C
#define USE_I2C_DEVICE_1
#define I2C_DEVICE_1           (I2CDEV_1)
#define I2C1_SCL               PB8
#define I2C1_SDA               PB9

#define USE_ADC
#define VBAT_ADC_PIN                PC1
#define CURRENT_METER_ADC_PIN       PC2
#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define DEFAULT_CURRENT_METER_SOURCE CURRENT_METER_ADC
#define CURRENT_METER_SCALE_DEFAULT 100

#define BEEPER_PWM_HZ           5400

#define USE_LED_STRIP
#define LED_STRIP_PIN        PA9

#define TARGET_IO_PORTA 0xffff
#define TARGET_IO_PORTB 0xffff
#define TARGET_IO_PORTC 0xffff
#define TARGET_IO_PORTD 0xffff
#define TARGET_IO_PORTE 0xffff

#define USABLE_TIMER_CHANNEL_COUNT      7
#define USED_TIMERS ( TIM_N(1) | TIM_N(3) | TIM_N(4) )
