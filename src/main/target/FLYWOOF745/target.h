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

 #define TARGET_BOARD_IDENTIFIER "FLWO"
 #define USBD_PRODUCT_STRING "FLYWOOF745"

 #define LED0_PIN   PA2

 #define USE_BEEPER
 #define BEEPER_PIN PD15
 #define BEEPER_INVERTED

 #define MPU6000_CS_PIN        PE4
 #define MPU6000_SPI_INSTANCE  SPI4

 #define USE_ACC
 #define USE_ACC_SPI_MPU6000
 #define ACC_MPU6000_ALIGN CW0_DEG

 #define USE_GYRO
 #define USE_GYRO_SPI_MPU6000
 #define GYRO_MPU6000_ALIGN CW270_DEG

 // MPU6000 interrupts
 #define USE_MPU_DATA_READY_SIGNAL
 #define MPU_INT_EXTI PE1
 #define USE_EXTI

 #define USE_MAG
 #define USE_MAG_HMC5883
 #define USE_MAG_QMC5883
 #define USE_MAG_LIS3MDL
 #define MAG_I2C_INSTANCE           (I2CDEV_1)
 //#define MAG_HMC5883_ALIGN CW270_DEG_FLIP
 //#define MAG_HMC5883_ALIGN CW90_DEG

 #define USE_BARO
 #define USE_BARO_BMP280
 #define BARO_I2C_INSTANCE           (I2CDEV_1)

 #define USABLE_TIMER_CHANNEL_COUNT 14

 #define USE_VCP
 #define USE_USB_DETECT
 #define USB_DETECT_PIN   PA8

 #define USE_UART1
 #define UART1_RX_PIN PA10
 #define UART1_TX_PIN PA9

 #define USE_UART2
 #define UART2_RX_PIN PD6
 #define UART2_TX_PIN PD5

 #define USE_UART3
 #define UART3_RX_PIN PB11
 #define UART3_TX_PIN PB10

 #define USE_UART4
 #define UART4_RX_PIN PA1
 #define UART4_TX_PIN PA0

 #define USE_UART5
 #define UART5_RX_PIN PD2
 #define UART5_TX_PIN PC12

 #define USE_UART6
 #define UART6_RX_PIN PC7
 #define UART6_TX_PIN PC6

 #define USE_UART7
 #define UART7_RX_PIN PE7
 #define UART7_TX_PIN PE8

 #define SERIAL_PORT_COUNT 8 //VCP, USART1, USART2, USART3, UART4, UART5, UART6, USART7

 #define USE_ESC_SENSOR

 #define USE_SPI
 #define USE_SPI_DEVICE_1  //FLASH
 #define USE_SPI_DEVICE_2  //OSD
 #define USE_SPI_DEVICE_4  //ICM20689


 #define SPI1_SCK_PIN            PA5
 #define SPI1_MISO_PIN           PA6
 #define SPI1_MOSI_PIN           PA7


 #define SPI2_SCK_PIN            PB13
 #define SPI2_MISO_PIN           PB14
 #define SPI2_MOSI_PIN           PB15


 #define SPI4_SCK_PIN            PE2
 #define SPI4_MISO_PIN           PE5
 #define SPI4_MOSI_PIN           PE6

 #define USE_MAX7456
 #define MAX7456_SPI_INSTANCE    SPI2
 #define MAX7456_SPI_CS_PIN      PB12
 #define MAX7456_SPI_CLK         (SPI_CLOCK_STANDARD) // 10MHz
 #define MAX7456_RESTORE_CLK     (SPI_CLOCK_FAST)

 #define ENABLE_BLACKBOX_LOGGING_ON_SDCARD_BY_DEFAULT
 #define ENABLE_BLACKBOX_LOGGING_ON_SPIFLASH_BY_DEFAULT
 #define USE_FLASHFS
 #define USE_FLASH_M25P16
 #define FLASH_CS_PIN            PA4
 #define FLASH_SPI_INSTANCE      SPI1

 #define USE_I2C
 #define USE_I2C_DEVICE_1
 #define I2C_DEVICE                  (I2CDEV_1)
 #define I2C1_SCL               PB6
 #define I2C1_SDA               PB7

 #define USE_ADC
 #define VBAT_ADC_PIN                PC3
 #define CURRENT_METER_ADC_PIN       PC2
 #define RSSI_ADC_PIN                PC5

 #define USE_LED_STRIP
 #define LED_STRIP_PIN         PD12

 #define DEFAULT_FEATURES        (FEATURE_TELEMETRY | FEATURE_OSD )
 #define DEFAULT_RX_FEATURE      FEATURE_RX_SERIAL
 #define SERIALRX_PROVIDER       SERIALRX_SBUS
 #define SERIALRX_UART           SERIAL_PORT_USART3
 #define USE_DSHOT

 #define TARGET_IO_PORTA 0xffff
 #define TARGET_IO_PORTB 0xffff
 #define TARGET_IO_PORTC 0xffff
 #define TARGET_IO_PORTD 0xffff
 #define TARGET_IO_PORTE 0xffff

 #define USED_TIMERS  ( TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(4) | TIM_N(5) | TIM_N(8) | TIM_N(9) )
