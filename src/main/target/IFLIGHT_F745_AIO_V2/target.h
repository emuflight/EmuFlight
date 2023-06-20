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

#define TARGET_BOARD_IDENTIFIER "IFRC"

#define USBD_PRODUCT_STRING "IFLIGHT_F745_AIO_V2"

#define LED0_PIN   PC13

#define USE_BEEPER
#define BEEPER_PIN PD2
#define BEEPER_INVERTED

//**********Gyro Acc *************//

#define USE_ACC
#define USE_GYRO
#define USE_DUAL_GYRO

#define USE_SPI_GYRO
#define USE_ACCGYRO_BMI270

#define USE_EXTI
#define USE_GYRO_EXTI
#define GYRO_1_EXTI_PIN             PD0
#define GYRO_2_EXTI_PIN             PD8
#define MPU_INT_EXTI

#define GYRO_1_CS_PIN               PA4
#define GYRO_1_SPI_INSTANCE         SPI1
#define GYRO_2_CS_PIN               PB12
#define GYRO_2_SPI_INSTANCE         SPI2

#define GYRO_1_ALIGN                CW0_DEG
#define ACC_1_ALIGN                 CW0_DEG
#define GYRO_2_ALIGN                CW180_DEG
#define ACC_2_ALIGN                 CW180_DEG

#define USE_MPU_DATA_READY_SIGNAL
#define ENSURE_MPU_DATA_READY_IS_LOW
#define GYRO_CONFIG_USE_GYRO_DEFAULT GYRO_CONFIG_USE_GYRO_1

//**********Mag and Baro**********//

#define USE_MAG
#define USE_MAG_HMC5883
#define USE_MAG_QMC5883
#define MAG_HMC5883_ALIGN       CW180_DEG
#define MAG_QMC5883_ALIGN       CW180_DEG
#define MAG_I2C_INSTANCE           (I2CDEV_1)

#define USE_BARO
#define USE_BARO_MS5611
#define USE_BARO_BMP280
#define USE_BARO_DPS310
#define BARO_I2C_INSTANCE           (I2CDEV_1)

//**********Serial****************//

#define USE_VCP
//#define USE_USB_DETECT
//#define USB_DETECT_PIN   PC4

#define USE_UART1
#define UART1_RX_PIN PA10
#define UART1_TX_PIN PA9

#define USE_UART2
#define UART2_RX_PIN PA3
#define UART2_TX_PIN PA2

#define USE_UART3
#define UART3_RX_PIN PB11
#define UART3_TX_PIN PB10

#define USE_UART4
#define UART4_RX_PIN PA1
#define UART4_TX_PIN PA0

#define USE_UART6
#define UART6_RX_PIN PC7
#define UART6_TX_PIN PC6

#define USE_UART7
#define UART7_RX_PIN PE7
#define UART7_TX_PIN PE8

#define USE_UART8
#define UART8_RX_PIN PE0
#define UART8_TX_PIN PE1

#define SERIAL_PORT_COUNT 8 //VCP, USART1, USART2, USART3, UART4, USART6, USART7, USART8

#define USE_ESCSERIAL
#define USE_CAMERA_CONTROL
#define CAMERA_CONTROL_PIN      PB3

#define USE_SPI
#define USE_SPI_DEVICE_1
#define USE_SPI_DEVICE_2
#define USE_SPI_DEVICE_3
#define USE_SPI_DEVICE_4

#define SPI1_SCK_PIN            PA5
#define SPI1_MISO_PIN           PA6
#define SPI1_MOSI_PIN           PA7

#define SPI2_SCK_PIN            PB13
#define SPI2_MISO_PIN           PB14
#define SPI2_MOSI_PIN           PB15

#define SPI3_SCK_PIN            PC10
#define SPI3_MISO_PIN           PC11
#define SPI3_MOSI_PIN           PC12

#define SPI4_SCK_PIN            PE2
#define SPI4_MISO_PIN           PE5
#define SPI4_MOSI_PIN           PE6

#define USE_MAX7456
#define MAX7456_SPI_INSTANCE    SPI4
#define MAX7456_SPI_CS_PIN      PE4
#define MAX7456_SPI_CLK         (SPI_CLOCK_STANDARD) // 10MHz
#define MAX7456_RESTORE_CLK     (SPI_CLOCK_FAST)

#define ENABLE_BLACKBOX_LOGGING_ON_SPIFLASH_BY_DEFAULT
#define USE_FLASHFS
#define USE_FLASH_M25P16
#define FLASH_CS_PIN            PA15
#define FLASH_SPI_INSTANCE      SPI3

#define USE_I2C
#define USE_I2C_DEVICE_1
#define I2C_DEVICE_1                  (I2CDEV_1)
#define I2C1_SCL               PB8
#define I2C1_SDA               PB9

#define USE_I2C_DEVICE_2
#define I2C_DEVICE_2                  (I2CDEV_2)
#define I2C2_SCL               PB10
#define I2C2_SDA               PB11

#define USE_ADC
#define VBAT_ADC_PIN                PC3
#define CURRENT_METER_ADC_PIN       PC2
#define RSSI_ADC_PIN                PC5
#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define DEFAULT_CURRENT_METER_SOURCE CURRENT_METER_ADC
#define CURRENT_METER_SCALE_DEFAULT 100

#define USE_LED_STRIP

#define DEFAULT_FEATURES        (FEATURE_OSD | FEATURE_TELEMETRY )
#define DEFAULT_RX_FEATURE      FEATURE_RX_SERIAL
#define SERIALRX_PROVIDER       SERIALRX_SBUS

#define TARGET_IO_PORTA 0xffff
#define TARGET_IO_PORTB 0xffff
#define TARGET_IO_PORTC 0xffff
#define TARGET_IO_PORTD 0xffff
#define TARGET_IO_PORTE 0xffff

#define USABLE_TIMER_CHANNEL_COUNT 10

#define USED_TIMERS  ( TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(4) | TIM_N(8) )
