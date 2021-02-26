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

#if defined(DIAT_MAMBAF405US_I2C)
#define TARGET_BOARD_IDENTIFIER "S405"
#define USBD_PRODUCT_STRING "MAMBAF405US I2C"
#else
#define TARGET_BOARD_IDENTIFIER "S405"
#define USBD_PRODUCT_STRING "MAMBAF405US"
#endif

#define USE_TARGET_CONFIG

#define LED0_PIN                  PC15
#define LED1_PIN                  PC14

#define USE_BEEPER
#define BEEPER_PIN                PC13
#define BEEPER_INVERTED

#define USE_PINIO
#define PINIO1_PIN                PB0
#define USE_PINIOBOX

#define ENABLE_DSHOT_DMAR        true

#define CAMERA_CONTROL_PIN        PB8
#define INVERTER_PIN_UART1        PC0

#define USE_EXTI
#define MPU_INT_EXTI              PC4
#define USE_MPU_DATA_READY_SIGNAL

#define MPU6000_CS_PIN            PA4
#define MPU6000_SPI_INSTANCE      SPI1
#define USE_ACC
#define USE_ACC_SPI_MPU6000
#define USE_GYRO
#define USE_GYRO_SPI_MPU6000
#define GYRO_MPU6000_ALIGN        CW180_DEG
#define ACC_MPU6000_ALIGN         CW180_DEG

#define USE_MAX7456
#define MAX7456_SPI_INSTANCE      SPI2
#define MAX7456_SPI_CS_PIN        PB12
#define MAX7456_SPI_CLK           (SPI_CLOCK_STANDARD)
#define MAX7456_RESTORE_CLK       (SPI_CLOCK_FAST)

#define ENABLE_BLACKBOX_LOGGING_ON_SPIFLASH_BY_DEFAULT
#define USE_FLASHFS
#define USE_FLASH_M25P16
#define FLASH_CS_PIN              PA15
#define FLASH_SPI_INSTANCE        SPI3

#define USE_BARO
#define USE_BARO_MS5611
#define USE_BARO_BMP280
#define USE_BARO_BMP085

#define USE_I2C
#define USE_I2C_DEVICE_2
#define I2C_DEVICE              (I2CDEV_2)
#if defined(DIAT_MAMBAF405US_I2C)
#define I2C2_SCL                PB8        // SCL pad  PB10, shared with UART3TX
#define I2C2_SDA                PB9        // SDA pad  PB11, shared with UART3RX
#else
#define I2C2_SCL                NONE        // SCL pad  PB10, shared with UART3TX
#define I2C2_SDA                NONE        // SDA pad  PB11, shared with UART3RX
#endif
#define BARO_I2C_INSTANCE       (I2CDEV_2)

#define USE_MAG
#define USE_MAG_HMC5883
#define USE_MAG_QMC5883

#define USE_VCP
#define USE_USB_DETECT
#define USB_DETECT_PIN            PC5

#define USE_UART1
#define UART1_RX_PIN              PB7
#define UART1_TX_PIN              PB6

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
#define ESCSERIAL_TIMER_TX_PIN    PB9

#define USE_SPI
#define USE_SPI_DEVICE_1
#define SPI1_NSS_PIN              PA4
#define SPI1_SCK_PIN              PA5
#define SPI1_MISO_PIN             PA6
#define SPI1_MOSI_PIN             PA7

#define USE_SPI_DEVICE_2
#define SPI2_NSS_PIN              PB12
#define SPI2_SCK_PIN              PB13
#define SPI2_MISO_PIN             PB14
#define SPI2_MOSI_PIN             PB15

#define USE_SPI_DEVICE_3
#define SPI3_NSS_PIN              PA15
#define SPI3_SCK_PIN              PC10
#define SPI3_MISO_PIN             PC11
#define SPI3_MOSI_PIN             PB5

#define USE_ADC
#define CURRENT_METER_ADC_PIN     PC3
#define VBAT_ADC_PIN              PC1
#define RSSI_ADC_PIN              PC2
#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define DEFAULT_CURRENT_METER_SOURCE CURRENT_METER_ADC

#define DEFAULT_RX_FEATURE        FEATURE_RX_SERIAL
#define DEFAULT_FEATURES          ( FEATURE_OSD )
#define CURRENT_METER_SCALE_DEFAULT 250

#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         0xffff
#define TARGET_IO_PORTD         (BIT(2))

#define USABLE_TIMER_CHANNEL_COUNT 11
#define USED_TIMERS  (  TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(4) | TIM_N(8) | TIM_N(11))
