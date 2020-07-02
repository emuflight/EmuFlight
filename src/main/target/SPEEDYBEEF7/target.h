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

#define TARGET_BOARD_IDENTIFIER   "SBF7"
#define USBD_PRODUCT_STRING       "SpeedyBeeF7"

#define USE_TARGET_CONFIG

#define LED0_PIN                  PC14

#define USE_BEEPER
#define BEEPER_PIN                PC13
#define BEEPER_INVERTED

#define ENABLE_DSHOT_DMAR         true

// *************** Gyro & ACC **********************
#define USE_SPI
#define USE_SPI_DEVICE_1

#define SPI1_SCK_PIN              PA5
#define SPI1_MISO_PIN             PA6
#define SPI1_MOSI_PIN             PA7

#define ICM20689_CS_PIN           PA4
#define ICM20689_SPI_INSTANCE     SPI1

//#define USE_EXTI
#define USE_EXTI
#define MPU_INT_EXTI              PC4
#define GYRO_1_EXTI_PIN           MPU_INT_EXTI
#define USE_MPU_DATA_READY_SIGNAL

//#define USE_GYRO
#define USE_GYRO_SPI_ICM20689

#define USE_ACC
#define USE_ACC_SPI_ICM20689

#define USE_MAG
#define USE_MAG_HMC5883
#define USE_MAG_QMC5883
//#define USE_MAG_LIS3MDL

// *************** Baro **************************
#define USE_I2C

#define USE_I2C_DEVICE_1
#define I2C1_SCL                  PB8
#define I2C1_SDA                  PB9
#define BARO_I2C_INSTANCE         (I2CDEV_1)

#define USE_I2C_DEVICE_2
#define I2C_DEVICE                (I2CDEV_2)
#define I2C2_SCL                  PB10
#define I2C2_SDA                  PB11

#define USE_BARO
#define USE_BARO_BMP280
#define DEFAULT_BARO_BMP280

// *************** Flash **************************
#define USE_SPI_DEVICE_3
#define SPI3_SCK_PIN              PB3
#define SPI3_MISO_PIN             PB4
#define SPI3_MOSI_PIN             PB5

#define USE_FLASHFS
#define USE_FLASH_M25P16
#define FLASH_CS_PIN              PC0
#define FLASH_SPI_INSTANCE        SPI3
#define ENABLE_BLACKBOX_LOGGING_ON_SPIFLASH_BY_DEFAULT

// *************** OSD *****************************
#define USE_SPI_DEVICE_2
#define SPI2_SCK_PIN              PB13
#define SPI2_MISO_PIN             PB14
#define SPI2_MOSI_PIN             PB15

#define USE_MAX7456
#define MAX7456_SPI_INSTANCE      SPI2
#define MAX7456_SPI_CS_PIN        PB12

// *************** UART *****************************
#define USE_VCP
#define USB_DETECT_PIN            PC15
#define USE_USB_DETECT

#define USE_UART1
#define UART1_RX_PIN              PA10
#define UART1_TX_PIN              PA9

#define USE_UART2
#define UART2_RX_PIN              PA3
#define UART2_TX_PIN              PA2

#define USE_UART3
#define UART3_RX_PIN              PC11
#define UART3_TX_PIN              PC10

#define USE_UART4
#define UART4_RX_PIN              PA1
#define UART4_TX_PIN              PA0

#define USE_UART5
#define UART5_RX_PIN              PD2
#define UART5_TX_PIN              PC12

#define USE_SOFTSERIAL1
#define USE_SOFTSERIAL2

#define SOFTSERIAL1_TX_PIN        PB0

#define SERIAL_PORT_COUNT         8

#define DEFAULT_RX_FEATURE        FEATURE_RX_SERIAL
#define SERIALRX_PROVIDER         SERIALRX_SBUS
#define SERIALRX_UART             SERIAL_PORT_USART2

// *************** ADC *****************************
#define USE_ADC
#define ADC_INSTANCE              ADC1
#define ADC1_DMA_OPT              0

#define CURRENT_METER_ADC_PIN     PC1
#define VBAT_ADC_PIN              PC2
#define RSSI_ADC_PIN              PC3

#define DEFAULT_FEATURES        (FEATURE_OSD | FEATURE_TELEMETRY | FEATURE_SOFTSERIAL)
#define DEFAULT_VOLTAGE_METER_SOURCE    VOLTAGE_METER_ADC
#define DEFAULT_CURRENT_METER_SOURCE    CURRENT_METER_ADC
#define CURRENT_METER_SCALE_DEFAULT 102

#define USE_ESCSERIAL
#define ESCSERIAL_TIMER_TX_PIN    PA3

#define TARGET_IO_PORTA           0xffff
#define TARGET_IO_PORTB           0xffff
#define TARGET_IO_PORTC           0xffff
#define TARGET_IO_PORTD           (BIT(2))

// *************** activate/deactivate Bluetooth When disarmed/armed using PINIO_BOX *****************************
#define PINIO1_PIN                PA15

#define USABLE_TIMER_CHANNEL_COUNT 10
#define USED_TIMERS             (TIM_N(1)|TIM_N(3)|TIM_N(4)|TIM_N(5)|TIM_N(8))
