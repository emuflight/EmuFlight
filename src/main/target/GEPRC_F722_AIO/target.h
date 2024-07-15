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
 *
 * Prepared by Kaio
 */
#pragma once

#define BOARD_NAME        GEPRC_F722_AIO
#define MANUFACTURER_ID   GEPR
#define TARGET_BOARD_IDENTIFIER "S7X2"  // generic ID
#define FC_TARGET_MCU     STM32F7X2     // not used in EmuF

#define LED0_PIN PC4

#define USE_BEEPER
#define BEEPER_PIN PC15
#define BEEPER_INVERTED

#define ENABLE_DSHOT_DMAR  true

#define USE_GYRO
#define USE_GYRO_SPI_MPU6000
#define USE_ACC
#define USE_ACC_SPI_MPU6000
#define USE_ACCGYRO_BMI270
#define USE_GYRO_SPI_ICM42688P
#define USE_ACC_SPI_ICM42688P

#define USE_EXTI
#define USE_SPI_GYRO
#define USE_GYRO_EXTI

#define MPU_INT_EXTI PA8
#define USE_MPU_DATA_READY_SIGNAL

#define ACC_MPU6000_ALIGN        CW90_DEG
#define GYRO_MPU6000_ALIGN       CW90_DEG
#define MPU6000_CS_PIN           PA15
#define MPU6000_SPI_INSTANCE     SPI1

#define ACC_ICM42688P_ALIGN      CW90_DEG
#define GYRO_ICM42688P_ALIGN     CW90_DEG
#define ICM42688P_CS_PIN         PA15
#define ICM42688P_SPI_INSTANCE   SPI1

#define ACC_BMI270_ALIGN         CW90_DEG
#define GYRO_BMI270_ALIGN        CW90_DEG
#define BMI270_CS_PIN            PA15
#define BMI270_SPI_INSTANCE      SPI1

#define USE_BARO
#define USE_BARO_BMP280
#define USE_BARO_BMP085
#define USE_BARO_MS5611
#define BARO_I2C_INSTANCE (I2CDEV_2)

#define USE_MAG
#define USE_MAG_HMC5883
#define USE_MAG_QMC5883
#define MAG_I2C_INSTANCE (I2CDEV_2)

#define USE_MAX7456
#define MAX7456_SPI_INSTANCE SPI2
#define MAX7456_SPI_CS_PIN PB12

#define USE_FLASHFS
#define USE_FLASH_M25P16
#define FLASH_SPI_INSTANCE SPI3
#define FLASH_CS_PIN PB9
#define ENABLE_BLACKBOX_LOGGING_ON_SPIFLASH_BY_DEFAULT

#define USE_VCP
#define USE_USB_DETECT

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
#define UART4_RX_PIN PC11
#define UART4_TX_PIN PC10

#define USE_UART5
#define UART5_RX_PIN PD2
#define UART5_TX_PIN PC12

#define SERIAL_PORT_COUNT 6     //USB + 5 UARTS

#define USE_ESCSERIAL                  //PPM
#define ESCSERIAL_TIMER_TX_PIN PA3

#define USE_SPI
#define USE_SPI_DEVICE_1
#define SPI1_SCK_PIN PA5
#define SPI1_MISO_PIN PA6
#define SPI1_MOSI_PIN PA7

#define USE_SPI_DEVICE_2
#define SPI2_SCK_PIN PB13
#define SPI2_MISO_PIN PB14
#define SPI2_MOSI_PIN PB15

#define USE_SPI_DEVICE_3
#define SPI3_SCK_PIN PB3
#define SPI3_MISO_PIN PB4
#define SPI3_MOSI_PIN PB5

//#define USE_I2C
//#define USE_I2C_DEVICE_1
//#define I2C1_SCL PB8
//#define I2C1_SDA PB9
//#define I2C_DEVICE (I2CDEV_1)

#define USE_I2C_DEVICE_2
#define I2C2_SCL PB10
#define I2C2_SDA PB11
#define I2C_DEVICE (I2CDEV_2)

#define USE_ADC
#define ADC3_DMA_STREAM DMA2_Stream0
#define VBAT_ADC_PIN PC2
#define CURRENT_METER_ADC_PIN PC1
//#define RSSI_ADC_PIN PC2
#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define DEFAULT_CURRENT_METER_SOURCE CURRENT_METER_ADC
#define CURRENT_METER_SCALE_DEFAULT 100

#define SERIALRX_PROVIDER SERIALRX_SBUS
//#define SERIALRX_UART SERIAL_PORT_USART2

#define DEFAULT_RX_FEATURE FEATURE_RX_SERIAL
#define DEFAULT_FEATURES ( FEATURE_OSD | FEATURE_TELEMETRY )

#define TARGET_IO_PORTA ( 0xffff )
#define TARGET_IO_PORTB ( 0xffff )
#define TARGET_IO_PORTC ( 0xffff )
#define TARGET_IO_PORTD ( 0xffff )

#define USABLE_TIMER_CHANNEL_COUNT 7
#define USED_TIMERS ( TIM_N(2) | TIM_N(5) | TIM_N(8) | TIM_N(9) )
