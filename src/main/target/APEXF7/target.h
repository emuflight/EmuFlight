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
#define TARGET_BOARD_IDENTIFIER "APF7"
#define USBD_PRODUCT_STRING     "APEXF7"

#define USE_TARGET_CONFIG

#define ENABLE_DSHOT_DMAR       true

#define USE_BEEPER
#define BEEPER_PIN              PB0
#define BEEPER_INVERTED

#define USE_VCP

#define USE_UART1
#define UART1_RX_PIN            PA10
#define UART1_TX_PIN            PA9

#define USE_UART2
#define UART2_TX_PIN            PA2

#define USE_UART3
#define UART3_RX_PIN            PC11
#define UART3_TX_PIN            PC10

#define USE_UART4
#define UART4_RX_PIN            PA1
#define UART4_TX_PIN            PA0

#define USE_UART5
#define UART5_TX_PIN            PC12

#define USE_UART6
#define UART6_RX_PIN            PC7
#define UART6_TX_PIN            PC6

//#define USE_SOFTSERIAL1
#define SERIAL_PORT_COUNT       7 //VCP, USART1, USART2,USART3,USART4,USART5,USART6

#define USE_I2C
#define USE_I2C_DEVICE_1       // External I2C
#define I2C_DEVICE               (I2CDEV_1)
#define I2C1_SCL                PB8
#define I2C1_SDA                PB9

#define LED0_PIN                PB12

#define USE_SPI
#define USE_SPI_DEVICE_1
#define USE_SPI_DEVICE_2
#define USE_SPI_DEVICE_3

#define SPI1_SCK_PIN             PA5
#define SPI1_MISO_PIN            PA6
#define SPI1_MOSI_PIN            PA7

#define SPI2_SCK_PIN             PB13
#define SPI2_MISO_PIN            PB14
#define SPI2_MOSI_PIN            PB15

#define SPI3_SCK_PIN             PB3
#define SPI3_MISO_PIN            PB4
#define SPI3_MOSI_PIN            PB5

#define CAMERA_CONTROL_PIN      PA8

#define USE_ADC
#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define DEFAULT_CURRENT_METER_SOURCE CURRENT_METER_ADC
#define CURRENT_METER_ADC_PIN   PC1
#define VBAT_ADC_PIN            PC0
#define RSSI_ADC_PIN            PC2
#define CURRENT_METER_SCALE_DEFAULT 250                     // 3.3/120A  = 25mv/A

#define USE_PINIO
#define PINIO1_PIN              PC14
#define PINIO2_PIN              PB11
#define USE_PINIOBOX

#define ENABLE_BLACKBOX_LOGGING_ON_SPIFLASH_BY_DEFAULT
#define USE_FLASHFS
#define USE_FLASH_M25P16
#define FLASH_CS_PIN            PA3
#define FLASH_SPI_INSTANCE      SPI1

#define USE_MAX7456
#define MAX7456_SPI_INSTANCE    SPI2
#define MAX7456_SPI_CS_PIN      PC15
#define MAX7456_SPI_CLK         (SPI_CLOCK_STANDARD) // 10MHz
#define MAX7456_RESTORE_CLK     (SPI_CLOCK_FAST)

#define USE_DUAL_GYRO
#define USE_EXTI
#define MPU_INT_EXTI            PA4
#define USE_MPU_DATA_READY_SIGNAL

#define GYRO_1_EXTI_PIN         PA4
#define GYRO_2_EXTI_PIN         PC3
#define GYRO_1_CS_PIN           PD2
#define GYRO_1_SPI_INSTANCE     SPI3
#define GYRO_2_CS_PIN           PB10
#define GYRO_2_SPI_INSTANCE     SPI3

#define ACC_MPU6000_1_ALIGN         CW0_DEG
#define GYRO_MPU6000_1_ALIGN        CW0_DEG
#define GYRO_1_ALIGN                GYRO_MPU6000_1_ALIGN
#define ACC_1_ALIGN                 ACC_MPU6000_1_ALIGN

#define ACC_MPU6500_2_ALIGN         CW270_DEG
#define GYRO_MPU6500_2_ALIGN        CW270_DEG
#define GYRO_2_ALIGN                GYRO_MPU6500_2_ALIGN
#define ACC_2_ALIGN                 ACC_MPU6500_2_ALIGN

#define GYRO_CONFIG_USE_GYRO_DEFAULT GYRO_CONFIG_USE_GYRO_1

//MPU-6000
#define USE_ACC
#define USE_ACC_SPI_MPU6000
#define USE_GYRO
#define USE_GYRO_SPI_MPU6000


// ICM-20602
#define USE_ACC_MPU6500
#define USE_ACC_SPI_MPU6500
#define USE_GYRO_MPU6500
#define USE_GYRO_SPI_MPU6500

#define USE_BARO
#define USE_BARO_BMP280
#define BARO_I2C_INSTANCE         (I2CDEV_1)
#define USE_MAG
#define USE_MAG_HMC5883
#define USE_MAG_QMC5883
#define MAG_I2C_INSTANCE         (I2CDEV_1)

#define DEFAULT_FEATURES        (FEATURE_OSD | FEATURE_TELEMETRY | FEATURE_AIRMODE)
#define DEFAULT_RX_FEATURE      FEATURE_RX_SERIAL
#define SERIALRX_UART           SERIAL_PORT_USART1

#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         0xffff
#define TARGET_IO_PORTD         (BIT(2))

#define USABLE_TIMER_CHANNEL_COUNT      6
#define USED_TIMERS             ( TIM_N(1) | TIM_N(2) | TIM_N(4) | TIM_N(8)  )
