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

#if defined(IFF4_TWIN_G_M)
#define TARGET_BOARD_IDENTIFIER "S405"
#define USBD_PRODUCT_STRING "IFLIGHTF4TWIN_G_mpu6000"
#else
#define TARGET_BOARD_IDENTIFIER "S405"
#define USBD_PRODUCT_STRING "IFLIGHTF4TWIN_G"
#endif

#define USE_DSHOT_DMAR
#define ENABLE_DSHOT_DMAR       true

#define INVERTER_PIN_UART1      PC13

#define LED0_PIN                PB6

#define USE_BEEPER
#define BEEPER_PIN              PB4
#define BEEPER_INVERTED

//define camera control
#define CAMERA_CONTROL_PIN      PA0

#define USE_PINIO
#define USE_PINIOBOX
#define PINIO1_PIN              PC14
#define PINIO2_PIN              PC15

#if defined(IFF4_TWIN_G_M)
#define USE_MPU_DATA_READY_SIGNAL

#define USE_EXTI
#define MPU_INT_EXTI            PA8
#define MPU6000_SPI_INSTANCE    SPI1
#define MPU6000_CS_PIN          PA4

#define USE_GYRO
#define USE_GYRO_SPI_MPU6000
#define USE_ACC
#define USE_ACC_SPI_MPU6000

#define GYRO_MPU6000_ALIGN      CW0_DEG
#define ACC_MPU6000_ALIGN       CW0_DEG

#define I2C1_SCL                PB10
#define I2C1_SDA                PB11

#else
#define USE_DUAL_GYRO

#define USE_EXTI
#define USE_GYRO_EXTI
#define MPU_INT_EXTI          PA8
#define GYRO_2_EXTI_PIN       PC4

#define USE_MPU_DATA_READY_SIGNAL
#define ENSURE_MPU_DATA_READY_IS_LOW

#define GYRO_1_SPI_INSTANCE     SPI1
#define GYRO_1_CS_PIN           PA4
#define GYRO_2_SPI_INSTANCE     SPI1
#define GYRO_2_CS_PIN           PC3

#define USE_GYRO
#define USE_GYRO_SPI_ICM20689
#define USE_ACC
#define USE_ACC_SPI_ICM20689

#define GYRO_1_ALIGN        CW0_DEG
#define ACC_1_ALIGN         CW0_DEG
#define GYRO_2_ALIGN        CW90_DEG
#define ACC_2_ALIGN         CW90_DEG

#define GYRO_CONFIG_USE_GYRO_DEFAULT GYRO_CONFIG_USE_GYRO_BOTH

#define I2C1_SCL                PB8
#define I2C1_SDA                PB9

#endif

#define USE_MAX7456
#define MAX7456_SPI_INSTANCE    SPI2
#define MAX7456_SPI_CS_PIN      PB12
#define MAX7456_SPI_CLK         (SPI_CLOCK_STANDARD) // 10MHz
#define MAX7456_RESTORE_CLK     (SPI_CLOCK_FAST)

#define ENABLE_BLACKBOX_LOGGING_ON_SPIFLASH_BY_DEFAULT
#define USE_FLASHFS
#define USE_FLASH_M25P16
#define FLASH_CS_PIN            PA15
#define FLASH_SPI_INSTANCE      SPI3

#define USE_I2C
#define USE_I2C_DEVICE_1
#define I2C_DEVICE              (I2CDEV_1)
#define BARO_I2C_INSTANCE       I2CDEV_1
#define MAG_I2C_INSTANCE        I2CDEV_1

#define USE_VCP

#define USE_UART1
#define UART1_RX_PIN            PA10
#define UART1_TX_PIN            PA9

#define USE_UART2
#define UART2_RX_PIN            PA3
#define UART2_TX_PIN            PA2

#define USE_UART3
#define UART3_RX_PIN            PB11
#define UART3_TX_PIN            PB10

#define USE_UART6
#define UART6_RX_PIN            PC7
#define UART6_TX_PIN            PC6

#define USE_UART11
#define UART11_RX_PIN           PB8
#define UART11_TX_PIN           PB9

//#define USE_SOFTSERIAL1
#define SERIAL_PORT_COUNT       6 //VCP, USART1, USART2,USART3,USART4,USART5,USART6

#define USE_BARO
#define USE_BARO_BMP280
#define USE_BARO_MS5611
#define USE_BARO_BMP085

#define USE_MAG
#define USE_MAG_HMC5883
#define USE_MAG_QMC5883

#define USE_SPI
#define USE_SPI_DEVICE_1
#define USE_SPI_DEVICE_2
#define USE_SPI_DEVICE_3

#define SPI1_NSS_PIN             PA4
#define SPI1_SCK_PIN             PA5
#define SPI1_MISO_PIN            PA6
#define SPI1_MOSI_PIN            PA7

#define SPI2_NSS_PIN             PB12
#define SPI2_SCK_PIN             PB13
#define SPI2_MISO_PIN            PB14
#define SPI2_MOSI_PIN            PB15

#define SPI3_NSS_PIN             PA15
#define SPI3_SCK_PIN             PC10
#define SPI3_MISO_PIN            PC11
#define SPI3_MOSI_PIN            PC12

#define USE_ADC
#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define DEFAULT_CURRENT_METER_SOURCE CURRENT_METER_ADC
#define CURRENT_METER_ADC_PIN   PC1
#define VBAT_ADC_PIN            PC2
//#define RSSI_ADC_PIN            PC3
#define CURRENT_METER_SCALE_DEFAULT 250                     // 3.3/120A  = 25mv/A

#define BINDPLUG_PIN            PC5
#define DEFAULT_RX_FEATURE      FEATURE_RX_SERIAL
#define SERIALRX_UART           SERIAL_PORT_UART5

#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         0xffff
#define TARGET_IO_PORTD         (BIT(2))

#define USABLE_TIMER_CHANNEL_COUNT      7
#define USED_TIMERS             ( TIM_N(2) | TIM_N(3) | TIM_N(4) | TIM_N(8)   )
