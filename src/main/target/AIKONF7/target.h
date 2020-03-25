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
#define TARGET_BOARD_IDENTIFIER "AIKO"
#define USBD_PRODUCT_STRING "AIKONF7"

#define ENABLE_DSHOT_DMAR       true

#define LED0_PIN                  PC13
#define USE_BEEPER
#define BEEPER_PIN                PC15
#define BEEPER_INVERTED
#define USE_PINIO
#define PINIO1_PIN                PC14
#define USE_PINIOBOX

//define camera control
#define CAMERA_CONTROL_PIN        PB14

#define USE_EXTI
#define MPU_INT_EXTI              PC4
#define USE_MPU_DATA_READY_SIGNAL
//MPU-6000
#define USE_ACC
#define USE_ACC_SPI_MPU6000
#define USE_GYRO
#define USE_GYRO_SPI_MPU6000

#define GYRO_MPU6000_ALIGN      CW0_DEG
#define ACC_MPU6000_ALIGN       CW0_DEG
#define MPU6000_CS_PIN          PA4
#define MPU6000_SPI_INSTANCE    SPI1

// ICM-20602
#define USE_ACC_MPU6500
#define USE_ACC_SPI_MPU6500
#define USE_GYRO_MPU6500
#define USE_GYRO_SPI_MPU6500

#define ACC_MPU6500_ALIGN       CW0_DEG
#define GYRO_MPU6500_ALIGN      CW0_DEG
#define MPU6500_CS_PIN          SPI1_NSS_PIN
#define MPU6500_SPI_INSTANCE    SPI1

#define USE_MAG
#define USE_MAG_HMC5883
#define USE_MAG_QMC5883
#define MAG_I2C_INSTANCE         (I2CDEV_2)

#define USE_MAX7456
#define MAX7456_SPI_INSTANCE    SPI2
#define MAX7456_SPI_CS_PIN      PB12
#define MAX7456_SPI_CLK         (SPI_CLOCK_STANDARD) // 10MHz
#define MAX7456_RESTORE_CLK     (SPI_CLOCK_FAST)

#define ENABLE_BLACKBOX_LOGGING_ON_SPIFLASH_BY_DEFAULT
#define USE_FLASHFS
#define USE_FLASH_M25P16
#define FLASH_CS_PIN            PB0
#define FLASH_SPI_INSTANCE      SPI3

#define USE_VCP

#define USE_UART1
#define UART1_RX_PIN            PA10
#define UART1_TX_PIN            PA9

#define USE_UART2
#define UART2_RX_PIN            PA3
#define UART2_TX_PIN            PA2

#define USE_UART3
#define UART3_RX_PIN            PC11
#define UART3_TX_PIN            PC10

#define USE_UART4
#define UART4_RX_PIN            PA1
#define UART4_TX_PIN            PA0

#define USE_UART5
#define UART5_RX_PIN            PD2
#define UART5_TX_PIN            PC12


//#define USE_SOFTSERIAL1
#define SERIAL_PORT_COUNT       6 //VCP, USART1, USART2,USART3,USART4,USART5,USART6

#define USE_ESCSERIAL
#define ESCSERIAL_TIMER_TX_PIN  PB8 // (Hardware=0, PPM)

#define USE_I2C
#define I2C_DEVICE_2               (I2CDEV_2)
#define USE_I2C_DEVICE_2
#define I2C2_SCL                PB10
#define I2C2_SDA                PB11

/*
#define USE_BARO
#define BMP280_SPI_INSTANCE     SPI3
#define BMP280_CS_PIN           PB2
*/
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
#define SPI2_MISO_PIN            PC2
#define SPI2_MOSI_PIN            PB15

#define SPI3_NSS_PIN             PB0
#define SPI3_SCK_PIN             PB3
#define SPI3_MISO_PIN            PB4
#define SPI3_MOSI_PIN            PB5

#define USE_ADC
#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define DEFAULT_CURRENT_METER_SOURCE CURRENT_METER_ADC
#define CURRENT_METER_ADC_PIN   PC1
#define VBAT_ADC_PIN            PC0
#define RSSI_ADC_PIN            PC3
#define CURRENT_METER_SCALE_DEFAULT 250                     // 3.3/120A  = 25mv/A

#define BINDPLUG_PIN            PB2
#define DEFAULT_RX_FEATURE      FEATURE_RX_SERIAL
#define SERIALRX_UART           SERIAL_PORT_UART5

#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         0xffff
#define TARGET_IO_PORTD         (BIT(2))

#define USABLE_TIMER_CHANNEL_COUNT      9
#define USED_TIMERS ( TIM_N(2) | TIM_N(4) | TIM_N(8) | TIM_N(9) | TIM_N(12)   ) //update based on update CLRACINGF7 Target BF4.1+
