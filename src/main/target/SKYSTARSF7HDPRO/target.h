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

#define USE_TARGET_CONFIG

#define TARGET_MANUFACTURER_IDENTIFIER "SKST"
#define USBD_PRODUCT_STRING     "SKST7HDPRO"

#define FC_TARGET_MCU     STM32F7X2     // not used in EmuF
#define TARGET_BOARD_IDENTIFIER "S7X2"

// ******* LEDs and BEEPER ********

#define LED0_PIN                PC15
#define LED1_PIN                PC14

#define USE_BEEPER
#define BEEPER_PIN              PB2
#define BEEPER_INVERTED

//#define ENABLE_DSHOT_DMAR       true

#define USE_PINIO
#define PINIO1_PIN PA14 // Bluetooth mode control, PB0 is connected to the 36 pin (P2.0) of the Bluetooth chip. Replace PB0 with the pin for your flight control and 36-pin connection

#define USE_CAMERA_CONTROL
#define CAMERA_CONTROL_PIN      PA8 // define dedicated camera osd pin


// ******* GYRO and ACC ********

#define USE_DUAL_GYRO
#define USE_EXTI
#define GYRO_1_EXTI_PIN         PC4
#define GYRO_2_EXTI_PIN         PC0
#define MPU_INT_EXTI

#define GYRO_1_CS_PIN           PA4
#define GYRO_1_SPI_INSTANCE     SPI1
#define GYRO_2_CS_PIN           PC13
#define GYRO_2_SPI_INSTANCE     SPI1

#define USE_GYRO
#define USE_GYRO_SPI_MPU6000
#define USE_GYRO_SPI_MPU6500
#define USE_SPI_GYRO
#define USE_ACCGYRO_BMI270

#define USE_ACC
#define USE_ACC_SPI_MPU6000
#define USE_ACC_SPI_MPU6500

#define GYRO_1_ALIGN     CW90_DEG_FLIP
#define ACC_1_ALIGN      CW90_DEG_FLIP

#define GYRO_2_ALIGN    CW0_DEG
#define ACC_2_ALIGN     CW0_DEG

#define USE_MPU_DATA_READY_SIGNAL
#define ENSURE_MPU_DATA_READY_IS_LOW
#define GYRO_CONFIG_USE_GYRO_DEFAULT GYRO_CONFIG_USE_GYRO_1

// *************** Baro ************************

#define USE_SPI

#define USE_SPI_DEVICE_2
#define SPI2_SCK_PIN            PB13
#define SPI2_MISO_PIN           PB14
#define SPI2_MOSI_PIN           PB15

#define USE_BARO
#define USE_BARO_SPI_BMP280
#define DEFAULT_BARO_SPI_BMP280
#define BMP280_SPI_INSTANCE     SPI2
#define BMP280_CS_PIN           PB1

#define USE_I2C
#define USE_I2C_DEVICE_1
#define I2C1_SCL                PB8
#define I2C1_SDA                PB9


//*********** Magnetometer / Compass *************
#define USE_MAG
#define USE_MAG_HMC5883
#define USE_MAG_QMC5883
#define MAG_I2C_INSTANCE           (I2CDEV_1)

// ******* SERIAL ********

#define USE_VCP

#define USE_UART1
#define USE_UART2
#define USE_UART3
#define USE_UART4
#define USE_UART5
#define USE_UART6

#define UART1_TX_PIN            PA9
#define UART1_RX_PIN            PA10

#define UART2_TX_PIN            PA2
#define UART2_RX_PIN            PA3

#define UART3_TX_PIN            PB10
#define UART3_RX_PIN            PB11

#define UART4_TX_PIN            PA0
#define UART4_RX_PIN            PA1

#define UART5_TX_PIN            PC12
#define UART5_RX_PIN            PD2

#define UART6_TX_PIN            PC6
#define UART6_RX_PIN            PC7

#define USE_SOFTSERIAL1
#define USE_SOFTSERIAL2

#define SERIAL_PORT_COUNT       9 //VCP, UART1-UART6 , 2 x Soft Serial

// ******* SPI ********

#define USE_SPI

#define USE_SPI_DEVICE_1
#define SPI1_SCK_PIN            PA5
#define SPI1_MISO_PIN           PA6
#define SPI1_MOSI_PIN           PA7


#define USE_SPI_DEVICE_3
#define SPI3_SCK_PIN            PC10
#define SPI3_MISO_PIN           PC11
#define SPI3_MOSI_PIN           PB5

// ******* ADC ********

#define USE_ADC
#define ADC_INSTANCE            ADC2
#define ADC2_DMA_STREAM         DMA2_Stream3

#define VBAT_ADC_PIN            PC1
#define RSSI_ADC_PIN            PC2
#define CURRENT_METER_ADC_PIN   PC3

// ******* OSD ********

#define USE_MAX7456
#define MAX7456_SPI_INSTANCE    SPI2
#define MAX7456_SPI_CS_PIN      PB12
#define MAX7456_SPI_CLK         ( SPI_CLOCK_STANDARD )
#define MAX7456_RESTORE_CLK     ( SPI_CLOCK_FAST )

//******* FLASH ********

#define USE_FLASHFS
#define USE_FLASH_M25P16
#define FLASH_CS_PIN            PA15
#define FLASH_SPI_INSTANCE      SPI3

#define ENABLE_BLACKBOX_LOGGING_ON_SPIFLASH_BY_DEFAULT

// ******* FEATURES ********

#define USE_OSD

#define DEFAULT_RX_FEATURE      FEATURE_RX_SERIAL
#define SERIALRX_UART           SERIAL_PORT_USART1
#define SERIALRX_PROVIDER       SERIALRX_SBUS

#define DEFAULT_FEATURES                (FEATURE_OSD | FEATURE_TELEMETRY | FEATURE_SOFTSERIAL)
#define DEFAULT_VOLTAGE_METER_SOURCE    VOLTAGE_METER_ADC
#define DEFAULT_CURRENT_METER_SOURCE    CURRENT_METER_ADC
#define CURRENT_METER_SCALE_DEFAULT 290


#define USE_ESCSERIAL

#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         0xffff
#define TARGET_IO_PORTD         0xffff

#define USABLE_TIMER_CHANNEL_COUNT 7
#define USED_TIMERS             ( TIM_N(1) | TIM_N(2) | TIM_N(3)| TIM_N(4) | TIM_N(8) )
