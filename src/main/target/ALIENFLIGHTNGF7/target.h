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

#define BOARD_NAME        ALIENFLIGHTNGF7
#define MANUFACTURER_ID   AFNG
#define TARGET_BOARD_IDENTIFIER "S7X2"  // generic ID
#define FC_TARGET_MCU     STM32F7X2     // not used in EmuF

#define USE_HARDWARE_REVISION_DETECTION
#define HW_PIN                  PC13

#define LED0_PIN                PC12
#define LED1_PIN                PD2

#define USE_BEEPER
#define BEEPER_PIN              PC13
#define BEEPER_INVERTED

// MPU interrupt
#define USE_EXTI
#define MPU_INT_EXTI            PC14
//#define DEBUG_MPU_DATA_READY_INTERRUPT
#define USE_MPU_DATA_READY_SIGNAL
#define ENSURE_MPU_DATA_READY_IS_LOW

// ICM2060x detected by MPU6500 driver

#define USE_ACC
#define USE_ACC_SPI_MPU6500
#define USE_ACC_SPI_MPU9250

#define USE_GYRO
#define USE_GYRO_SPI_MPU6500
#define USE_GYRO_SPI_MPU9250

#define ACC_MPU6500_ALIGN       CW270_DEG
#define GYRO_MPU6500_ALIGN      CW270_DEG
#define MPU6500_CS_PIN          SPI1_NSS_PIN
#define MPU6500_SPI_INSTANCE    SPI1

#define ACC_MPU9250_ALIGN       CW270_DEG
#define GYRO_MPU9250_ALIGN      CW270_DEG
#define MPU9250_CS_PIN          PA4
#define MPU9250_SPI_INSTANCE    SPI1

#define USE_MAG
#define USE_MAG_HMC5883
#define USE_MAG_SPI_HMC5883
#define USE_MAG_QMC5883
#define USE_MAG_AK8963
#define USE_MAG_SPI_AK8963

#define HMC5883_CS_PIN          PC15
#define HMC5883_SPI_INSTANCE    SPI3

#define AK8963_CS_PIN           PC15
#define AK8963_SPI_INSTANCE     SPI3

#define MAG_AK8963_ALIGN        CW180_DEG_FLIP

#define USE_BARO
#define USE_BARO_MS5611
#define USE_BARO_SPI_MS5611
#define USE_BARO_BMP280
#define USE_BARO_SPI_BMP280

#define MS5611_CS_PIN           SPI3_NSS_PIN
#define MS5611_SPI_INSTANCE     SPI3

#define BMP280_CS_PIN           SPI3_NSS_PIN
#define BMP280_SPI_INSTANCE     SPI3

#define USE_SDCARD

#define SDCARD_DETECT_INVERTED
#define SDCARD_DETECT_PIN               PB11

#define SDCARD_SPI_INSTANCE             SPI2
#define SDCARD_SPI_CS_PIN               PB10

// SPI2 is on the APB1 bus whose clock runs at 84MHz. Divide to under 400kHz for init:
#define SDCARD_SPI_INITIALIZATION_CLOCK_DIVIDER 256 // 422kHz
// Divide to under 25MHz for normal operation:
#define SDCARD_SPI_FULL_SPEED_CLOCK_DIVIDER 8 // 27MHz

#define SDCARD_DMA_STREAM_TX_FULL           DMA1_Stream4
#define SDCARD_DMA_CHANNEL                  0

// Performance logging for SD card operations:
// #define AFATFS_USE_INTROSPECTIVE_LOGGING

#define FLASH_CS_PIN         SPI2_NSS_PIN
#define FLASH_SPI_INSTANCE   SPI2

#define USE_FLASHFS
#define USE_FLASH_M25P16

#define USE_VCP

#define USE_UART1
#define UART1_RX_PIN            PA10
#define UART1_TX_PIN            PA9

#define USE_UART2
#define UART2_RX_PIN            PA3
#define UART2_TX_PIN            PA2

#define USE_UART3
#define UART3_RX_PIN            NONE
#define UART3_TX_PIN            NONE

#define USE_UART4
#define UART4_RX_PIN            PC11
#define UART4_TX_PIN            PC10

#define USE_SOFTSERIAL1
#define USE_SOFTSERIAL2

#define SERIAL_PORT_COUNT       7

#define USE_ESCSERIAL
#define ESCSERIAL_TIMER_TX_PIN  PA8 // (Hardware=0, PPM/LED_STRIP) XXX Crash if using an LED strip.

#define USE_SPI
#define USE_SPI_DEVICE_1
#define USE_SPI_DEVICE_2
#define USE_SPI_DEVICE_3

#define SPI1_NSS_PIN            PA4
#define SPI1_SCK_PIN            PA5
#define SPI1_MISO_PIN           PA6
#define SPI1_MOSI_PIN           PA7

#define SPI2_NSS_PIN            PB12
#define SPI2_SCK_PIN            PB13
#define SPI2_MISO_PIN           PC2
#define SPI2_MOSI_PIN           PC3

#define SPI3_NSS_PIN            PA15
#define SPI3_SCK_PIN            PB3
#define SPI3_MISO_PIN           PB4
#define SPI3_MOSI_PIN           PB5

#define USE_I2C
#define USE_I2C_PULLUP
#define USE_I2C_DEVICE_1
#define I2C_DEVICE              (I2CDEV_1)
#define I2C1_SCL                PB6
#define I2C1_SDA                PB7

#define USE_MAX7456
#define MAX7456_SPI_INSTANCE    SPI3
#define MAX7456_SPI_CS_PIN      PB12
#define MAX7456_SPI_CLK         (SPI_CLOCK_STANDARD) // 10MHz
#define MAX7456_RESTORE_CLK     (SPI_CLOCK_FAST)

#define USE_ADC
#define ADC1_DMA_STREAM         DMA2_Stream0
//#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
//#define DEFAULT_CURRENT_METER_SOURCE CURRENT_METER_ADC
#define VBAT_ADC_PIN            PC0
#define CURRENT_METER_ADC_PIN   PC1
#define RSSI_ADC_PIN            PC4

#define CURRENT_METER_OFFSET_DEFAULT 2500                      // ACS712/714-30A - 0A = 2.5V
#define CURRENT_METER_SCALE_DEFAULT -667                       // ACS712/714-30A - 66.666 mV/A inverted mode

#define BINDPLUG_PIN            PB2

#define ENABLE_BLACKBOX_LOGGING_ON_SDCARD_BY_DEFAULT

#define DEFAULT_FEATURES        (FEATURE_MOTOR_STOP)
#define DEFAULT_RX_FEATURE      FEATURE_RX_SERIAL
#define SERIALRX_UART           SERIAL_PORT_USART2
#define RX_CHANNELS_TAER

#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         0xffff
#define TARGET_IO_PORTD         (BIT(2))

#define USABLE_TIMER_CHANNEL_COUNT      13
#define USED_TIMERS             ( TIM_N(1) | TIM_N(3) | TIM_N(4) | TIM_N(5) | TIM_N(8) )
