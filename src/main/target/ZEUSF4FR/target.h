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


#define TARGET_BOARD_IDENTIFIER "HGLR"
#define USBD_PRODUCT_STRING "ZEUSF4FR"

#define LED0_PIN PC13

#define USE_BEEPER
#define BEEPER_PIN PC15
#define BEEPER_INVERTED

// *************** Gyro & ACC **********************
#define USE_SPI
#define USE_SPI_DEVICE_1

#define SPI1_SCK_PIN PA5
#define SPI1_MISO_PIN PA6
#define SPI1_MOSI_PIN PA7

#define MPU6000_CS_PIN PA4
//#define ICM20689_CS_PIN PA4
#define MPU6000_SPI_INSTANCE SPI1
//#define ICM20689_SPI_INSTANCE SPI1

#define USE_EXTI
#define USE_GYRO_EXTI
#define MPU_INT_EXTI PA1
#define USE_MPU_DATA_READY_SIGNAL
#define ENSURE_MPU_DATA_READY_IS_LOW

#define USE_GYRO
#define USE_GYRO_SPI_MPU6000
#define GYRO_MPU6000_ALIGN CW180_DEG
//#define USE_GYRO_SPI_ICM20689
//#define ACC_ICM20689_ALIGN CW180_DEG
#define USE_ACC
#define USE_ACC_SPI_MPU6000
#define ACC_MPU6000_ALIGN CW180_DEG
//#define USE_ACC_SPI_ICM20689
//#define ACC_ICM20689_ALIGN CW180_DEG

// *************** Baro **************************
//#define USE_I2C

//#define USE_I2C_DEVICE_1
//#define I2C_DEVICE (I2CDEV_1)
//#define I2C1_SCL PB8 // SCL pad
//#define I2C1_SDA PB9 // SDA pad
//#define BARO_I2C_INSTANCE (I2CDEV_1)
#define BARO_SPI_INSTANCE   SPI1

#define USE_BARO //External, connect to I2C1
#define BARO_CS_PIN A13
#define USE_BARO_BMP280


//#define USE_MAG
//#define USE_MAG_HMC5883 //External, connect to I2C1
//#define USE_MAG_QMC5883

// *************** UART *****************************
#define USE_VCP

#define USE_UART1
#define UART1_RX_PIN PA10
#define UART1_TX_PIN PA9

#define USE_UART2
#define UART2_RX_PIN PA3
#define UART2_TX_PIN PA2

#define USE_SOFTSERIAL1

#define SERIAL_PORT_COUNT 4

// *************** SPI3 CC2500 ***************************

#define USE_SPI_DEVICE_3
#define SPI3_SCK_PIN            PB3
#define SPI3_MISO_PIN           PB4
#define SPI3_MOSI_PIN           PB5
#define SPI3_NSS_PIN            PA15 //RX_SPI_CS
#define RX_NSS_PIN              SPI3_NSS_PIN

#define RX_NSS_GPIO_CLK_PERIPHERAL   RCC_APB2Periph_GPIOA

#define USE_RX_SPI
#define RX_SPI_INSTANCE         SPI3

#define RX_CC2500_SPI_DISABLE_CHIP_DETECTION
#define RX_CC2500_SPI_GDO_0_PIN     PC14 //RX_SPI_EXTI
#define RX_CC2500_SPI_LED_PIN       PB9 //RX_SPI_led

#define BINDPLUG_PIN            PB2 //RX_SPI_BIND

#define USE_RX_FRSKY_SPI_D
#define USE_RX_FRSKY_SPI_X
#define USE_RX_REDPINE_SPI
#define DEFAULT_RX_FEATURE      FEATURE_RX_SPI
#define RX_SPI_DEFAULT_PROTOCOL RX_SPI_FRSKY_D
#define USE_RX_FRSKY_SPI_TELEMETRY

// *************** OSD/FLASH *****************************
#define USE_SPI_DEVICE_2
#define SPI2_SCK_PIN            PB13
#define SPI2_MISO_PIN           PB14
#define SPI2_MOSI_PIN           PB15

#define USE_OSD
#define USE_MAX7456
#define MAX7456_SPI_INSTANCE    SPI2
#define MAX7456_SPI_CS_PIN      PB12

#define ENABLE_BLACKBOX_LOGGING_ON_SPIFLASH_BY_DEFAULT
#define USE_FLASHFS
#define USE_FLASH_M25P16
#define FLASH_CS_PIN            PA8
#define FLASH_SPI_INSTANCE      SPI2

// *************** ADC *****************************
#define USE_ADC
#define ADC_INSTANCE         ADC1
#define ADC1_DMA_OPT            0  // DMA 2 Stream 0 Channel 0

#define VBAT_ADC_PIN            PB0
#define CURRENT_METER_ADC_PIN   PB1
//#define RSSI_ADC_PIN            PB1
//#define EXTERNAL1_ADC_PIN       PA4

#define USE_ESCSERIAL

#define USE_LED_STRIP

//#define ENABLE_DSHOT_DMAR       true
//#define USE_PINIO
//#define PINIO1_PIN              PB5  // VTX  switcher
//#define PINIO2_PIN              PA15 // Camera switcher
//#define USE_PINIOBOX

#define DEFAULT_FEATURES                (FEATURE_OSD | FEATURE_TELEMETRY | FEATURE_SOFTSERIAL)
#define CURRENT_METER_SCALE_DEFAULT         179
#define DEFAULT_VOLTAGE_METER_SOURCE    VOLTAGE_METER_ADC
#define DEFAULT_CURRENT_METER_SOURCE    CURRENT_METER_ADC

#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         0xffff
#define TARGET_IO_PORTD         (BIT(2))

#define USABLE_TIMER_CHANNEL_COUNT 9
#define USED_TIMERS             ( TIM_N(1)|TIM_N(2)|TIM_N(4)|TIM_N(5)|TIM_N(9) )
