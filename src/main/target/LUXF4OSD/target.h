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

#define TARGET_BOARD_IDENTIFIER "LUX4"

#define USBD_PRODUCT_STRING "LuxF4osd"

#define LED0_PIN                PB5
#define USE_BEEPER
#define BEEPER_PIN              PB4
#define BEEPER_INVERTED

#define INVERTER_PIN_UART1      PC0 // DYS F4 Pro; Omnibus F4 AIO (1st gen) have a FIXED inverter on UART1

#define USE_ACC
#define USE_ACC_SPI_MPU6000

#define USE_GYRO
#define USE_GYRO_SPI_MPU6000

#define MPU6000_CS_PIN          PA4
#define MPU6000_SPI_BUS    SPIDEV_1

// MPU6000 interrupts
#define USE_EXTI
#define MPU_INT_EXTI            PC4
#define USE_MPU_DATA_READY_SIGNAL

#define GYRO_MPU6000_ALIGN       CW180_DEG
#define ACC_MPU6000_ALIGN        CW180_DEG

#define USE_MAG
#define USE_MAG_HMC5883
#define USE_MAG_QMC5883
#define MAG_HMC5883_ALIGN       CW90_DEG

#define USE_BARO
#define USE_BARO_BMP085
#define USE_BARO_BMP280
#define USE_BARO_MS5611
#define BARO_I2C_INSTANCE       (I2CDEV_2)

#define DEFAULT_BARO_BMP280

#define USE_MAX7456
#define MAX7456_SPI_INSTANCE    SPI3
#define MAX7456_SPI_CS_PIN      PA15
#define MAX7456_SPI_CLK         (SPI_CLOCK_STANDARD) // 10MHz
#define MAX7456_RESTORE_CLK     (SPI_CLOCK_FAST)

// Globally configure flashfs and drivers for various flash chips
#define USE_FLASHFS
#define USE_FLASH_M25P16
#define USE_FLASH_W25M512

#define ENABLE_BLACKBOX_LOGGING_ON_SPIFLASH_BY_DEFAULT
#define FLASH_CS_PIN            PB12
#define FLASH_SPI_INSTANCE      SPI2
#define USE_FLASHFS
#define USE_FLASH_M25P16

#define USE_VCP
#define USE_USB_DETECT
#define USB_DETECT_PIN   PC5

#define USE_UART1
#define UART1_RX_PIN            PA10
#define UART1_TX_PIN            PA9

#define USE_UART3
#define UART3_RX_PIN            PB11
#define UART3_TX_PIN            PB10

#define USE_UART6
#define UART6_RX_PIN            PC7
#define UART6_TX_PIN            PC6

#define USE_SOFTSERIAL1
#define USE_SOFTSERIAL2

#define SERIAL_PORT_COUNT       6 // VCP, USART1, USART3, USART6, SOFTSERIAL x 2

#define ESCSERIAL_TIMER_TX_PIN  PB14 // (Hardware=0)

#define USE_SPI
#define USE_SPI_DEVICE_1

#define USE_SPI_DEVICE_2
#define SPI2_NSS_PIN            PB12
#define SPI2_SCK_PIN            PB13
#define SPI2_MISO_PIN           PB14
#define SPI2_MOSI_PIN           PB15

#define USE_SPI_DEVICE_3
#define SPI3_NSS_PIN          PB3
#define SPI3_SCK_PIN            PC10
#define SPI3_MISO_PIN           PC11
#define SPI3_MOSI_PIN           PC12

#define USE_I2C
#define USE_I2C_DEVICE_2
#define I2C2_SCL                NONE // PB10, shared with UART3TX
#define I2C2_SDA                NONE // PB11, shared with UART3RX
#define I2C_DEVICE              (I2CDEV_2)

#define USE_ADC
#define ADC_INSTANCE            ADC2
//#define ADC_INSTANCE            ADC1

#define CURRENT_METER_ADC_PIN   PC1  // Direct from CRNT pad (part of onboard sensor for Pro)
#define VBAT_ADC_PIN            PC2  // 11:1 (10K + 1K) divider
#define RSSI_ADC_PIN            PA0  // Direct from RSSI pad

#define USE_TRANSPONDER

#define USE_RANGEFINDER
#define USE_RANGEFINDER_HCSR04
#define RANGEFINDER_HCSR04_TRIGGER_PIN     PA1
#define RANGEFINDER_HCSR04_ECHO_PIN        PA8
#define USE_RANGEFINDER_TF

#define DEFAULT_FEATURES        (FEATURE_OSD)

#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define DEFAULT_CURRENT_METER_SOURCE CURRENT_METER_ADC


#define USABLE_TIMER_CHANNEL_COUNT 14
#define USE_TIMER_MGMT
#define TIMER_PIN_MAPPING \
    TIMER_PIN_MAP( 0, PB14 , 3,  0) \
    TIMER_PIN_MAP( 1, PB15 , 3,  0) \
    TIMER_PIN_MAP( 2, PC6 , 2,  0) \
    TIMER_PIN_MAP( 3, PC7 , 2,  0) \
    TIMER_PIN_MAP( 4, PC8 , 2,  0) \
    TIMER_PIN_MAP( 5, PC9 , 2,  0) \
    TIMER_PIN_MAP( 6, PB0 , 2,  0) \
    TIMER_PIN_MAP( 7, PB1 , 2,  0) \
    TIMER_PIN_MAP( 8, PA3 , 1,  1) \
    TIMER_PIN_MAP( 9, PA2 , 1,  0) \
    TIMER_PIN_MAP( 10, PA1 , 2,  0) \
    TIMER_PIN_MAP( 11, PA8 , 1,  0) \
    TIMER_PIN_MAP( 12, PA9 , 1,  0) \
    TIMER_PIN_MAP( 13, PA10 , 1,  0)

#define MOTOR1_PIN              PB0
#define MOTOR2_PIN              PB1
#define MOTOR3_PIN              PA3
#define MOTOR4_PIN              PA2
#define MOTOR5_PIN              PA1
#define MOTOR6_PIN              PA8
#define RX_PWM1_PIN             PB14
#define RX_PWM2_PIN             PB15
#define RX_PWM3_PIN             PC6
#define RX_PWM4_PIN             PC7
#define RX_PWM5_PIN             PC8
#define RX_PWM6_PIN             PC9
#define RX_PPM_PIN              PB14
#define LED_STRIP_PIN           PA1
