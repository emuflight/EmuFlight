/*
 * This file is part of EmuFlight. It is derived from Betaflight.
 *
 * This is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#define BOARD_NAME              SPRACINGH7NANO
#define MANUFACTURER_ID         SPRO
#define TARGET_BOARD_IDENTIFIER "SP7N"
#define USBD_PRODUCT_STRING     "SPRacingH7NANO"
#define FC_TARGET_MCU           STM32H750     // not used in EmuF
#define EEPROM_SIZE             8192

// Config lives in the external QUADSPI flash (partition), same chip as blackbox.
#define CONFIG_IN_EXTERNAL_FLASH

#define USE_SPRACING_PERSISTENT_RTC_WORKAROUND
#define USE_BUTTONS
#define BUTTON_A_PIN            PE4
#define BUTTON_A_PIN_INVERTED
#define BUTTON_B_PIN            PE4
#define BUTTON_B_PIN_INVERTED

#define USE_VCP
#define USE_LED
#define LED0_PIN                PE3
#define USE_BEEPER
#define BEEPER_PIN              PD7
#define USE_LED_STRIP
#define USE_OSD
#define USE_MAX7456
#define USE_CAMERA_CONTROL
#define USE_TRANSPONDER

// QUADSPI bank 2 carries the W25N01G flash (CS PB10, software CS).
#define USE_QUADSPI
#define USE_QUADSPI_DEVICE_1
#define QUADSPI1_SCK_PIN        PB2
#define QUADSPI1_BK1_IO0_PIN    PD11
#define QUADSPI1_BK1_IO1_PIN    PD12
#define QUADSPI1_BK1_IO2_PIN    PE2
#define QUADSPI1_BK1_IO3_PIN    PD13
#define QUADSPI1_BK1_CS_PIN     NONE
#define QUADSPI1_BK2_IO0_PIN    PE7
#define QUADSPI1_BK2_IO1_PIN    PE8
#define QUADSPI1_BK2_IO2_PIN    PE9
#define QUADSPI1_BK2_IO3_PIN    PE10
#define QUADSPI1_BK2_CS_PIN     PB10
#define QUADSPI1_MODE           QUADSPI_MODE_BK2_ONLY
#define QUADSPI1_CS_FLAGS       (QUADSPI_BK1_CS_NONE | QUADSPI_BK2_CS_SOFTWARE | QUADSPI_CS_MODE_SEPARATE)
#define USE_FLASH
#define USE_FLASH_W25N01G
// Excluded (.nomk): the QUADSPI flash path does not build; flashConfig_t has no quadSpiDevice (flash.c:87, flash.c:265).
#define USE_FLASH_QUADSPI
#define FLASH_QUADSPI_INSTANCE  QUADSPI
#define USE_FLASHFS
#define ENABLE_BLACKBOX_LOGGING_ON_SPIFLASH_BY_DEFAULT

// The SD card slot is not populated; its detect pin is PD10.

#define USE_SPI
#define USE_SPI_DEVICE_2
#define SPI2_SCK_PIN            PD3
#define SPI2_MISO_PIN           PC2
#define SPI2_MOSI_PIN           PC3
#define SPI2_NSS_PIN            PB12
#define USE_SPI_DEVICE_3
#define SPI3_SCK_PIN            PB3
#define SPI3_MISO_PIN           PB4
#define SPI3_MOSI_PIN           PD6
#define SPI3_NSS_PIN            PA15
#define USE_SPI_DEVICE_4
#define SPI4_SCK_PIN            PE12
#define SPI4_MISO_PIN           PE13
#define SPI4_MOSI_PIN           PE14
#define SPI4_NSS_PIN            PE11

#define USE_I2C
#define USE_I2C_DEVICE_1
#define I2C1_SCL                PB8
#define I2C1_SDA                PB9
#define USE_I2C_DEVICE_4
#define I2C4_SCL                PB6 // shared with motor outputs 5 and 6
#define I2C4_SDA                PB7
#define MAG_I2C_INSTANCE        I2CDEV_1
#define BARO_I2C_INSTANCE       I2CDEV_1

#define USE_ACC
#define USE_ACC_SPI_MPU6500
#define USE_GYRO
#define USE_GYRO_SPI_MPU6500
#define USE_MULTI_GYRO
#define USE_DUAL_GYRO
#define USE_SPI_GYRO
#define USE_EXTI
#define USE_GYRO_EXTI
#define ACC_1_ALIGN             CW0_DEG
#define GYRO_1_ALIGN            CW0_DEG
#define ACC_2_ALIGN             CW0_DEG
#define GYRO_2_ALIGN            CW0_DEG
#define GYRO_1_CS_PIN           PB12
#define GYRO_1_EXTI_PIN         PE15
#define GYRO_1_SPI_BUS          SPIDEV_2
#define GYRO_2_CS_PIN           PA15
#define GYRO_2_EXTI_PIN         PD4
#define GYRO_2_SPI_BUS          SPIDEV_3

#define USE_BARO
#define USE_BARO_BMP388
#define USE_BARO_BMP280
#define USE_BARO_MS5611
#define USE_MAG
#define USE_MAG_HMC5883
#define USE_MAG_QMC5883

#define MAX7456_SPI_CS_PIN      PE11
#define MAX7456_SPI_INSTANCE    SPI4

#define USE_UART1
#define UART1_TX_PIN            PB14
#define UART1_RX_PIN            PB15
#define USE_UART2
#define UART2_TX_PIN            PD5
#define UART2_RX_PIN            NONE
#define USE_UART3
#define UART3_TX_PIN            PD8
#define UART3_RX_PIN            PD9
#define USE_UART4
#define UART4_TX_PIN            PD1
#define UART4_RX_PIN            PD0
#define USE_UART5
#define UART5_TX_PIN            PB13
#define UART5_RX_PIN            PB5
#define USE_UART6
#define UART6_TX_PIN            PC6 // shared with motor 7
#define UART6_RX_PIN            PC7 // shared with motor 8
#define USE_UART8
#define UART8_TX_PIN            PE1
#define UART8_RX_PIN            PE0
#define SERIAL_PORT_COUNT       8
#define SERIALRX_UART           SERIAL_PORT_USART1

#define USE_ADC
#define ADC_INSTANCE            ADC1
#define ADC1_DMA_OPT            8
#define ADC3_DMA_OPT            9
#define VBAT_ADC_PIN            PC1
#define CURRENT_METER_ADC_PIN   PC0
#define RSSI_ADC_PIN            PC4
#define EXTERNAL1_ADC_PIN       PC5
#define DEFAULT_VOLTAGE_METER_SOURCE    VOLTAGE_METER_ADC
#define DEFAULT_CURRENT_METER_SOURCE    CURRENT_METER_ADC
#define DEFAULT_VOLTAGE_METER_SCALE     110
#define DEFAULT_CURRENT_METER_SCALE     100
#define DEFAULT_FEATURES        FEATURE_RSSI_ADC

// Reference values. PA6/PA7/PB0/PB1 (spare TIM3 pads) share DMA1 Stream 0 with motor 1; PA2 and
// TIMUP8 share DMA1 Stream 2 when DShot burst is on (burst is off by default).
#define TIMER_PIN_MAPPING \
    TIMER_PIN_MAP( 0, PA8 , 1, 10) /* led strip */ \
    TIMER_PIN_MAP( 1, PB11 , 1, 11) /* transponder */ \
    TIMER_PIN_MAP( 2, PB15 , 2, -1) /* ppm / pwm 1 */ \
    TIMER_PIN_MAP( 3, PE5 , 1,  0) /* camera control */ \
    TIMER_PIN_MAP( 4, PE6 , 1, -1) \
    TIMER_PIN_MAP( 5, PA0 , 2,  0) /* motor 1 */ \
    TIMER_PIN_MAP( 6, PA1 , 2,  1) /* motor 2 */ \
    TIMER_PIN_MAP( 7, PA2 , 2,  2) /* motor 3 */ \
    TIMER_PIN_MAP( 8, PA3 , 2,  3) /* motor 4 */ \
    TIMER_PIN_MAP( 9, PB6 , 2,  4) /* motor 5 */ \
    TIMER_PIN_MAP( 10, PB7 , 2,  5) /* motor 6 */ \
    TIMER_PIN_MAP( 11, PC6 , 2,  6) /* motor 7 */ \
    TIMER_PIN_MAP( 12, PC7 , 2,  7) /* motor 8 */ \
    TIMER_PIN_MAP( 13, PD14 , 1, 12) \
    TIMER_PIN_MAP( 14, PD15 , 1, -1) \
    TIMER_PIN_MAP( 15, PA6 , 1,  0) \
    TIMER_PIN_MAP( 16, PA7 , 2,  0) \
    TIMER_PIN_MAP( 17, PB0 , 2,  0) \
    TIMER_PIN_MAP( 18, PB1 , 2,  0)

#define TIMUP1_DMA_OPT          0
#define TIMUP2_DMA_OPT          0
#define TIMUP3_DMA_OPT          0
#define TIMUP4_DMA_OPT          0
#define TIMUP5_DMA_OPT          0
#define TIMUP8_DMA_OPT          2

#define MOTOR1_PIN              PA0
#define MOTOR2_PIN              PA1
#define MOTOR3_PIN              PA2
#define MOTOR4_PIN              PA3
#define MOTOR5_PIN              PB6
#define MOTOR6_PIN              PB7
#define MOTOR7_PIN              PC6
#define MOTOR8_PIN              PC7
#define RX_PPM_PIN              PB15
#define RX_PWM1_PIN             PB15
#define LED_STRIP_PIN           PA8
#define TRANSPONDER_PIN         PB11
#define CAMERA_CONTROL_PIN      PE5
