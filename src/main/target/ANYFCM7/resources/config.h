/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
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

#define FC_TARGET_MCU     STM32F7X2

#define BOARD_NAME        ANYFCM7
#define MANUFACTURER_ID   FOSS

#define USE_GYRO
#define USE_GYRO_SPI_MPU6000
#define USE_ACC
#define USE_ACC_SPI_MPU6000
#define USE_BARO_MS5611
#define USE_MAX7456

#define BEEPER_PIN           PB2
#define MOTOR1_PIN           PB8
#define MOTOR2_PIN           PA2
#define MOTOR3_PIN           PB7
#define MOTOR4_PIN           PA3
#define MOTOR5_PIN           PA1
#define MOTOR6_PIN           PB0
#define MOTOR7_PIN           PB5
#define MOTOR8_PIN           PA0
#define RX_PPM_PIN           PB14
#define RX_PWM1_PIN          PB14
#define RX_PWM2_PIN          PB15
#define RX_PWM3_PIN          PC6
#define RX_PWM4_PIN          PC7
#define RX_PWM5_PIN          PC8
#define RX_PWM6_PIN          PC9
#define LED_STRIP_PIN        PB5
#define UART1_TX_PIN         PA9
#define UART4_TX_PIN         PC10
#define UART5_TX_PIN         PC12
#define UART6_TX_PIN         PC6
#define UART1_RX_PIN         PA10
#define UART4_RX_PIN         PC11
#define UART5_RX_PIN         PD2
#define UART6_RX_PIN         PC7
#define I2C2_SCL_PIN         PB10
#define I2C2_SDA_PIN         PB11
#define LED0_PIN             PB6
#define LED1_PIN             PB9
#define SPI1_SCK_PIN         PA5
#define SPI2_SCK_PIN         PB13
#define SPI3_SCK_PIN         PC10
#define SPI1_SDI_PIN         PA6
#define SPI2_SDI_PIN         PC2
#define SPI3_SDI_PIN         PC11
#define SPI1_SDO_PIN         PA7
#define SPI2_SDO_PIN         PC1
#define SPI3_SDO_PIN         PC12
#define ADC_VBAT_PIN         PC0
#define FLASH_CS_PIN         PB12
#define MAX7456_SPI_CS_PIN   PD2
#define GYRO_1_EXTI_PIN      PC4
#define GYRO_1_CS_PIN        PA4
#define USB_DETECT_PIN       PA8

#define TIMER_PIN_MAPPING \
    TIMER_PIN_MAP( 0, PB14, 3, -1) \
    TIMER_PIN_MAP( 1, PB15, 3, -1) \
    TIMER_PIN_MAP( 2, PC6 , 2,  0) \
    TIMER_PIN_MAP( 3, PC7 , 2,  1) \
    TIMER_PIN_MAP( 4, PC8 , 2,  1) \
    TIMER_PIN_MAP( 5, PC9 , 2,  0) \
    TIMER_PIN_MAP( 6, PB8 , 1,  0) \
    TIMER_PIN_MAP( 7, PA2 , 1,  0) \
    TIMER_PIN_MAP( 8, PB7 , 1,  0) \
    TIMER_PIN_MAP( 9, PA3 , 1,  1) \
    TIMER_PIN_MAP(10, PA1 , 2,  0) \
    TIMER_PIN_MAP(11, PB0 , 1,  1) \
    TIMER_PIN_MAP(12, PB5 , 1,  0) \
    TIMER_PIN_MAP(13, PA0 , 2,  0) \
    TIMER_PIN_MAP(14, PB1 , 1,  0) \
    TIMER_PIN_MAP(15, PB4 , 1,  0)



#define ADC1_DMA_OPT        1

#define USE_BARO
#define BARO_I2C_INSTANCE (I2CDEV_2)
#define DEFAULT_BLACKBOX_DEVICE     BLACKBOX_DEVICE_FLASH
#define BEEPER_INVERTED
#define MAX7456_SPI_INSTANCE SPI3
#define FLASH_SPI_INSTANCE SPI2
#define GYRO_1_SPI_INSTANCE SPI1
#define GYRO_1_ALIGN CW270_DEG
#define GYRO_1_ALIGN_YAW 2700
