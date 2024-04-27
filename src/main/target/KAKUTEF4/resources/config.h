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

#define FC_TARGET_MCU     STM32F405

#define BOARD_NAME        KAKUTEF4
#define MANUFACTURER_ID   HBRO

#define USE_ACC
#define USE_ACC_SPI_MPU6000
#define USE_GYRO
#define USE_GYRO_SPI_MPU6000
#define USE_GYRO_SPI_ICM20689
#define USE_ACC_SPI_ICM20689
#define USE_BARO
#define USE_BARO_BMP280
#define USE_FLASH
#define USE_FLASH_W25Q128FV
#define USE_MAX7456

#define BEEPER_PIN           PC9
#define MOTOR1_PIN           PB0
#define MOTOR2_PIN           PB1
#define MOTOR3_PIN           PA3
#define MOTOR4_PIN           PA2
#define MOTOR5_PIN           PA0
#define MOTOR6_PIN           PC8
#define RX_PPM_PIN           PC7
#define LED_STRIP_PIN        PA1
#define UART1_TX_PIN         PA9
#define UART3_TX_PIN         PB10
#define UART6_TX_PIN         PC6
#define UART1_RX_PIN         PA10
#define UART3_RX_PIN         PB11
#define UART6_RX_PIN         PC7
#define INVERTER_PIN_UART3   PB15
#define LED0_PIN             PB5
#define LED1_PIN             PB4
#define LED2_PIN             PB6
#define SPI1_SCK_PIN         PA5
#define SPI3_SCK_PIN         PC10
#define SPI1_SDI_PIN         PA6
#define SPI3_SDI_PIN         PC11
#define SPI1_SDO_PIN         PA7
#define SPI3_SDO_PIN         PC12
#define ESCSERIAL_PIN        PC7
#define ADC_VBAT_PIN         PC3
#define ADC_RSSI_PIN         PC1
#define ADC_CURR_PIN         PC2
#define FLASH_CS_PIN         PB3
#define MAX7456_SPI_CS_PIN   PB14
#define GYRO_1_EXTI_PIN      PC5
#define GYRO_1_CS_PIN        PC4
#define USB_DETECT_PIN       PA8

#define TIMER_PIN_MAPPING \
    TIMER_PIN_MAP( 0, PC7 , 2,  0) \
    TIMER_PIN_MAP( 1, PB0 , 2,  0) \
    TIMER_PIN_MAP( 2, PB1 , 2,  0) \
    TIMER_PIN_MAP( 3, PA3 , 1,  1) \
    TIMER_PIN_MAP( 4, PA2 , 1,  0) \
    TIMER_PIN_MAP( 5, PA0 , 2,  0) \
    TIMER_PIN_MAP( 6, PC8 , 2,  1) \
    TIMER_PIN_MAP( 7, PA1 , 2,  0)


#define ADC1_DMA_OPT        0


#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define DEFAULT_CURRENT_METER_SCALE 275
#define BEEPER_INVERTED
#define SYSTEM_HSE_MHZ 8
#define MAX7456_SPI_INSTANCE SPI3
//TODO #define MAX7456_PREINIT_OPU ON
#define FLASH_SPI_INSTANCE SPI3
#define GYRO_1_SPI_INSTANCE SPI1
#define GYRO_1_ALIGN CW270_DEG
#define GYRO_1_ALIGN_YAW 2700
