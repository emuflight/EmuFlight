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

#define BOARD_NAME        NBD_CRICKETF7V2
#define MANUFACTURER_ID   NEBD

#define USE_ACC
#define USE_GYRO
#define USE_ACCGYRO_BMI270

#define BEEPER_PIN           PC15
#define MOTOR1_PIN           PB4
#define MOTOR2_PIN           PB1
#define MOTOR3_PIN           PB3
#define MOTOR4_PIN           PB0
#define LED_STRIP_PIN        PA8
#define UART2_TX_PIN         PA2
#define UART3_TX_PIN         PB10
#define UART4_TX_PIN         PA0
#define UART6_TX_PIN         PC6
#define UART1_RX_PIN         PA10
#define UART2_RX_PIN         PA3
#define UART3_RX_PIN         PB11
#define UART4_RX_PIN         PA1
#define UART5_RX_PIN         PD2
#define UART6_RX_PIN         PC7
#define LED0_PIN             PA15
#define SPI1_SCK_PIN         PA5
#define SPI2_SCK_PIN         PB13
#define SPI3_SCK_PIN         PC10
#define SPI1_SDI_PIN         PA6
#define SPI2_SDI_PIN         PB14
#define SPI3_SDI_PIN         PC11
#define SPI1_SDO_PIN         PA7
#define SPI2_SDO_PIN         PB15
#define SPI3_SDO_PIN         PC12
#define ADC_VBAT_PIN         PC0
#define ADC_CURR_PIN         PC1
#define MAX7456_SPI_CS_PIN   PC13
#define GYRO_1_EXTI_PIN      PC4
#define GYRO_2_EXTI_PIN      PC9
#define GYRO_1_CS_PIN        PB12
#define GYRO_2_CS_PIN        PA4

#define TIMER_PIN_MAPPING \
    TIMER_PIN_MAP( 0, PB4 , 1,  0) \
    TIMER_PIN_MAP( 1, PB1 , 2,  0) \
    TIMER_PIN_MAP( 2, PB3 , 1,  0) \
    TIMER_PIN_MAP( 3, PB0 , 2,  0) \
    TIMER_PIN_MAP( 4, PA8 , 1,  0)


#define ADC1_DMA_OPT        1

#define DEFAULT_GYRO_TO_USE GYRO_CONFIG_USE_GYRO_BOTH

#define DEFAULT_CURRENT_METER_SOURCE CURRENT_METER_ADC
#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define BEEPER_INVERTED
#define MAX7456_SPI_INSTANCE SPI3
#define GYRO_1_SPI_INSTANCE SPI2
#define GYRO_1_ALIGN CW270_DEG
#define GYRO_1_ALIGN_YAW 2700
#define GYRO_2_SPI_INSTANCE SPI1
#define GYRO_2_ALIGN_YAW 0
