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

#define FC_TARGET_MCU     STM32F411

#define BOARD_NAME        TRANSTECF411AIO
#define MANUFACTURER_ID   TTRH

#define USE_ACC
#define USE_ACC_SPI_MPU6000
#define USE_GYRO
#define USE_GYRO_SPI_MPU6000
#define USE_MAX7456

#define BEEPER_PIN           PB2
#define MOTOR1_PIN           PB4
#define MOTOR2_PIN           PB5
#define MOTOR3_PIN           PB6
#define MOTOR4_PIN           PB7
#define MOTOR5_PIN           PB3
#define MOTOR6_PIN           PB10
#define RX_PPM_PIN           PA3
#define LED_STRIP_PIN        PA8
#define UART1_TX_PIN         PA9
#define UART2_TX_PIN         PA2
#define UART1_RX_PIN         PA10
#define UART2_RX_PIN         PA3
#define INVERTER_PIN_UART1   PA14
#define LED0_PIN             PC13
#define LED1_PIN             PC14
#define SPI1_SCK_PIN         PA5
#define SPI2_SCK_PIN         PB13
#define SPI1_SDI_PIN         PA6
#define SPI2_SDI_PIN         PB14
#define SPI1_SDO_PIN         PA7
#define SPI2_SDO_PIN         PB15
#define ADC_VBAT_PIN         PB0
#define ADC_CURR_PIN         PB1
#define MAX7456_SPI_CS_PIN   PB12
#define GYRO_1_EXTI_PIN      PA1
#define GYRO_1_CS_PIN        PA4
#define USB_DETECT_PIN       PC15

#define TIMER_PIN_MAPPING \
    TIMER_PIN_MAP( 0, PA3 , 3, -1) \
    TIMER_PIN_MAP( 1, PB4 , 1,  0) \
    TIMER_PIN_MAP( 2, PB5 , 1,  0) \
    TIMER_PIN_MAP( 3, PB6 , 1,  0) \
    TIMER_PIN_MAP( 4, PB7 , 1,  0) \
    TIMER_PIN_MAP( 5, PB3 , 1,  0) \
    TIMER_PIN_MAP( 6, PB10, 1,  0) \
    TIMER_PIN_MAP( 7, PA0 , 2,  0) \
    TIMER_PIN_MAP( 8, PA2 , 2,  0) \
    TIMER_PIN_MAP( 9, PA8 , 1,  0)


#define ADC1_DMA_OPT        1

//TODO #define DSHOT_IDLE_VALUE 600
#define DEFAULT_DSHOT_BURST DSHOT_DMAR_AUTO
#define DEFAULT_DSHOT_TELEMETRY DSHOT_TELEMETRY_ON
#define DEFAULT_DSHOT_BITBANG DSHOT_BITBANG_OFF
//TODO #define MOTOR_POLES 12
//TODO #define VBAT_MAX_CELL_VOLTAGE 450
#define DEFAULT_CURRENT_METER_SOURCE CURRENT_METER_ADC
#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define BEEPER_INVERTED
//TODO #define YAW_MOTORS_REVERSED ON
//TODO #define OSD_CURRENT_POS 2486
//TODO #define OSD_CRAFT_NAME_POS 2112
#define SYSTEM_HSE_MHZ 8
#define MAX7456_SPI_INSTANCE SPI2
#define GYRO_1_SPI_INSTANCE SPI1
