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

#define BOARD_NAME        HAKRCF411
#define MANUFACTURER_ID   HARC

#define USE_ACC
#define USE_ACC_SPI_MPU6000
#define USE_GYRO
#define USE_GYRO_SPI_MPU6000
#define USE_ACC_SPI_ICM20689
#define USE_GYRO_SPI_ICM20689
#define USE_MAX7456

#define BEEPER_PIN           PC14
#define MOTOR1_PIN           PA8
#define MOTOR2_PIN           PB10
#define MOTOR3_PIN           PB1
#define MOTOR4_PIN           PB0
#define MOTOR5_PIN           PA15
#define MOTOR6_PIN           PB3
#define RX_PPM_PIN           PB7
#define LED_STRIP_PIN        PB6
#define UART1_TX_PIN         PA9
#define UART2_TX_PIN         PA2
#define UART1_RX_PIN         PA10
#define UART2_RX_PIN         PA3
#define INVERTER_PIN_UART1   PB4
#define I2C1_SCL_PIN         PB8
#define I2C1_SDA_PIN         PB9
#define LED0_PIN             PC13
#define SPI1_SCK_PIN         PA5
#define SPI2_SCK_PIN         PB13
#define SPI1_SDI_PIN         PA6
#define SPI2_SDI_PIN         PB14
#define SPI1_SDO_PIN         PA7
#define SPI2_SDO_PIN         PB15
#define ESCSERIAL_PIN        PA3
#define ADC_VBAT_PIN         PA0
#define ADC_CURR_PIN         PA1
#define MAX7456_SPI_CS_PIN   PB12
#define GYRO_1_EXTI_PIN      PB2
#define GYRO_1_CS_PIN        PA4

#define TIMER_PIN_MAPPING \
    TIMER_PIN_MAP( 0, PB7 , 1,  0) \
    TIMER_PIN_MAP( 1, PA8 , 1,  0) \
    TIMER_PIN_MAP( 2, PB10, 1,  0) \
    TIMER_PIN_MAP( 3, PB1 , 2,  0) \
    TIMER_PIN_MAP( 4, PB0 , 2,  0) \
    TIMER_PIN_MAP( 5, PA15, 1,  0) \
    TIMER_PIN_MAP( 6, PB3 , 1,  0) \
    TIMER_PIN_MAP( 7, PB6 , 1,  0)


#define ADC1_DMA_OPT        1

#define DEFAULT_DSHOT_BURST DSHOT_DMAR_ON
#define DEFAULT_CURRENT_METER_SOURCE CURRENT_METER_ADC
#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define BEEPER_INVERTED
#define SYSTEM_HSE_MHZ 8
#define MAX7456_SPI_INSTANCE SPI2
#define GYRO_1_SPI_INSTANCE SPI1
#define GYRO_1_ALIGN CW270_DEG
#define GYRO_1_ALIGN_YAW 2700
