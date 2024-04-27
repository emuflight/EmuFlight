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

#define FC_TARGET_MCU     STM32F745

#define USE_GYRO
#define USE_GYRO_SPI_MPU6000
#define USE_ACC
#define USE_ACC_SPI_MPU6000
#define USE_ACCGYRO_BMI270
#define USE_FLASH
#define USE_FLASH_W25N01G
#define USE_MAX7456

#define BOARD_NAME NBD_INFINITY200RS
#define MANUFACTURER_ID NEBD

#define MOTOR1_PIN PC8
#define MOTOR2_PIN PC6
#define MOTOR3_PIN PC9
#define MOTOR4_PIN PC7
#define LED_STRIP_PIN PA9
#define UART1_TX_PIN PB6
#define UART2_TX_PIN PA2
#define UART3_TX_PIN PB10
#define UART7_TX_PIN PE8
#define UART8_TX_PIN PE1
#define UART1_RX_PIN PB7
#define UART2_RX_PIN PA3
#define UART5_RX_PIN PD2
#define UART7_RX_PIN PE7
#define UART8_RX_PIN PE0
#define I2C1_SCL_PIN PB8
#define I2C1_SDA_PIN PB9
#define LED0_PIN PC0
#define SPI1_SCK_PIN PA5
#define SPI2_SCK_PIN PB13
#define SPI3_SCK_PIN PB3
#define SPI4_SCK_PIN PE12
#define SPI1_SDI_PIN PA6
#define SPI2_SDI_PIN PB14
#define SPI3_SDI_PIN PB4
#define SPI4_SDI_PIN PE13
#define SPI1_SDO_PIN PA7
#define SPI2_SDO_PIN PB15
#define SPI3_SDO_PIN PD6
#define SPI4_SDO_PIN PE14
#define ADC_VBAT_PIN PC1
#define ADC_CURR_PIN PC2
#define FLASH_CS_PIN PB0
#define MAX7456_SPI_CS_PIN PA15
#define GYRO_1_EXTI_PIN PB1
#define GYRO_2_EXTI_PIN PE9
#define GYRO_1_CS_PIN PE11
#define GYRO_2_CS_PIN PB12

#define TIMER_PIN_MAPPING \
    TIMER_PIN_MAP( 0, MOTOR1_PIN, 1, 0 ) \
    TIMER_PIN_MAP( 1, MOTOR2_PIN, 1, 0 ) \
    TIMER_PIN_MAP( 2, MOTOR3_PIN, 1, 0 ) \
    TIMER_PIN_MAP( 3, MOTOR4_PIN, 1, 0 ) \
    TIMER_PIN_MAP( 4, LED_STRIP_PIN, 1, 0 )

#define ADC1_DMA_OPT 0

#define VTX_SMARTAUDIO_UART SERIAL_PORT_USART2
#define ESC_SENSOR_UART SERIAL_PORT_UART5

#define DEFAULT_BLACKBOX_DEVICE BLACKBOX_DEVICE_FLASH
#define DEFAULT_DSHOT_BURST DSHOT_DMAR_ON
#define DEFAULT_CURRENT_METER_SOURCE CURRENT_METER_ADC
#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define DEFAULT_CURRENT_METER_SCALE 230
#define DEFAULT_CURRENT_METER_OFFSET 10
//TODO set small_angle = 180
#define MAX7456_SPI_INSTANCE SPI3
#define FLASH_SPI_INSTANCE SPI1
#define GYRO_1_SPI_INSTANCE SPI4
#define GYRO_1_ALIGN CW90_DEG
#define GYRO_1_ALIGN_YAW 900
#define GYRO_2_SPI_INSTANCE SPI2
