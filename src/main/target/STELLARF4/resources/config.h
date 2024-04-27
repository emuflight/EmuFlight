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

#define FC_TARGET_MCU STM32F405
#define BOARD_NAME STELLARF4
#define MANUFACTURER_ID FLMO

#define USE_ACC
#define USE_BARO
#define USE_FLASH
#define USE_GYRO

#define USE_ACCGYRO_BMI270
#define USE_BARO_BMP280
#define USE_BARO_DPS310
#define USE_FLASH_M25P16
#define USE_MAX7456

#define BEEPER_PIN PC5
#define MOTOR1_PIN PB6
#define MOTOR2_PIN PB7
#define MOTOR3_PIN PB8
#define MOTOR4_PIN PB9
#define MOTOR5_PIN PB0
#define MOTOR6_PIN PB1
#define MOTOR7_PIN PB5
#define MOTOR8_PIN PB4
#define SERVO1_PIN PA8
#define SERVO2_PIN PC9
#define RX_PPM_PIN PA3
#define UART1_TX_PIN PA9
#define UART2_TX_PIN PA2
#define UART3_TX_PIN PC10
#define UART4_TX_PIN PA0
#define UART5_TX_PIN PC12
#define UART6_TX_PIN PC6
#define UART1_RX_PIN PA10
#define UART2_RX_PIN PA3
#define UART3_RX_PIN PC11
#define UART4_RX_PIN PA1
#define UART5_RX_PIN PD2
#define UART6_RX_PIN PC7
#define I2C2_SCL_PIN PB10
#define I2C2_SDA_PIN PB11
#define LED0_PIN PC8
#define SPI1_SCK_PIN PA5
#define SPI2_SCK_PIN PB13
#define SPI1_SDI_PIN PA6
#define SPI2_SDI_PIN PB14
#define SPI1_SDO_PIN PA7
#define SPI2_SDO_PIN PB15
#define ADC_VBAT_PIN PC0
#define ADC_RSSI_PIN PC2
#define ADC_CURR_PIN PC1
#define FLASH_CS_PIN PA15
#define PINIO1_PIN PC3
#define MAX7456_SPI_CS_PIN PB12
#define GYRO_1_EXTI_PIN PC4
#define GYRO_1_CS_PIN PA4

#define TIMER_PIN_MAPPING   TIMER_PIN_MAP( 0,  PB6, 1,  0 ) \
                            TIMER_PIN_MAP( 1,  PB7, 1,  0 ) \
                            TIMER_PIN_MAP( 2,  PB8, 1,  0 ) \
                            TIMER_PIN_MAP( 3,  PB9, 1, -1 ) \
                            TIMER_PIN_MAP( 4,  PB0, 2,  0 ) \
                            TIMER_PIN_MAP( 5,  PB1, 2,  0 ) \
                            TIMER_PIN_MAP( 6,  PB5, 1,  0 ) \
                            TIMER_PIN_MAP( 7,  PB4, 1,  0 ) \
                            TIMER_PIN_MAP( 8,  PA8, 1,  0 ) \
                            TIMER_PIN_MAP( 9,  PA3, 2,  0 ) \
                            TIMER_PIN_MAP( 10, PB3, 1,  0 ) \
                            TIMER_PIN_MAP( 11, PC9, 2,  0 )

#define ADC1_DMA_OPT 0
//TODO dma SPI_MOSI 2 0
#define SPI3_TX_DMA_OPT 0

//TODO feature RX_SERIAL
//TODO feature TELEMETRY
//TODO feature OSD

#define SERIALRX_UART SERIAL_PORT_USART2

//TODO beacon RX_LOST
//TODO beacon RX_SET

#define MAG_I2C_INSTANCE (I2CDEV_2)
#define BARO_I2C_INSTANCE (I2CDEV_2)
#define DASHBOARD_I2C_INSTANCE (I2CDEV_2)

#define DEFAULT_DSHOT_BURST DSHOT_DMAR_ON
//TODO #define MOTOR_PWM_PROTOCOL DSHOT300
#define DEFAULT_CURRENT_METER_SOURCE CURRENT_METER_ADC
#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define DEFAULT_VOLTAGE_METER_SCALE 110
#define DEFAULT_CURRENT_METER_SCALE 386
#define BEEPER_INVERTED
#define DEFAULT_BLACKBOX_DEVICE BLACKBOX_DEVICE_FLASH
#define FLASH_SPI_INSTANCE SPI2
#define MAX7456_SPI_INSTANCE SPI2
//TODO set osd_displayport_device = MAX7456
#define PINIO1_BOX 40
#define GYRO_1_SPI_INSTANCE SPI1
//TODO set box_user_1_name = CAMERA SWITCH
