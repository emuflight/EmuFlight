/*
 * This file is part of EmuFlight. It is derived from Betaflight.
 *
 * This is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 * You should have received a copy of the GNU General Public
 * License along with this software.
 * If not, see <http://www.gnu.org/licenses/>.
 */

// Synthetic CI target — STM32F446 family (gap coverage; only 1 BF board).
// GPIO pins copied from NUCLEOF446RE (valid for F446; not a real product).
// .hex output is NOT flashable. See make/targets.mk UNSUPPORTED_TARGETS.
// IMU selection exercises unique F446 peripheral paths.

#pragma once

#define TARGET_MANUFACTURER_IDENTIFIER "CIST"
#define USBD_PRODUCT_STRING "CI_STM32F446"

#define FC_TARGET_MCU     STM32F446     // not used in EmuF
#define TARGET_BOARD_IDENTIFIER "CF446"  // generic ID

#define USE_GYRO
#define USE_ACC
// MPU6500 — covers MPU6500 + MPU9250 code path
#define USE_GYRO_SPI_MPU6500
#define USE_ACC_SPI_MPU6500
// BMI270 — gap/modern IMU coverage
#define USE_ACCGYRO_BMI270
#define USE_BARO
#define USE_BARO_BMP280

#define USE_VCP
#define USE_OSD

#define USE_LED
#define LED0_PIN             PB7
#define USE_BEEPER
#define BEEPER_PIN           PB6

#define USE_SPI
#define USE_SPI_DEVICE_2
#define SPI2_SCK_PIN         PB13
#define SPI2_MISO_PIN        PB14
#define SPI2_MOSI_PIN        PB15

#define USE_EXTI
#define MPU_INT_EXTI         PB11
#define USE_MPU_DATA_READY_SIGNAL

#define GYRO_1_ALIGN         CW0_DEG
#define ACC_1_ALIGN          CW0_DEG
#define GYRO_1_CS_PIN        PB12
#define GYRO_1_EXTI_PIN      PB11
#define GYRO_1_SPI_BUS  SPIDEV_2

#define USE_SPI_GYRO

// Both IMU drivers share CS/bus — compile coverage only
#define MPU6500_CS_PIN        PB12
#define MPU6500_SPI_BUS  SPIDEV_2
#define BMI270_CS_PIN         PB12
#define BMI270_SPI_BUS   SPIDEV_2

#define BARO_CS_PIN           PB10

#define USE_UART1
#define UART1_TX_PIN         PA9
#define UART1_RX_PIN         PA10
#define USE_UART2
#define UART2_TX_PIN         PA2
#define UART2_RX_PIN         PA3

#define SERIAL_PORT_COUNT    3 // VCP + UART1 + UART2

#define USE_I2C
#define USE_I2C_DEVICE_1
#define I2C_DEVICE           (I2CDEV_1)
#define I2C1_SCL             PB8
#define I2C1_SDA             PB9

#define USE_ADC
#define ADC_INSTANCE         ADC1
#define VBAT_ADC_PIN         PC0

#define DEFAULT_RX_FEATURE   FEATURE_RX_SERIAL
#define SERIALRX_PROVIDER    SERIALRX_SBUS

#define TARGET_IO_PORTA 0xffff
#define TARGET_IO_PORTB 0xffff
#define TARGET_IO_PORTC 0xffff

#define USABLE_TIMER_CHANNEL_COUNT 6
#define USED_TIMERS (TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(4))
