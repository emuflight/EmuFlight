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

// This resource file generated using https://github.com/nerdCopter/target-convert
// Commit: d7caf4a

#pragma once

#define BOARD_NAME              SPRACINGH7EF
#define MANUFACTURER_ID         SPRO
#define TARGET_BOARD_IDENTIFIER "S730"  // generic ID
#define FC_TARGET_MCU           STM32H730     // not used in EmuF

// Config storage: OctoSPI flash is configured by bootloader (memory-mapped mode);
// firmware runs from OctoSPI but does not configure it. CONFIG_IN_EXTERNAL_FLASH
// requires flash driver + config_streamer work not yet implemented in EF.
#define CONFIG_IN_RAM

#define USE_SPRACING_PERSISTENT_RTC_WORKAROUND
#define USE_BUTTONS
#define USE_SPI
#define USE_SPI_DEVICE_2
#define USE_SPI_DEVICE_3
// SPI6 is on D3 domain (requires BDMA, not yet implemented in EF)
//#define USE_SPI_DEVICE_6
#define USE_I2C
#define USE_I2C_DEVICE_1
#define I2C1_SCL                PB6
#define I2C1_SDA                PB7
#define USE_I2C_DEVICE_4
#define I2C4_SCL                PD12  // J8:5
#define I2C4_SDA                PD13  // J8:6
#define USE_PINIO
#define USE_PINIOBOX
#define USE_ACC
#define USE_ACC_SPI_ICM42605
#define USE_ACC_SPI_ICM42688P
#define USE_GYRO
#define USE_GYRO_SPI_ICM42605
#define USE_GYRO_SPI_ICM42688P
#define USE_MULTI_GYRO
#define USE_BARO
#define USE_BARO_BMP388
#define USE_MAG
#define USE_MAG_HMC5883
#define USE_MAG_QMC5883
#define USE_LED_STRIP
#define USE_OSD

#define USE_VCP

#define USE_LED
#define LED0_PIN             PE5
#define LED1_PIN             PE6
#define LED2_PIN             NONE
#define LED_STRIP_PIN        PB8
#define USE_BEEPER
#define BEEPER_PIN           PD11
#define BEEPER_INVERTED

#define SPI2_SCK_PIN            PB13
#define SPI2_MISO_PIN           PB14
#define SPI2_MOSI_PIN           PB15
#define SPI2_NSS_PIN            PB12
#define SPI3_SCK_PIN            PC10
#define SPI3_MISO_PIN           PC11
#define SPI3_MOSI_PIN           PC12
#define SPI3_NSS_PIN            PA15
// SPI6 pins disabled (D3/BDMA not supported yet)
//#define SPI6_SCK_PIN          PB3
//#define SPI6_MISO_PIN         PB4
//#define SPI6_MOSI_PIN         PB5

#define USE_SPI_GYRO
#define USE_EXTI
#define USE_GYRO_EXTI

#define ACC_1_ALIGN          CW0_DEG
#define GYRO_1_ALIGN         CW0_DEG
#define GYRO_1_CS_PIN           SPI3_NSS_PIN
#define GYRO_1_EXTI_PIN         PC6 // TIM8 CH1
#define GYRO_1_SPI_BUS     SPIDEV_3

#define USE_DUAL_GYRO

#define ACC_2_ALIGN          CW180_DEG
#define GYRO_2_ALIGN         CW180_DEG
#define GYRO_2_CS_PIN           SPI2_NSS_PIN
#define GYRO_2_EXTI_PIN         PC8 // TIM8 CH3
#define GYRO_2_SPI_BUS     SPIDEV_2

#define USE_UART1
#define UART1_TX_PIN         PB6
#define UART1_RX_PIN         PB7
#define USE_UART2
#define UART2_TX_PIN         PD5
#define UART2_RX_PIN         PD6
#define USE_UART3
#define UART3_TX_PIN         PD8
#define UART3_RX_PIN         PD9
#define USE_UART4
#define UART4_TX_PIN         PD1
#define UART4_RX_PIN         PD0
#define USE_UART5
#define UART5_TX_PIN         NONE
#define UART5_RX_PIN         PD2
#define USE_UART6
#define UART6_TX_PIN         NONE
#define UART6_RX_PIN         NONE
#define USE_UART7
#define UART7_TX_PIN         NONE
#define UART7_RX_PIN         NONE
#define USE_UART8
#define UART8_TX_PIN         PE1
#define UART8_RX_PIN         PE0
#define USE_UART9
#define UART9_TX_PIN         PD15
#define UART9_RX_PIN         PD14
#define USE_UART10
#define UART10_TX_PIN        PE3
#define UART10_RX_PIN        PE2
#define SERIAL_PORT_COUNT    11

// RX_SPI_CS_PIN shares PB12 with GYRO_2_CS_PIN (SPI2_NSS_PIN) — BF parity.
// Hardware design: USE_RX_SPI and dual-gyro are mutually exclusive on this board.
#define RX_SPI_CS_PIN        PB12
#define RX_SPI_BIND_PIN      NONE
#define RX_SPI_LED_PIN       NONE
#define RX_SPI_EXPRESSLRS_RESET_PIN PD10
#define RX_SPI_EXPRESSLRS_BUSY_PIN  PC7
#define BINDPLUG_PIN         NONE

#define TARGET_IO_PORTA                 0xffff
#define TARGET_IO_PORTB                 0xffff
#define TARGET_IO_PORTC                 0xffff
#define TARGET_IO_PORTD                 0xffff
#define TARGET_IO_PORTE                 0xffff

#define USABLE_TIMER_CHANNEL_COUNT      10
#define USED_TIMERS                     ( TIM_N(3) | TIM_N(5) | TIM_N(16) | TIM_N(17) )
