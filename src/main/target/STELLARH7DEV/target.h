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

#pragma once

#define BOARD_NAME                      STELLARH7DEV
#define MANUFACTURER_ID                 STBE
#define TARGET_BOARD_IDENTIFIER "SH74"
#define FC_TARGET_MCU                   STM32H743

#define USE_BARO
#define USE_BARO_DPS310
#define USE_ACC
#define USE_GYRO
#define USE_GYRO_SPI_ICM42688P
#define USE_ACC_SPI_ICM42688P
#define USE_MAX7456

#define USE_VCP
#define USE_OSD
#define USB_DETECT_PIN                  PA12

#define USE_LED
#define LED1_PIN                        PB1
#define LED_STRIP_PIN                   PA15
#define USE_BEEPER
#define BEEPER_PIN                      PA3
#define USE_USB_DETECT

#define USE_SPI
#define USE_SPI_DEVICE_1
#define SPI1_SCK_PIN                    PA5
#define SPI1_MISO_PIN                   PA6
#define SPI1_MOSI_PIN                   PA7
#define USE_SPI_DEVICE_2
#define SPI2_SCK_PIN                    PB13
#define SPI2_MISO_PIN                   PC2
#define SPI2_MOSI_PIN                   PC1
#define USE_SPI_DEVICE_3
#define SPI3_SCK_PIN                    PC10
#define SPI3_MISO_PIN                   PC11
#define SPI3_MOSI_PIN                   PC12
#define USE_SPI_DEVICE_4
#define SPI4_SCK_PIN                    PE2
#define SPI4_MISO_PIN                   PE5
#define SPI4_MOSI_PIN                   PE6

#define USE_SPI_GYRO
#define USE_EXTI
#define USE_GYRO_EXTI

#define MPU_INT_EXTI                    PA8

#define ACC_ICM42688P_ALIGN             CW180_DEG
#define GYRO_ICM42688P_ALIGN            CW180_DEG
#define ICM42688P_CS_PIN                PE3
#define ICM42688P_SPI_BUS               SPIDEV_3

#define USE_UART3
#define UART3_TX_PIN                    PD8
#define UART3_RX_PIN                    PD9
#define USE_UART4
#define UART4_TX_PIN                    PA0
#define UART4_RX_PIN                    PA1
#define USE_UART5
#define UART5_TX_PIN                    PB6
#define UART5_RX_PIN                    PB5
#define USE_UART7
#define UART7_TX_PIN                    PE8
#define UART7_RX_PIN                    PE7
#define USE_UART8
#define UART8_TX_PIN                    PE1
#define UART8_RX_PIN                    PE0
#define MSP_DISPLAYPORT_UART            SERIAL_PORT_USART7
#define ESC_SENSOR_UART                 SERIAL_PORT_USART8
#define SERIALRX_UART                   SERIAL_PORT_UART4
#define SERIAL_PORT_COUNT               6

#define USE_I2C
#define USE_I2C_DEVICE_2
#define I2C_DEVICE_2                    (I2CDEV_2)
#define BARO_I2C_INSTANCE               I2CDEV_2
#define MAG_I2C_INSTANCE                I2CDEV_2
#define I2C2_SCL                        PB10
#define I2C2_SDA                        PB11

// Flash chip is Winbond W25N01G (JEDEC 0xEFAA21) — NAND, not NOR.
// No EmuFlight NAND flash driver exists yet; blackbox to SPI flash unsupported.

#define MAX7456_SPI_CS_PIN              PB8
#define MAX7456_SPI_INSTANCE            SPI2

#define USE_ADC
#define VBAT_ADC_PIN                    PC5
#define CURRENT_METER_ADC_PIN           PB0
#define DEFAULT_VOLTAGE_METER_SOURCE    VOLTAGE_METER_ADC
#define DEFAULT_VOLTAGE_METER_SCALE     210
#define DEFAULT_CURRENT_METER_SOURCE    CURRENT_METER_ADC
#define DEFAULT_CURRENT_METER_SCALE     120

// PB3: VTX/DJI connector power enable (active-high).
// PINIO1_CONFIG = 0x81: push-pull, inverted — USER1 inactive → PB3 HIGH → VTX ON.
// USER1 active (box 40) drives PB3 LOW → VTX pit mode / power cut.
#define PINIO1_PIN                      PB3
#define PINIO1_CONFIG                   129
#define PINIO1_BOX                      40

#define TARGET_IO_PORTA                 0xffff
#define TARGET_IO_PORTB                 0xffff
#define TARGET_IO_PORTC                 0xffff
#define TARGET_IO_PORTD                 0xffff
#define TARGET_IO_PORTE                 0xffff

#define DEFAULT_FEATURES                (FEATURE_OSD | FEATURE_TELEMETRY | FEATURE_AIRMODE | FEATURE_RX_SERIAL)
#define DEFAULT_RX_FEATURE              FEATURE_RX_SERIAL

#define USABLE_TIMER_CHANNEL_COUNT      8
#define USED_TIMERS                     ( TIM_N(2) | TIM_N(3) | TIM_N(4) | TIM_N(8) )
