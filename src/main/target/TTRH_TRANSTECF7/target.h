/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#define TARGET_BOARD_IDENTIFIER         "S7X2"
#define USBD_PRODUCT_STRING             "TRANSTECF7"
#define TARGET_MANUFACTURER_IDENTIFIER  "TTRH"
#define USE_TARGET_CONFIG


#define ENABLE_DSHOT_DMAR               true

//Aux
#define LED0_PIN                        PA14

#define USE_PINIO
#define PINIO1_PIN                      PB12        //VTX Power Switch
#define USE_PINIOBOX

#define USE_BEEPER
// XXX CAMERA_CONTROL_PIN is deprecated.
// XXX Target maintainer must create a valid timerHardware[] array entry for PB8 with TIM_USE_CAMERA_CONTROL
//#define CAMERA_CONTROL_PIN              PB8         //Camera OSD


// *************** Gyro & ACC ***************
#define USE_ACC
#define USE_ACC_SPI_MPU6000


#define USE_EXTI
#define MPU_INT_EXTI                    PC3
#define USE_MPU_DATA_READY_SIGNAL

#define USE_GYRO
#define USE_GYRO_SPI_MPU6000
#define GYRO_MPU6000_ALIGN              CW180_DEG_FLIP
#define MPU6000_CS_PIN                  PC2
#define MPU6000_SPI_INSTANCE            SPI1


#define USE_SPI
#define USE_SPI_DEVICE_1
#define SPI1_SCK_PIN                    PA5
#define SPI1_MISO_PIN                   PA6
#define SPI1_MOSI_PIN                   PA7

// *************** I2C ***************
#define USE_I2C
#define USE_I2C_DEVICE_1
#define I2C_DEVICE                      (I2CDEV_1)
#define I2C1_SCL                        PB6         // SCL pad
#define I2C1_SDA                        PB7         // SDA pad
#define USE_I2C_PULLUP

// *************** OSD ***************
#define USE_SPI_DEVICE_2
#define SPI2_SCK_PIN                    PB13
#define SPI2_MISO_PIN                   PB14
#define SPI2_MOSI_PIN                   PB15

#define USE_MAX7456
#define MAX7456_SPI_INSTANCE            SPI2
#define MAX7456_SPI_CS_PIN              PB10
#define MAX7456_SPI_CLK                 (SPI_CLOCK_STANDARD) // 10MHz
#define MAX7456_RESTORE_CLK             (SPI_CLOCK_FAST)

// *************** UART ***************
#define USE_VCP
#define USB_DETECT_PIN                  PA4
#define USE_USB_DETECT

#define USE_UART1
#define UART1_RX_PIN                    PA10
#define UART1_TX_PIN                    PA9

#define USE_UART2
#define UART2_RX_PIN                    PA3
#define UART2_TX_PIN                    PA2

#define USE_UART3
#define UART3_RX_PIN                    PC11
#define UART3_TX_PIN                    PC10

#define USE_UART4
#define UART4_RX_PIN                    PA1
#define UART4_TX_PIN                    PA0

#define USE_UART5
#define UART5_RX_PIN                    PD2
#define UART5_TX_PIN                    PC12

#define SERIAL_PORT_COUNT               6


#define USE_OSD
#define DEFAULT_FEATURES                (FEATURE_OSD | FEATURE_SOFTSERIAL)
#define DEFAULT_RX_FEATURE              FEATURE_RX_SERIAL
#define SERIALRX_PROVIDER               SERIALRX_SBUS
#define SERIALRX_UART                   SERIAL_PORT_UART5

// *************** ADC ****************************
#define USE_ADC
#define ADC_INSTANCE                    ADC1    // Default added
#define ADC1_DMA_OPT                    0       // DMA 2 Stream 0 Channel 0

#define VBAT_ADC_PIN                    PC0
#define CURRENT_METER_ADC_PIN           PC1
#define RSSI_ADC_PIN                    PB5

#define DEFAULT_VOLTAGE_METER_SOURCE    VOLTAGE_METER_ADC
#define DEFAULT_CURRENT_METER_SOURCE    CURRENT_METER_ADC

#define USE_ESCSERIAL

#define TARGET_IO_PORTA                 0xffff
#define TARGET_IO_PORTB                 0xffff
#define TARGET_IO_PORTC                 0xffff
#define TARGET_IO_PORTD                 (BIT(2))

#define USABLE_TIMER_CHANNEL_COUNT      8
#define USED_TIMERS                     (TIM_N(2)|TIM_N(3)|TIM_N(4)|TIM_N(8))
