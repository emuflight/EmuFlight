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

#define TARGET_BOARD_IDENTIFIER         "BBPRO" // BeeBrain Pro
#define USBD_PRODUCT_STRING             "BeeBrain PRO"

#define USE_TARGET_CONFIG
#define TARGET_PREINIT

#define BRUSHED_MOTORS

// *************** SPI *****************************
#define USE_SPI
#define USE_SPI_DEVICE_2
#define USE_SPI_DEVICE_3

#define SPI2_NSS_PIN                    PB12
#define SPI2_SCK_PIN                    PB13
#define SPI2_MISO_PIN                   PB14
#define SPI2_MOSI_PIN                   PB15

#define SPI3_NSS_PIN                    PA15
#define SPI3_SCK_PIN                    PB3
#define SPI3_MISO_PIN                   PB4
#define SPI3_MOSI_PIN                   PB5

// *************** UART *****************************
#define USE_VCP
#define USE_UART1
#define USE_UART2

#define UART1_TX_PIN                    PA9
#define UART1_RX_PIN                    PA10

#define UART2_TX_PIN                    PA2
#define UART2_RX_PIN                    PA3

#define SERIAL_PORT_COUNT               3
#define USE_MSP_UART

// *************** Gyro & ACC **********************
#define GYRO
#define ACC
#define USE_GYRO_SPI_MPU6000
#define USE_ACC_SPI_MPU6000

#define MPU6000_CS_PIN                  PA4
#define MPU6000_SPI_INSTANCE            SPI3

#define USE_EXTI
#define MPU_INT_EXTI                    PB0
#define USE_MPU_DATA_READY_SIGNAL

#define GYRO_MPU6000_ALIGN              CW90_DEG
#define ACC_MPU6000_ALIGN               CW90_DEG

// *************** RX ******************************
#if (defined(BEEBRAIN_PRO_DSM_US) || defined(BEEBRAIN_PRO_DSM_INTL))
#define SERIALRX_PROVIDER           SERIALRX_SPEKTRUM2048
#undef  USE_SPEKTRUM_REAL_RSSI
#undef  USE_SPEKTRUM_FAKE_RSSI
#define DEFAULT_RX_FEATURE          FEATURE_RX_SERIAL
#define SERIALRX_UART               SERIAL_PORT_USART2
#define RX_CHANNELS_TAER
#else
#define USE_RX_SPI
#define RX_SPI_INSTANCE             SPI2
#define RX_SCK_PIN                  SPI2_SCK_PIN
#define RX_MISO_PIN                 SPI2_MISO_PIN
#define RX_MOSI_PIN                 SPI2_MOSI_PIN
#define RX_NSS_PIN                  SPI2_NSS_PIN
#define RX_CC2500_SPI_GDO_0_PIN      PB2
#define RX_CC2500_SPI_LED_PIN        PA13
#define RX_CC2500_SPI_LED_PIN_INVERTED
#define RX_CC2500_SPI_TX_EN_PIN      PB10
#define RX_CC2500_SPI_ANT_SEL_PIN    PA7
#define BINDPLUG_PIN                PC15
#define RX_CC2500_SPI_LNA_EN_PIN     NONE
#define DEFAULT_RX_FEATURE          FEATURE_RX_SPI
#define RX_SPI_DEFAULT_PROTOCOL     RX_SPI_FRSKY_D
#define USE_RX_FRSKY_SPI_TELEMETRY
#define USE_RX_CC2500_SPI_DIVERSITY
#define USE_RX_CC2500_SPI_PA_LNA
#define USE_RX_FRSKY_SPI_D
#define USE_RX_FRSKY_SPI_X
#define USE_RX_SFHSS_SPI
#define USE_RX_REDPINE_SPI
#endif

// *************** OSD *****************************
#define USE_MAX7456
#define MAX7456_SPI_INSTANCE            SPI3
#define MAX7456_SPI_CS_PIN              SPI3_NSS_PIN

// *************** VTX *****************************
#define USE_VTX_RTC6705
#define RTC6705_CS_PIN                  PA14
#define RTC6705_SPI_INSTANCE            SPI3

#define USE_VTX_RTC6705_SOFTSPI
#define RTC6705_SPI_MOSI_PIN            SPI3_MOSI_PIN
#define RTC6705_SPICLK_PIN              SPI3_SCK_PIN
#define USE_RTC6705_SOFTSPI_ON_HW_SPI
#define RTC6705_POWER_PIN               PA6
#define RTC6705_POWER_PIN_HIGH_ENABLE
#define USE_RTC6705_PITMODE_CTRL
#if (defined(BEEBRAIN_PRO_DSM_US) || defined(BEEBRAIN_PRO_FRSKY_US) || defined(BEEBRAIN_PRO))
#define USE_VTX_LOCK_FREQ
#endif

// *************** BARO ****************************
// #define USE_BARO
// #define USE_BARO_BMP280
// #define USE_BARO_SPI_BMP280
// #define DEFAULT_BARO_SPI_BMP280
// #define BARO_SPI_INSTANCE               SPI3
// #define BARO_CS_PIN                     PA1

// *************** ADC *****************************
#define USE_ADC
#define ADC_INSTANCE                    ADC1
#define ADC1_DMA_STREAM                 DMA2_Stream0
#define VBAT_ADC_PIN                    PB1

#define DEFAULT_VOLTAGE_METER_SOURCE    VOLTAGE_METER_ADC
#define DEFAULT_CURRENT_METER_SOURCE    CURRENT_METER_NONE

// *************** FLASH ***************************
// #if defined(BEEBRAIN_LITED)
// #define USE_FLASHFS
// #define USE_FLASH_M25P16
// #define FLASH_CS_PIN         SPI2_NSS_PIN
// #define FLASH_SPI_INSTANCE   SPI2
// #define ENABLE_BLACKBOX_LOGGING_ON_SPIFLASH_BY_DEFAULT
// #endif
// *************** OTHERS **************************
#define LED0_PIN                        PC13
#define LED1_PIN                        PC14

#define USE_BEEPER
#define BEEPER_PIN                      PC0
#define BEEPER_INVERTED

/*---------- turtle SWITCH---------*/
#define USE_PINIO
#define PINIO1_PIN              PA8 // turtle switcher
#define USE_PINIOBOX

#define USE_USB_DETECT
#define USB_DETECT_PIN                  PA5

#define USE_ESCSERIAL
#define USE_SERIAL_4WAY_BLHELI_INTERFACE
#define ENABLE_DSHOT_DMAR               true

#define DEFAULT_FEATURES                (FEATURE_OSD | FEATURE_TELEMETRY | FEATURE_LED_STRIP)

#define TARGET_IO_PORTA                 0xffff
#define TARGET_IO_PORTB                 0xffff
#define TARGET_IO_PORTC                 0xffff
#define TARGET_IO_PORTD                 (BIT(2))

#define USABLE_TIMER_CHANNEL_COUNT      5
#define USED_TIMERS                     ( TIM_N(4)|TIM_N(5) )
