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

#define BOARD_NAME        BETAFPVF411RX
#define MANUFACTURER_ID   BEFH
#define TARGET_BOARD_IDENTIFIER "S411"  // generic ID
#define FC_TARGET_MCU     STM32F411     // not used in EmuF

#define LED0_PIN                PC14

#define USE_BEEPER
#define BEEPER_PIN           PA14
#define BEEPER_INVERTED

// *************** Gyro & ACC **********************
#define USE_SPI
#define USE_SPI_DEVICE_1

#define SPI1_SCK_PIN            PA5
#define SPI1_MISO_PIN           PA6
#define SPI1_MOSI_PIN           PA7

#define USE_EXTI
#define USE_GYRO_EXTI
#define MPU_INT_EXTI         PB6
#define USE_MPU_DATA_READY_SIGNAL
#define ENSURE_MPU_DATA_READY_IS_LOW

#define USE_ACC
#define USE_GYRO

#define USE_ACC_SPI_MPU6000
#define USE_GYRO_SPI_MPU6000
#define GYRO_MPU6000_ALIGN      CW180_DEG
#define ACC_MPU6000_ALIGN       CW180_DEG
#define MPU6000_CS_PIN          PA4
#define MPU6000_SPI_INSTANCE    SPI1

#define USE_ACC_SPI_MPU6500
#define USE_GYRO_SPI_MPU6500
#define ACC_MPU6500_ALIGN       CW180_DEG
#define GYRO_MPU6500_ALIGN      CW180_DEG
#define MPU6500_CS_PIN          PA4
#define MPU6500_SPI_INSTANCE    SPI1

#define USE_ACCGYRO_BMI270
#define USE_SPI_GYRO
#define ACC_BMI270_ALIGN        CW180_DEG
#define GYRO_BMI270_ALIGN       CW180_DEG
#define BMI270_CS_PIN           PA4
#define BMI270_SPI_INSTANCE     SPI1

// *************** UART *****************************
#define USE_VCP
#define USE_USB_DETECT

#define USE_UART1
#define UART1_RX_PIN            PA10
#define UART1_TX_PIN            PA9

#define USE_UART2
#define UART2_RX_PIN            PA3
#define UART2_TX_PIN            PA2

#define SERIAL_PORT_COUNT       3


// *************** SPI3 CC2500 ***************************
#define USE_SPI_DEVICE_3
#define SPI3_SCK_PIN            PB3
#define SPI3_MISO_PIN           PB4
#define SPI3_MOSI_PIN           PB5
#define SPI3_NSS_PIN            PA15 //RX_SPI_CS
#define RX_NSS_PIN              SPI3_NSS_PIN

#define RX_NSS_GPIO_CLK_PERIPHERAL   RCC_APB2Periph_GPIOA

#define USE_RX_SPI
#define RX_SPI_INSTANCE         SPI3
#define RX_CC2500_SPI_DISABLE_CHIP_DETECTION
#define RX_CC2500_SPI_GDO_0_PIN     PC13 //RX_SPI_EXTI
#define RX_CC2500_SPI_LED_PIN       PC15 //RX_SPI_led
#define RX_CC2500_SPI_LED_PIN_INVERTED
#define RX_FRSKY_SPI_LED_PIN_INVERTED
#define USE_RX_CC2500_SPI_PA_LNA
#define RX_CC2500_SPI_TX_EN_PIN      PB9 //RX_SPI_CC2500_TX_EN
#define RX_CC2500_SPI_LNA_EN_PIN     PA13 //RX_SPI_CC2500_LNA_EN
#define USE_RX_CC2500_SPI_DIVERSITY
#define RX_CC2500_SPI_ANT_SEL_PIN    PA14 //RX_SPI_C250_ANT_SEL
#define BINDPLUG_PIN            PB2 //RX_SPI_BIND
#define USE_RX_FRSKY_SPI_D
#define USE_RX_FRSKY_SPI_X
#define USE_RX_REDPINE_SPI
#define USE_RX_SFHSS_SPI
#define DEFAULT_RX_FEATURE      FEATURE_RX_SPI
#define RX_SPI_DEFAULT_PROTOCOL RX_SPI_FRSKY_D
#define USE_RX_FRSKY_SPI_TELEMETRY

// *************** OSD/FLASH *****************************
#define USE_SPI_DEVICE_2
#define SPI2_SCK_PIN            PB13
#define SPI2_MISO_PIN           PB14
#define SPI2_MOSI_PIN           PB15

#define USE_OSD
#define USE_MAX7456
#define MAX7456_SPI_INSTANCE    SPI2
#define MAX7456_SPI_CS_PIN      PB12

#define ENABLE_BLACKBOX_LOGGING_ON_SPIFLASH_BY_DEFAULT
#define USE_FLASHFS
#define USE_FLASH_M25P16
#define FLASH_CS_PIN            PA8
#define FLASH_SPI_INSTANCE      SPI2

// *************** ADC *****************************
#define USE_ADC
#define ADC_INSTANCE         ADC1
#define ADC1_DMA_OPT         1
#define ADC1_DMA_STREAM DMA2_Stream4 //# ADC 1: DMA2 Stream 4 Channel 0
#define VBAT_ADC_PIN            PA1
#define CURRENT_METER_ADC_PIN   PB0
#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define DEFAULT_CURRENT_METER_SOURCE CURRENT_METER_ADC
#define DEFAULT_CURRENT_METER_SCALE 179

//#define RSSI_ADC_PIN            PB1
//#define EXTERNAL1_ADC_PIN       PA4

#define USE_ESCSERIAL

#define USE_LED_STRIP

//#define ENABLE_DSHOT_DMAR       true
//#define USE_PINIO
//#define PINIO1_PIN              PB5  // VTX  switcher
//#define PINIO2_PIN              PA15 // Camera switcher
//#define USE_PINIOBOX

#define DEFAULT_FEATURES                (FEATURE_OSD | FEATURE_TELEMETRY | FEATURE_AIRMODE)

#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         0xffff
#define TARGET_IO_PORTD         (BIT(2))

#define USABLE_TIMER_CHANNEL_COUNT 7
#define USED_TIMERS             ( TIM_N(1)|TIM_N(2)|TIM_N(3)|TIM_N(4) )
