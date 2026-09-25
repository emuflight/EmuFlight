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
#define TARGET_BOARD_IDENTIFIER "KTV1"
#define USBD_PRODUCT_STRING "KakuteF4-V1"

#define USE_TARGET_CONFIG

#define LED0_PIN                PB5
#define LED1_PIN                PB4
#define LED2_PIN                PB6


#define USE_BEEPER

#define BEEPER_PIN              PC9

#define BEEPER_INVERTED
#define INVERTER_PIN_UART3      PB15

// ICM20689 interrupt
#define USE_EXTI
#define MPU_INT_EXTI            PC5
//#define DEBUG_MPU_DATA_READY_INTERRUPT
#define USE_MPU_DATA_READY_SIGNAL
#define ENSURE_MPU_DATA_READY_IS_LOW

#define  ICM20689_CS_PIN          PC4
#define ICM20689_SPI_BUS    SPIDEV_1

#define USE_ACC
#define USE_ACC_SPI_ICM20689
#define ACC_ICM20689_ALIGN       CW270_DEG

#define USE_GYRO
#define USE_GYRO_SPI_ICM20689
#define GYRO_ICM20689_ALIGN      CW270_DEG

#define USE_MAX7456
#define MAX7456_SPI_INSTANCE    SPI3
#define MAX7456_SPI_CS_PIN      PB14
#define MAX7456_SPI_CLK         (SPI_CLOCK_STANDARD)
#define MAX7456_RESTORE_CLK     (SPI_CLOCK_FAST)
#define FLASH_CS_PIN            PB3
#define FLASH_SPI_INSTANCE      SPI3

#define USE_FLASHFS
#define USE_FLASH_M25P16
#define USE_FLASH_W25Q128FV

#define USE_VCP
#define USB_DETECT_PIN          PA8
#define USE_USB_DETECT

#define USE_UART1
#define UART1_RX_PIN            PA10

#define UART1_TX_PIN            PA9

#define UART1_AHB1_PERIPHERALS  RCC_AHB1Periph_DMA2

#define USE_UART3
#define UART3_RX_PIN            PB11
#define UART3_TX_PIN            PB10

#define USE_UART6
#define UART6_RX_PIN            PC7
#define UART6_TX_PIN            PC6

#define USE_SOFTSERIAL1
#define USE_SOFTSERIAL2
#define SERIAL_PORT_COUNT 6   //vcp, uart1, uart3,, uart6, softSerial1, softSerial2

#define ESCSERIAL_TIMER_TX_PIN  PC7  // (HARDARE=0,PPM)

#define USE_SPI
#define USE_SPI_DEVICE_1 //ICM20689
#define SPI1_NSS_PIN            PC4
#define SPI1_SCK_PIN            PA5
#define SPI1_MISO_PIN           PA6
#define SPI1_MOSI_PIN           PA7

#define USE_SPI_DEVICE_3 //dataflash
#define SPI3_NSS_PIN            PB3
#define SPI3_SCK_PIN            PC10
#define SPI3_MISO_PIN           PC11
#define SPI3_MOSI_PIN           PC12

#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define USE_ADC
#define ADC1_DMA_STREAM             DMA2_Stream0
#define VBAT_ADC_PIN                PC3
#define CURRENT_METER_ADC_PIN       PC2
#define DEFAULT_CURRENT_METER_SCALE        275
#define RSSI_ADC_PIN                PC1

#define DEFAULT_FEATURES        ( FEATURE_TELEMETRY | FEATURE_OSD )
#define DEFAULT_RX_FEATURE      FEATURE_RX_SERIAL
#define SERIALRX_PROVIDER       SERIALRX_SBUS
#define SERIALRX_UART           SERIAL_PORT_USART3


#define USABLE_TIMER_CHANNEL_COUNT 8
#define USE_TIMER_MGMT
#define TIMER_PIN_MAPPING \
    TIMER_PIN_MAP( 0, PC7 , 2,  0) \
    TIMER_PIN_MAP( 1, PB0 , 2,  0) \
    TIMER_PIN_MAP( 2, PB1 , 2,  0) \
    TIMER_PIN_MAP( 3, PA3 , 1,  1) \
    TIMER_PIN_MAP( 4, PA2 , 1,  0) \
    TIMER_PIN_MAP( 5, PA0 , 2,  0) \
    TIMER_PIN_MAP( 6, PC8 , 2,  1) \
    TIMER_PIN_MAP( 7, PA1 , 2,  0)

#define MOTOR1_PIN              PB0
#define MOTOR2_PIN              PB1
#define MOTOR3_PIN              PA3
#define MOTOR4_PIN              PA2
#define MOTOR5_PIN              PA0
#define MOTOR6_PIN              PC8
#define RX_PPM_PIN              PC7
#define LED_STRIP_PIN           PA1
