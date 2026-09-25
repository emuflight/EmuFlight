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
#define TARGET_BOARD_IDENTIFIER "ELL0"

#define TARGET_XTAL_MHZ         25

#define USBD_PRODUCT_STRING "Elle0"

#define LED0_PIN                PA8
#define LED1_PIN                PB4
#define LED2_PIN                PC2

// MPU9250 interrupt
#define USE_EXTI
#define MPU_INT_EXTI            PB5
//#define DEBUG_MPU_DATA_READY_INTERRUPT
#define USE_MPU_DATA_READY_SIGNAL
#define ENSURE_MPU_DATA_READY_IS_LOW

#define MPU6500_CS_PIN          PB12
#define MPU6500_SPI_BUS    SPIDEV_2

// Using MPU6050 for the moment.
#define USE_GYRO
#define USE_GYRO_SPI_MPU6500
#define GYRO_MPU6500_ALIGN      CW270_DEG

#define USE_ACC
#define USE_ACC_SPI_MPU6500
#define ACC_MPU6500_ALIGN       CW270_DEG

//#define USE_BARO
//#define USE_BARO_MS5611

#define USE_MAG
#define USE_MAG_AK8963
#define MAG_AK8963_ALIGN        CW0_DEG_FLIP

#define USE_VCP

/* Telemetry (Overlaps with DMA from motors) */
//#define USE_UART1
//#define UART1_RX_PIN            PA10
//#define UART1_TX_PIN            PA9
//#define UART1_AHB1_PERIPHERALS  RCC_AHB1Periph_DMA2

/* RX1 */
#define USE_UART2
#define UART2_RX_PIN        PA3
#define UART2_TX_PIN        PA2

/* I2C */
#define USE_UART3
#define UART3_RX_PIN            PB11
#define UART3_TX_PIN            PB10

/* RX2 */
//#define USE_UART5
//#define UART5_RX_PIN            PD2
//#define UART5_TX_PIN            PC12

#define USE_SOFTSERIAL1
#define USE_SOFTSERIAL2

#define SERIAL_PORT_COUNT 5

#define USE_SPI

#define USE_SPI_DEVICE_2 //MPU9250
#define SPI2_NSS_PIN            PB12
#define SPI2_SCK_PIN            PB13
#define SPI2_MISO_PIN           PB14
#define SPI2_MOSI_PIN           PB15

#define USE_ADC
#define VBAT_ADC_PIN            PC4
#define CURRENT_METER_ADC_PIN   PC5
#define ADC1_DMA_STREAM DMA2_Stream0 //# ADC 1: DMA2 Stream 0 Channel 0

#undef USE_LED_STRIP

#define DEFAULT_RX_FEATURE      FEATURE_RX_SERIAL
#define SERIALRX_PROVIDER    SERIALRX_SPEKTRUM2048
#define SERIALRX_UART           SERIAL_PORT_USART2
#define RX_CHANNELS_TAER


#define USABLE_TIMER_CHANNEL_COUNT 9
#define USE_TIMER_MGMT
#define TIMER_PIN_MAPPING \
    TIMER_PIN_MAP( 0, PA2 , 1,  0) \
    TIMER_PIN_MAP( 1, PC6 , 2,  0) \
    TIMER_PIN_MAP( 2, PC7 , 2,  1) \
    TIMER_PIN_MAP( 3, PC8 , 2,  1) \
    TIMER_PIN_MAP( 4, PC9 , 2,  0) \
    TIMER_PIN_MAP( 5, PA0 , 2,  0) \
    TIMER_PIN_MAP( 6, PA1 , 2,  0) \
    TIMER_PIN_MAP( 7, PB8 , 1,  0) \
    TIMER_PIN_MAP( 8, PB9 , 1,  0)

#define MOTOR1_PIN              PC6
#define MOTOR2_PIN              PC7
#define MOTOR3_PIN              PC8
#define MOTOR4_PIN              PC9
#define MOTOR5_PIN              PA0
#define MOTOR6_PIN              PA1
#define MOTOR7_PIN              PB8
#define MOTOR8_PIN              PB9
#define RX_PWM1_PIN             PA2
#define RX_PPM_PIN              PA2
