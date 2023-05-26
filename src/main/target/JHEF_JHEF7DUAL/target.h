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

#pragma once

#define TARGET_BOARD_IDENTIFIER "JHEF"
#define USBD_PRODUCT_STRING "JHEF7DUAL"

#define USE_ACC
#define USE_ACC_SPI_MPU6000
#define USE_GYRO
#define USE_GYRO_SPI_MPU6000
#define USE_ACC_SPI_ICM20689
#define USE_GYRO_SPI_ICM20689
#define USE_GYRO_SPI_ICM42688P
#define USE_ACC_SPI_ICM42688P
#define USE_BARO_BMP280
#define USE_FLASH
#define USE_FLASH_M25P16
#define USE_MAX7456
#define USE_BARO
#define USE_ADC
#define USE_SPI_GYRO

#define USE_VCP
#define USE_FLASHFS
#define USE_BEEPER
#define USE_USB_DETECT

#define USE_LED
#define LED0_PIN             PA15
#define LED_STRIP_PIN        PA8
#define BEEPER_PIN           PC15
#define BEEPER_INVERTED
#define CAMERA_CONTROL_PIN   PB8

#define USE_SPI
#define USE_SPI_DEVICE_1
#define SPI1_SCK_PIN         PA5
#define SPI1_MOSI_PIN        PA6
#define SPI1_MISO_PIN        PA7
#define USE_SPI_DEVICE_2
#define SPI2_SCK_PIN         PB13
#define SPI2_MOSI_PIN        PB14
#define SPI2_MISO_PIN        PB15
#define USE_SPI_DEVICE_3
#define SPI3_SCK_PIN         PC10
#define SPI3_MOSI_PIN        PC11
#define SPI3_MISO_PIN        PB5

#define GYRO_1_ALIGN CW90_DEG
#define ACC_1_ALIGN GYRO_1_ALIGN
#define GYRO_1_CS_PIN        PB2
#define GYRO_1_EXTI_PIN      PC4
#define GYRO_1_SPI_INSTANCE SPI1
#define GYRO_2_ALIGN CW90_DEG
#define ACC_2_ALIGN GYRO_2_ALIGN
#define GYRO_2_CS_PIN        PA4
#define GYRO_2_EXTI_PIN      PC3
#define GYRO_2_SPI_INSTANCE SPI1

#define USE_DUAL_GYRO

#define USE_EXTI
#define USE_GYRO_EXTI

#define USE_MPU_DATA_READY_SIGNAL

#define ACC_MPU6000_ALIGN         GYRO_1_ALIGN
#define GYRO_MPU6000_ALIGN        GYRO_1_ALIGN
#define MPU6000_CS_PIN            GYRO_1_CS_PIN
#define MPU6000_SPI_INSTANCE      GYRO_1_SPI_INSTANCE
#define MPU_INT_EXTI              GYRO_1_EXTI_PIN

#define ACC_ICM20689_ALIGN        GYRO_1_ALIGN
#define GYRO_ICM20689_ALIGN       GYRO_1_ALIGN
#define ICM20689_CS_PIN           GYRO_1_CS_PIN
#define ICM20689_SPI_INSTANCE     GYRO_1_SPI_INSTANCE

#define ACC_ICM42688P_ALIGN       GYRO_1_ALIGN
#define GYRO_ICM42688P_ALIGN      GYRO_1_ALIGN
#define ICM42688P_CS_PIN          GYRO_1_CS_PIN
#define ICM42688P_SPI_INSTANCE    GYRO_1_SPI_INSTANCE

// notice - this file was programmatically generated and may need GYRO_2 manually added.

#define USE_I2C
#define USE_I2C_DEVICE_1
#define BARO_I2C_INSTANCE (I2CDEV_1)
#define I2C1_SCL PB6
#define I2C1_SDA PB7

#define FLASH_CS_PIN         PC13
#define FLASH_SPI_INSTANCE SPI3
#define ENABLE_BLACKBOX_LOGGING_ON_SPIFLASH_BY_DEFAULT

#define MAX7456_SPI_CS_PIN   PB12
#define MAX7456_SPI_INSTANCE SPI2

#define USE_UART1
#define USE_UART2
#define USE_UART3
#define USE_UART4
#define USE_UART5
#define USE_UART6
#define UART1_TX_PIN         PA9
#define UART2_TX_PIN         PA2
#define UART3_TX_PIN         PB10
#define UART4_TX_PIN         PA0
#define UART5_TX_PIN         PC12
#define UART6_TX_PIN         PC6
#define UART1_RX_PIN         PA10
#define UART2_RX_PIN         PA3
#define UART3_RX_PIN         PB11
#define UART4_RX_PIN         PA1
#define UART5_RX_PIN         PD2
#define UART6_RX_PIN         PC7
#define SERIAL_PORT_COUNT 6

//manually edited
#define RX_PPM_PIN           PA3

// notice - UART/USART were programmatically generated - should verify UART/USART.
// notice - may need "#define SERIALRX_UART SERIAL_PORT_USART_"
// notice - should verify serial count.

#define VBAT_ADC_PIN PC2
#define CURRENT_METER_ADC_PIN PC1
#define RSSI_ADC_PIN PC0
#define ADC3_DMA_STREAM DMA2_Stream1 //manually edited Stream1
#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define DEFAULT_CURRENT_METER_SOURCE CURRENT_METER_ADC
#define DEFAULT_CURRENT_METER_SCALE 450

#define ENABLE_DSHOT_DMAR true

// notice - this file was programmatically generated and did not account for any potential LEDx_INVERTED, inverted Telem, etc.

#define TARGET_IO_PORTA 0xffff
#define TARGET_IO_PORTB 0xffff
#define TARGET_IO_PORTC 0xffff
#define TARGET_IO_PORTD 0xffff
// notice - masks were programmatically generated - must verify last port group for 0xffff or (BIT(2))

#define USABLE_TIMER_CHANNEL_COUNT 9
#define USED_TIMERS ( TIM_N(1)|TIM_N(2)|TIM_N(3)|TIM_N(4)|TIM_N(8)|TIM_N(9) )

// notice - this file was programmatically generated and may be incomplete.
