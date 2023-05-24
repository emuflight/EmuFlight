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

#define TARGET_BOARD_IDENTIFIER "DIAT"
#define USBD_PRODUCT_STRING "MAMBAF405_2022B"

#define USE_ACC
#define USE_ACC_SPI_MPU6000
#define USE_GYRO
#define USE_GYRO_SPI_MPU6000
#define USE_ACC_SPI_MPU6500
#define USE_GYRO_SPI_MPU6500
#define USE_GYRO_SPI_ICM42688P
#define USE_ACC_SPI_ICM42688P
#define USE_BARO_DPS310
#define USE_FLASH
#define USE_FLASHFS
#define USE_FLASH_M25P16
#define USE_MAX7456
#define USE_BARO
#define USE_ADC
#define USE_SPI_GYRO
// manual
#define USE_VCP
#define USE_SERIAL_4WAY_BLHELI_INTERFACE

#define USE_LED
#define LED0_PIN             PC15
#define LED1_PIN             PC14
#define BEEPER_PIN           PC13
#define BEEPER_INVERTED

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

#define USE_EXTI

#define GYRO_1_ALIGN CW270_DEG
#define GYRO_1_CS_PIN        PA4
#define GYRO_1_EXTI_PIN      PC4
#define GYRO_1_SPI_INSTANCE SPI1

#define USE_MPU_DATA_READY_SIGNAL

#define ACC_MPU6000_ALIGN         GYRO_1_ALIGN
#define GYRO_MPU6000_ALIGN        GYRO_1_ALIGN
#define MPU6000_CS_PIN            GYRO_1_CS_PIN
#define MPU6000_SPI_INSTANCE      GYRO_1_SPI_INSTANCE
#define MPU_INT_EXTI              GYRO_1_EXTI_PIN

#define ACC_MPU6500_ALIGN         GYRO_1_ALIGN
#define GYRO_MPU6500_ALIGN        GYRO_1_ALIGN
#define MPU6500_CS_PIN            GYRO_1_CS_PIN
#define MPU6500_SPI_INSTANCE      GYRO_1_SPI_INSTANCE
#define MPU_INT_EXTI              GYRO_1_EXTI_PIN

#define ACC_ICM42688P_ALIGN       GYRO_1_ALIGN
#define GYRO_ICM42688P_ALIGN      GYRO_1_ALIGN
#define ICM42688P_CS_PIN          GYRO_1_CS_PIN
#define ICM42688P_SPI_INSTANCE    GYRO_1_SPI_INSTANCE

// notice - this file was programmatically generated and may need GYRO_2 manually added.

#define USE_I2C
#define USE_I2C_DEVICE_1
#define MAG_I2C_INSTANCE (I2CDEV_1)
#define BARO_I2C_INSTANCE (I2CDEV_1)
#define I2C1_SCL PB8
#define I2C1_SDA PB9

#define FLASH_CS_PIN         PA15
#define FLASH_SPI_INSTANCE SPI3
#define ENABLE_BLACKBOX_LOGGING_ON_SPIFLASH_BY_DEFAULT

#define MAX7456_SPI_CS_PIN   PB12
#define MAX7456_SPI_INSTANCE SPI2

#define UART1_TX_PIN         PB6
#define UART2_TX_PIN         PA2
#define UART3_TX_PIN         PB10
#define UART4_TX_PIN         PA0
#define UART5_TX_PIN         PC12
#define UART6_TX_PIN         PC6
#define UART1_RX_PIN         PB7
#define UART2_RX_PIN         PA3
#define UART3_RX_PIN         PB11
#define UART4_RX_PIN         PA1
#define UART5_RX_PIN         PD2
#define UART6_RX_PIN         PC7
#define INVERTER_PIN_UART1   PC0
#define SERIAL_PORT_COUNT 6
// notice - UART/USART were programmatically generated - must check USART validity.${hFile}
// notice - may need "#define SERIALRX_UART SERIAL_PORT_USART_" 
// notice - should check serial count.

//manual
#define USE_UART1
#define USE_UART2
#define USE_UART3
#define USE_UART4
#define USE_UART5
#define USE_UART6


#define VBAT_ADC_PIN PC1
#define CURRENT_METER_ADC_PIN PC3
#define ADC3_DMA_STREAM DMA2_Stream0 // notice - DMA2_Stream0 likely need correcting, please modify.
#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define DEFAULT_CURRENT_METER_SOURCE CURRENT_METER_ADC
#define DEFAULT_CURRENT_METER_SCALE 183

#define ENABLE_DSHOT_DMAR true

#define TARGET_IO_PORTA 0xffff
#define TARGET_IO_PORTB 0xffff
#define TARGET_IO_PORTC 0xffff
#define TARGET_IO_PORTD 0xffff
// notice - masks were programmatically generated - must verify last port group for 0xffff or (BIT(2))

#define USABLE_TIMER_CHANNEL_COUNT 9
#define USED_TIMERS ( TIM_N(1)|TIM_N(2)|TIM_N(3)|TIM_N(4)|TIM_N(8)|TIM_N(11))  /* notice - incomplete */

// notice - this file was programmatically generated and may be incomplete.
