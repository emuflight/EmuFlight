/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
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

#define FC_TARGET_MCU     STM32F7X2

#define BOARD_NAME        ELINF722
#define MANUFACTURER_ID   DRCL

#define USE_ACC
#define USE_ACC_SPI_MPU6500
#define USE_GYRO
#define USE_GYRO_SPI_MPU6500
#define USE_MAX7456

#define BEEPER_PIN           PB4
#define MOTOR1_PIN           PB0
#define MOTOR2_PIN           PB1
#define MOTOR3_PIN           PA3
#define MOTOR4_PIN           PA2
#define RX_PPM_PIN           PB6
#define LED_STRIP_PIN        PB6
#define UART1_TX_PIN         PA9
#define UART3_TX_PIN         PB10
#define UART4_TX_PIN         PA0
#define UART6_TX_PIN         PC6
#define SOFTSERIAL1_TX_PIN   PC9
#define UART1_RX_PIN         PA10
#define UART3_RX_PIN         PB11
#define UART4_RX_PIN         PA1
#define UART6_RX_PIN         PC7
#define SOFTSERIAL2_RX_PIN   PA8
#define I2C1_SCL_PIN         PB8
#define I2C1_SDA_PIN         PB9
#define LED0_PIN             PB5
#define SPI1_SCK_PIN         PA5
#define SPI2_SCK_PIN         PB13
#define SPI3_SCK_PIN         PC10
#define SPI1_SDI_PIN         PA6
#define SPI2_SDI_PIN         PB14
#define SPI3_SDI_PIN         PC11
#define SPI1_SDO_PIN         PA7
#define SPI2_SDO_PIN         PB15
#define SPI3_SDO_PIN         PC12
#define CAMERA_CONTROL_PIN   PB7
#define ADC_VBAT_PIN         PC2
#define ADC_CURR_PIN         PC1
#define PINIO1_PIN           PC13
#define PINIO2_PIN           PC14
#define FLASH_CS_PIN         PB3
#define MAX7456_SPI_CS_PIN   PC8
#define GYRO_1_EXTI_PIN      PC4
#define GYRO_1_CS_PIN        PA4
#define USB_DETECT_PIN       PC15

#define TIMER_PIN_MAPPING \
    TIMER_PIN_MAP( 0, PB0 , 2,  0) \
    TIMER_PIN_MAP( 1, PB1 , 2,  0) \
    TIMER_PIN_MAP( 2, PA3 , 1,  1) \
    TIMER_PIN_MAP( 3, PA2 , 1,  0) \
    TIMER_PIN_MAP( 4, PA8 , 1,  0) \
    TIMER_PIN_MAP( 5, PC9 , 2,  0) \
    TIMER_PIN_MAP( 6, PB6 , 1,  0) \
    TIMER_PIN_MAP( 7, PB7 , 1,  0)



#define ADC1_DMA_OPT        1


//TODO #define SERIALRX_HALFDUPLEX ON
#define DEFAULT_DSHOT_BURST DSHOT_DMAR_ON
#define DEFAULT_CURRENT_METER_SOURCE CURRENT_METER_ADC
#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define DEFAULT_PID_PROCESS_DENOM 1
//TODO #define OSD_WARN_CORE_TEMP OFF
//TODO #define OSD_WARN_RC_SMOOTHING OFF
//TODO #define OSD_WARN_FAIL_SAFE OFF
//TODO #define OSD_WARN_LAUNCH_CONTROL OFF
//TODO #define OSD_WARN_NO_GPS_RESCUE OFF
//TODO #define OSD_WARN_GPS_RESCUE_DISABLED OFF
//TODO #define OSD_VBAT_POS 2401
//TODO #define OSD_RSSI_POS 2106
//TODO #define OSD_VTX_CHANNEL_POS 2424
//TODO #define OSD_CROSSHAIRS_POS 2253
//TODO #define OSD_AH_SBAR_POS 2254
//TODO #define OSD_AH_POS 2126
//TODO #define OSD_COMPASS_BAR_POS 106
//TODO #define OSD_WARNINGS_POS 2377
//TODO #define VCD_VIDEO_SYSTEM NTSC
#define MAX7456_SPI_INSTANCE SPI2
#define PINIO1_BOX 40
#define PINIO2_BOX 41
#define FLASH_SPI_INSTANCE SPI3
#define GYRO_1_SPI_INSTANCE SPI1
#define GYRO_2_SPI_INSTANCE SPI1
