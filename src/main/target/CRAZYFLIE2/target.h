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

/*
 * This target is for the Crazyflie 2.0 nanocopter board
 *
 * For details on using this target with the Crazyflie see:
 * https://wiki.bitcraze.io/misc:hacks:betaflight
 *
 * Target code written and maintained by Sean Kelly (theseankelly@outlook.com)
 */

#pragma once

#define FC_TARGET_MCU     STM32F405
#define TARGET_BOARD_IDENTIFIER "CF20"
#define USBD_PRODUCT_STRING     "Crazyflie 2.0"


#define TIMER_PIN_MAPPING \
    TIMER_PIN_MAP( 0, PB11 , 1,  0) /* PWM1 - OUT1 (Motor 1) */ \
    TIMER_PIN_MAP( 1, PA1 , 1,  0) /* PWM2 - OUT2 (Motor 2) */ \
    TIMER_PIN_MAP( 2, PA15 , 1,  0) /* PWM3 - OUT3 (Motor 3) */ \
    TIMER_PIN_MAP( 3, PB9 , 1,  -1) /* PWM4 - OUT4 (Motor 4) */

#define MOTOR1_PIN              PB11
#define MOTOR2_PIN              PA1
#define MOTOR3_PIN              PA15
#define MOTOR4_PIN              PB9

#define LED0_PIN                PD2
#define LED1_PIN                PC0
#define LED2_PIN                PC3

// Using STM32F405RG, 64 pin package (LQFP64)
// 16 pins per port, ports A, B, C, and also PD2

#define USE_VCP

#define USE_UART6
#define UART6_TX_PIN            PC6
#define UART6_RX_PIN            PC7

#define SERIAL_PORT_COUNT       2

#define USE_I2C
#define USE_I2C_DEVICE_3
#define I2C_DEVICE              (I2CDEV_3)

// MPU9250 has the AD0 pin held high so the
// address is 0x69 instead of the default 0x68
#define MPU_ADDRESS             0x69

#define USE_GYRO
#define USE_GYRO_MPU6500
#define GYRO_MPU6500_ALIGN      CW270_DEG

#define USE_ACC
#define USE_ACC_MPU6500
#define ACC_MPU6500_ALIGN       CW270_DEG

#define USE_MAG
#define USE_MPU9250_MAG // Enables bypass configuration on the MPU9250 I2C bus
#define USE_MAG_AK8963
#define MAG_AK8963_ALIGN        CW270_DEG

#define USE_EXTI
#define MPU_INT_EXTI            PC13

#define USE_SERIALRX_TARGET_CUSTOM
#define SERIALRX_UART           SERIAL_PORT_USART6
#define SERIALRX_PROVIDER       SERIALRX_TARGET_CUSTOM
#define RX_CHANNELS_TAER

#define DEFAULT_RX_FEATURE      FEATURE_RX_SERIAL

#define BRUSHED_MOTORS
