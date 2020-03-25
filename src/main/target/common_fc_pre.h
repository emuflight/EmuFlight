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

#define USE_PARAMETER_GROUPS
// type conversion warnings.
// -Wconversion can be turned on to enable the process of eliminating these warnings
//#pragma GCC diagnostic warning "-Wconversion"
#pragma GCC diagnostic ignored "-Wsign-conversion"
// -Wpadded can be turned on to check padding of structs
//#pragma GCC diagnostic warning "-Wpadded"

//#define SCHEDULER_DEBUG // define this to use scheduler debug[] values. Undefined by default for performance reasons
#define DEBUG_MODE DEBUG_NONE // change this to change initial debug mode

#define I2C1_OVERCLOCK true
#define I2C2_OVERCLOCK true

#ifdef STM32F1
#define MINIMAL_CLI
// Using RX DMA disables the use of receive callbacks
#define USE_UART1_RX_DMA
#define USE_UART1_TX_DMA
#endif

#ifdef STM32F3
#define MINIMAL_CLI
#define USE_DSHOT
#define USE_GYRO_DATA_ANALYSE
#endif

#ifdef STM32F4
#define USE_SRAM2
#if defined(STM32F40_41xxx)
#define USE_FAST_RAM
#endif
#define USE_DSHOT
#define I2C3_OVERCLOCK true
#define USE_GYRO_DATA_ANALYSE
#define USE_ADC
#define USE_ADC_INTERNAL
#define USE_USB_CDC_HID
#define USE_USB_MSC

#if defined(STM32F40_41xxx) || defined(STM32F411xE)
#define USE_OVERCLOCK
#endif

#endif // STM32F4

#ifdef STM32F7
#define USE_SRAM2
#define USE_ITCM_RAM
#define USE_FAST_RAM
#define USE_DSHOT
#define I2C3_OVERCLOCK true
#define I2C4_OVERCLOCK true
#define USE_GYRO_DATA_ANALYSE
#define USE_OVERCLOCK
#define USE_ADC_INTERNAL
#define USE_USB_CDC_HID
#define USE_USB_MSC
#endif

#if defined(STM32F4) || defined(STM32F7)
#define TASK_GYROPID_DESIRED_PERIOD     125 // 125us = 8kHz
#define SCHEDULER_DELAY_LIMIT           10
#else
#define TASK_GYROPID_DESIRED_PERIOD     1000 // 1000us = 1kHz
#define SCHEDULER_DELAY_LIMIT           100
#endif

#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
#define DEFAULT_AUX_CHANNEL_COUNT       MAX_AUX_CHANNEL_COUNT
#else
#define DEFAULT_AUX_CHANNEL_COUNT       6
#endif

#ifdef USE_ITCM_RAM
#define FAST_CODE                   __attribute__((section(".tcm_code")))
#define FAST_CODE_NOINLINE          NOINLINE
#else
#define FAST_CODE
#define FAST_CODE_NOINLINE
#endif // USE_ITCM_RAM

#ifdef USE_FAST_RAM
#define FAST_RAM_ZERO_INIT             __attribute__ ((section(".fastram_bss"), aligned(4)))
#define FAST_RAM                    __attribute__ ((section(".fastram_data"), aligned(4)))
#else
#define FAST_RAM_ZERO_INIT
#define FAST_RAM
#endif // USE_FAST_RAM

#ifdef STM32F4
// Data in RAM which is guaranteed to not be reset on hot reboot
#define PERSISTENT                  __attribute__ ((section(".persistent_data"), aligned(4)))
#endif

#ifdef USE_SRAM2
#define SRAM2                       __attribute__ ((section(".sram2"), aligned(4)))
#else
#define SRAM2
#endif

#define USE_BRUSHED_ESC_AUTODETECT  // Detect if brushed motors are connected and set defaults appropriately to avoid motors spinning on boot
#define USE_CLI
#define USE_GYRO_REGISTER_DUMP  // Adds gyroregisters command to cli to dump configured register values
#define USE_PPM
#define USE_PWM
#define USE_SERIAL_RX
#define USE_SERIALRX_CRSF       // Team Black Sheep Crossfire protocol
#define USE_SERIALRX_IBUS       // FlySky and Turnigy receivers
#define USE_SERIALRX_SBUS       // Frsky and Futaba receivers
#define USE_SERIALRX_SPEKTRUM   // SRXL, DSM2 and DSMX protocol
#define USE_SERIALRX_SUMD       // Graupner Hott protocol
#define USE_SERIALRX_XBUS       // JR



#if (FLASH_SIZE > 64)
#define PID_PROFILE_COUNT 3
#else
#define PID_PROFILE_COUNT 2
#endif

#if (FLASH_SIZE > 64)
#define USE_BLACKBOX
#define USE_RESOURCE_MGMT
#define USE_RUNAWAY_TAKEOFF     // Runaway Takeoff Prevention (anti-taz) - Marked for removal
#define USE_TELEMETRY
#define USE_TELEMETRY_FRSKY_HUB
#define USE_TELEMETRY_HOTT
#define USE_TELEMETRY_LTM
#define USE_TELEMETRY_SMARTPORT
#endif

#if (FLASH_SIZE > 128)
#define USE_PEGASUS_UI
#define USE_SERIALRX_SUMH       // Graupner legacy protocol
#define USE_CAMERA_CONTROL
#define USE_CMS
#define USE_EXTENDED_CMS_MENUS
#define USE_DSHOT_DMAR
#define USE_GYRO_OVERFLOW_CHECK
#define USE_YAW_SPIN_RECOVERY
#define USE_HUFFMAN
#define USE_MSP_DISPLAYPORT
#define USE_MSP_OVER_TELEMETRY
#define MSP_OVER_CLI
#define USE_OSD
#define USE_OSD_OVER_MSP_DISPLAYPORT
#define USE_PINIO
#define USE_PINIOBOX
#define USE_RCDEVICE
#define USE_RTC_TIME
#define USE_RX_MSP
#define USE_SERIALRX_FPORT      // FrSky FPort
#define USE_TELEMETRY_CRSF
#define USE_TELEMETRY_SRXL
#define USE_VIRTUAL_CURRENT_METER
#define USE_VTX_COMMON
#define USE_VTX_CONTROL
#define USE_VTX_SMARTAUDIO
#define USE_VTX_TRAMP
#define USE_SERIAL_4WAY_BLHELI_BOOTLOADER
#define USE_GYRO_LPF2
#define USE_ESC_SENSOR
#define USE_ESC_SENSOR_INFO
#define USE_CRSF_CMS_TELEMETRY
#define USE_BOARD_INFO
#define USE_THROTTLE_BOOST
#define USE_RC_SMOOTHING_FILTER
#define USE_ITERM_RELAX

#ifdef USE_SERIALRX_SPEKTRUM
#define USE_SPEKTRUM_BIND
#define USE_SPEKTRUM_BIND_PLUG
#define USE_SPEKTRUM_REAL_RSSI
#define USE_SPEKTRUM_FAKE_RSSI
#define USE_SPEKTRUM_RSSI_PERCENT_CONVERSION
#define USE_SPEKTRUM_VTX_CONTROL
#define USE_SPEKTRUM_VTX_TELEMETRY
#define USE_SPEKTRUM_CMS_TELEMETRY
#endif
#endif

#if (FLASH_SIZE > 256)
#define USE_SERVOS
#define USE_LED_STRIP
#define USE_ALT_HOLD
#define USE_DASHBOARD
#define USE_GPS
#define USE_GPS_NMEA
#define USE_GPS_UBLOX
#define USE_GPS_RESCUE
#define USE_OSD
#define USE_OSD_OVER_MSP_DISPLAYPORT
#define USE_OSD_ADJUSTMENTS
#define USE_SENSOR_NAMES
#define USE_SERIALRX_JETIEXBUS
#define USE_TELEMETRY_IBUS
#define USE_TELEMETRY_IBUS_EXTENDED
#define USE_TELEMETRY_JETIEXBUS
#define USE_TELEMETRY_MAVLINK
#define USE_UNCOMMON_MIXERS
#define USE_SIGNATURE
#define USE_ABSOLUTE_CONTROL
#define USE_CMS_FAILSAFE_MENU
#define USE_CMS_GPS_RESCUE_MENU
#endif
