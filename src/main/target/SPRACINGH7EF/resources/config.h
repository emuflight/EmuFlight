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

#define FC_TARGET_MCU           STM32H730

#define BOARD_NAME              SPRACINGH7EF
#define MANUFACTURER_ID         SPRO

#define TARGET_BOARD_IDENTIFIER "SP7E"
#define USBD_PRODUCT_STRING     "SPRacingH7EF"

#define FC_VMA_ADDRESS          0x90100000

#define EEPROM_SIZE             8192

#define USE_SPRACING_PERSISTENT_RTC_WORKAROUND

#define USE_BUTTONS
#define BUTTON_A_PIN            PD10
#define BUTTON_A_PIN_INVERTED
#define BUTTON_B_PIN            PD10
#define BUTTON_B_PIN_INVERTED

// FC has 2 flash chips, one for logging on SPI6, one for code/data storage on OCTOSPIM_P1 (Memory mapped mode)
// Config is to be stored on the flash chip used for code/data storage, this requires that the
// config load/save routines must run from RAM and a method to enable/disable memory mapped mode is needed.

// Since there is currently no firmware support for using OCTOSPI *and* SPI for 2 flash chips by default we just
// define the SPI flash configuration, then the firmware ignores OCTOSPI even though the firmware is running from OctoSPI!
// At some point in the future, memory mapped flash, with config storage on octospi memory mapped flash, can be made to work with SPI flash logging.
//#define USE_FLASH_MEMORY_MAPPED
//#define USE_OCTOSPI
#if defined(USE_OCTOSPI)
#define USE_OCTOSPI_DEVICE_1

#if !defined(USE_FLASH_MEMORY_MAPPED)
// Bootloader will have configured the OCTOSPI and OCTOSPIM peripherals and pins, firmware needs no-knowledge of the configuration.
#define OCTOSPIM_P1_SCK_PIN PB2
#define OCTOSPIM_P1_CS_PIN PB10

// Not using IO0:3
#define OCTOSPIM_P1_IO0_PIN NONE
#define OCTOSPIM_P1_IO1_PIN NONE
#define OCTOSPIM_P1_IO2_PIN NONE
#define OCTOSPIM_P1_IO3_PIN NONE

// Using IO4:7
#define OCTOSPIM_P1_IO4_PIN PE7
#define OCTOSPIM_P1_IO5_PIN PE8
#define OCTOSPIM_P1_IO6_PIN PE9
#define OCTOSPIM_P1_IO7_PIN PE10

#define OCTOSPIM_P1_MODE OCTOSPIM_P1_MODE_IO47_ONLY
#define OCTOSPIM_P1_CS_FLAGS (OCTOSPIM_P1_CS_HARDWARE)
#endif
#endif

#define USE_FLASH_CHIP
//#define CONFIG_IN_MEMORY_MAPPED_FLASH
#define CONFIG_IN_EXTERNAL_FLASH
//#define CONFIG_IN_SDCARD
//#define CONFIG_IN_RAM
#if !defined(CONFIG_IN_RAM) && !defined(CONFIG_IN_SDCARD) && !defined(CONFIG_IN_EXTERNAL_FLASH) && !defined(CONFIG_IN_MEMORY_MAPPED_FLASH)
#error "Config storage location not defined"
#endif

#define USE_FLASHFS
#define USE_FLASH_TOOLS
// For the SPI flash chip for logging
#define USE_FLASH_M25P16

#define FLASH_CS_PIN            PD7 // *NOT* SPI6_NSS, SOFTWARE CS ONLY
#define FLASH_SPI_INSTANCE      SPI6

// For the Octo-Spi connected
#ifdef USE_OCTOSPI
#define USE_FLASH_W25Q128FV
#define FLASH_OCTOSPI_INSTANCE  OCTOSPI1

#if defined(USE_FLASH_MEMORY_MAPPED)
// When the memory mapped flash is used the flash partitioning api must be aware of the firmware partition.
#define USE_FIRMWARE_PARTITION
#endif

#endif

#define ENABLE_BLACKBOX_LOGGING_ON_SPIFLASH_BY_DEFAULT

#define USE_SPI

#define USE_SPI_DEVICE_2
#define SPI2_SCK_PIN            PB13
#define SPI2_SDI_PIN            PB14
#define SPI2_SDO_PIN            PB15
#define SPI2_NSS_PIN            PB12

#define USE_SPI_DEVICE_3
#define SPI3_SCK_PIN            PC10
#define SPI3_SDI_PIN            PC11
#define SPI3_SDO_PIN            PC12
#define SPI3_NSS_PIN            PA15

#define USE_SPI_DEVICE_6
#define SPI6_SCK_PIN            PB3
#define SPI6_SDI_PIN            PB4
#define SPI6_SDO_PIN            PB5
#define SPI6_NSS_PIN            NONE

// Disable OCTOSPI pins PB2/CLK, PB10/NCS
// PE7/IO4, PE8/IO5, PE9/IO6, PE10/IO7

#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         (0xffff & ~(BIT(2)|BIT(10)))
#define TARGET_IO_PORTC         0xffff
#define TARGET_IO_PORTD         0xffff
#define TARGET_IO_PORTE         (0xffff & ~(BIT(7)|BIT(8)|BIT(9)|BIT(10)))
#define TARGET_IO_PORTF         0xffff
#define TARGET_IO_PORTG         0xffff
#define TARGET_IO_PORTH         0xffff

#define USE_I2C
#define USE_I2C_DEVICE_1
#define I2C1_SCL_PIN            PB6 // NC
#define I2C1_SDA_PIN            PB7 // NC

#define I2C2_SCL_PIN            NONE
#define I2C2_SDA_PIN            NONE

#define I2C3_SCL_PIN            NONE
#define I2C3_SDA_PIN            NONE

#define USE_I2C_DEVICE_4
#define I2C4_SCL_PIN            PD12 // J8:5
#define I2C4_SDA_PIN            PD13 // J8:6

#define MAG_I2C_INSTANCE        (I2CDEV_4)
#define BARO_I2C_INSTANCE       (I2CDEV_4)

#define ENSURE_MPU_DATA_READY_IS_LOW

#define DEFAULT_RX_FEATURE            FEATURE_RX_SERIAL
#define DEFAULT_FEATURES              (FEATURE_TELEMETRY | FEATURE_OSD | FEATURE_LED_STRIP)

#define ADC3_DMA_OPT                  10
#define ADC_INSTANCE                  ADC3

#define ADC1_INSTANCE                 ADC1
//#define ADC2_INSTANCE               ADC2 // ADC2 not used
#define ADC3_INSTANCE                 ADC3

// Hardware ADC pins and intended usage
#define CURRENT_METER_1_ADC_PIN       PC0 // ADC3_INP10
#define CURRENT_METER_1_ADC_INSTANCE  ADC3
#define CURRENT_METER_2_ADC_PIN       PC1 // ADC3_INP10
#define CURRENT_METER_2_ADC_INSTANCE  ADC3
#define ADC_RSSI_PIN                  PC2 // ADC3_INP0 - NOT CONNECTED
#define ADC_RSSI_INSTANCE             ADC3
#define ADC_VBAT_PIN                  PC3 // ADC3_INP1
#define ADC_VBAT_INSTANCE             ADC3

#define VIDEO_IN_ADC_PIN              PC5 // ADC1_INP4 - Reserved for video
#define VIDEO_OUT_ADC_PIN             PC4 // ADC1_INP8 - Reserved for video

// BF usage of ADC pins
#define ADC_CURR_PIN         CURRENT_METER_1_ADC_PIN
#define ADC_EXTERNAL1_PIN    CURRENT_METER_2_ADC_PIN

#define VTX_ENABLE_PIN                PB11
#define USE_PINIO
#define PINIO1_PIN                    VTX_ENABLE_PIN
#define USE_PINIOBOX

#define USE_ACC
#define USE_ACC_SPI_ICM42605
#define USE_ACC_SPI_ICM42688P
#define USE_GYRO
#define USE_GYRO_SPI_ICM42605
#define USE_GYRO_SPI_ICM42688P
#define USE_MULTI_GYRO
#define USE_BARO
#define USE_BARO_BMP388
#define USE_MAG
#define USE_MAG_HMC5883
#define USE_MAG_QMC5883

// The target has a connector and circuit for the LED strip.
#ifndef USE_LED_STRIP
#define USE_LED_STRIP
#endif

#ifndef USE_OSD
#define USE_OSD
#endif

#define USE_SPRACING_PIXEL_OSD
#define SPRACING_PIXEL_OSD_BLACK_PIN                    PE12
#define SPRACING_PIXEL_OSD_WHITE_PIN                    PE13
#define SPRACING_PIXEL_OSD_MASK_ENABLE_PIN              PE14
#define SPRACING_PIXEL_OSD_WHITE_SOURCE_SELECT_PIN      PE15

#define SPRACING_PIXEL_OSD_SYNC_IN_PIN                  PE11 // COMP2_INP
#define SPRACING_PIXEL_OSD_SYNC_OUT_PIN                 PA8  // TIM1_CH1

#define SPRACING_PIXEL_OSD_WHITE_SOURCE_PIN             PA4  // DAC1_OUT1
#define SPRACING_PIXEL_OSD_VIDEO_THRESHOLD_DEBUG_PIN    PA5  // DAC1_OUT2

#define SPRACING_PIXEL_OSD_PIXEL_DEBUG_1_PIN            PE5  // TIM15_CH1 - For DMA updates
#define SPRACING_PIXEL_OSD_PIXEL_DEBUG_2_PIN            PE6  // TIM15_CH2 - Spare
#define SPRACING_PIXEL_OSD_PIXEL_GATING_DEBUG_PIN       PB0 // TIM1_CH2N // actual gating is on CH4
#define SPRACING_PIXEL_OSD_PIXEL_BLANKING_DEBUG_PIN     PB1 // TIM1_CH3N // actual blanking is on CH5

#define BEEPER_INVERTED
#define BEEPER_PIN           PD11

#define MOTOR1_PIN           PA0
#define MOTOR2_PIN           PA1
#define MOTOR3_PIN           PA2
#define MOTOR4_PIN           PA3
#define MOTOR5_PIN           PA6
#define MOTOR6_PIN           PA7
#define MOTOR7_PIN           PB0
#define MOTOR8_PIN           PB1
#define LED_STRIP_PIN        PB8

#define UART1_TX_PIN         PB6 // NC
#define UART2_TX_PIN         PD5 // J8:8
#define UART3_TX_PIN         PD8 // Jumper to select between J14:3/4
#define UART4_TX_PIN         PD1 // J5:4 - FDCAN_TX
#define UART5_TX_PIN         NONE
#define UART6_TX_PIN         NONE
#define UART7_TX_PIN         NONE
#define UART8_TX_PIN         PE1 // NC
#define UART9_TX_PIN         PD15 // J11:4, J8:4
#define UART10_TX_PIN        PE3 // NC
#define UART1_RX_PIN         PB7 // NC
#define UART2_RX_PIN         PD6 // J8:7
#define UART3_RX_PIN         PD9 // NC
#define UART4_RX_PIN         PD0 // J5:3 - FDCAN_RX
#define UART5_RX_PIN         PD2   // J6:5, J9:5 ESC TLM
#define UART6_RX_PIN         NONE
#define UART7_RX_PIN         NONE
#define UART8_RX_PIN         PE0 // NC
#define UART9_RX_PIN         PD14 // J11:3, J8:3
#define UART10_RX_PIN        PE2 // NC

#define UART2_RTS_PIN        PD4 // J8:9
#define UART2_CTS_PIN        PD3 // J8:10

#define LED0_PIN             PE5
#define LED1_PIN             PE6
#define LED2_PIN             NONE

#define SPEKTRUM_RX_BIND_PIN NONE
#define RX_BIND_PLUG_PIN     NONE

//#define PINIO1_PIN           PC15
#define RX_SPI_CS_PIN        PB12
//#define RX_SPI_EXTI_PIN      PC6
#define RX_SPI_BIND_PIN      NONE
#define RX_SPI_LED_PIN       NONE
#define RX_SPI_EXPRESSLRS_RESET_PIN PD10
#define RX_SPI_EXPRESSLRS_BUSY_PIN PC7

#define GYRO_1_EXTI_PIN         PC6 // TIM8 CH1
#define GYRO_1_FSYNC_PIN        PC7 // TIM8 CH2
#define GYRO_2_EXTI_PIN         PC8 // TIM8 CH3
#define GYRO_2_FSYNC_PIN        PC9 // TIM8 CH4
#define GYRO_1_CS_PIN           SPI3_NSS_PIN
#define GYRO_1_SPI_INSTANCE     SPI3
#define GYRO_2_CS_PIN           SPI2_NSS_PIN
#define GYRO_2_SPI_INSTANCE     SPI2

#define GYRO_1_ALIGN            CW0_DEG
#define GYRO_2_ALIGN            CW180_DEG

#define GYRO_CONFIG_USE_GYRO_DEFAULT GYRO_CONFIG_USE_GYRO_BOTH

// Pre-BF-4.4 config:
// On 4in1ESC_1
// DEF_TIM(TIM5,  CH1, PA0,  TIM_USE_MOTOR,               0,  0,  0 ), // M1
// DEF_TIM(TIM5,  CH2, PA1,  TIM_USE_MOTOR,               0,  1,  0 ), // ..
// DEF_TIM(TIM5,  CH3, PA2,  TIM_USE_MOTOR,               0,  2,  0 ), // ..
// DEF_TIM(TIM5,  CH4, PA3,  TIM_USE_MOTOR,               0,  3,  0 ), // M4
//
// On 4in1ESC_2
// DEF_TIM(TIM3,  CH1, PA6,  TIM_USE_MOTOR,               0,  4,  0 ), // M5
// DEF_TIM(TIM3,  CH2, PA7,  TIM_USE_MOTOR,               0,  5,  0 ), // ..
// DEF_TIM(TIM3,  CH3, PB0,  TIM_USE_MOTOR,               0,  6,  0 ), // ..
// DEF_TIM(TIM3,  CH4, PB1,  TIM_USE_MOTOR,               0,  7,  0 ), // M8
//
// On GPS_IO1
// DEF_TIM(TIM4,  CH1, PD12, TIM_USE_MOTOR,               0,  8,  1 ), // M9  / I2C4_SCL
// DEF_TIM(TIM4,  CH2, PD13, TIM_USE_MOTOR,               0,  9,  1 ), // ..  / I2C4_SDA
// DEF_TIM(TIM4,  CH3, PD14, TIM_USE_MOTOR,               0,  10, 1 ), // ..  / UART9_RX
// DEF_TIM(TIM4,  CH4, PD15, TIM_USE_MOTOR,               0,  0, 0 ), // M12 / UART9_TX
//
// On B_L_IO2
// DEF_TIM(TIM16, CH1, PB8,  TIM_USE_LED,                 0,  12, 1 ), // LED Strip
// DEF_TIM(TIM17, CH1, PB9,  TIM_USE_CAMERA_CONTROL,      0,  13, 1 ), // Camera Control
//
// DEF_TIM(TIM15, CH1, PE5,  TIM_USE_VIDEO_PIXEL,         0,  15, 15 ), // Pixel DMA
// DEF_TIM(TIM1,  CH1, PA8,  TIM_USE_VIDEO_SYNC,          0,  14, 14 ), // Sync

// Index, Pin, 1-based occurrence of pin in fullTimerHardware, dma opt (use -1 for no DMA)
// See timerio.c and timer_stm32h7xx.c

#define TIMER_PIN_MAPPING \
    TIMER_PIN_MAP( 0, PA0 , 2,  0 ) \
    TIMER_PIN_MAP( 1, PA1 , 2,  1 ) \
    TIMER_PIN_MAP( 2, PA2 , 2,  2 ) \
    TIMER_PIN_MAP( 3, PA3 , 2,  3 ) \
    TIMER_PIN_MAP( 4, PA6 , 1,  4 ) \
    TIMER_PIN_MAP( 5, PA7 , 2,  5 ) \
    TIMER_PIN_MAP( 6, PB0 , 2,  6 ) \
    TIMER_PIN_MAP( 7, PB1 , 2,  7 ) \
    TIMER_PIN_MAP( 8, PB8 , 1,  8 ) \
    TIMER_PIN_MAP( 9, PB9 , 1,  9 )

#define PINIO1_BOX 40

#define DEFAULT_BLACKBOX_DEVICE     BLACKBOX_DEVICE_SDCARD

#define DEFAULT_CURRENT_METER_SOURCE CURRENT_METER_ADC
#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC

