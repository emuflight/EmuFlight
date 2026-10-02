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

#include <stdbool.h>

#include "drivers/io_types.h"
#include "drivers/time.h"

#ifndef ADC_INSTANCE
#define ADC_INSTANCE                ADC1
#endif

// Per-device DMA option; a target.h overrides these. Meaning differs by MCU family:
// F4/F7 pick one of two fixed streams per ADC, H7 picks a pool stream (0-7 DMA1, 8-15 DMA2).
#if defined(STM32F4) || defined(STM32F7)
#ifndef ADC1_DMA_OPT
#define ADC1_DMA_OPT 1 // DMA2 ST4 (0: ST0)
#endif

#ifndef ADC2_DMA_OPT
#define ADC2_DMA_OPT 1 // DMA2 ST3 (0: ST2)
#endif

#ifndef ADC3_DMA_OPT
#define ADC3_DMA_OPT 0 // DMA2 ST0 (1: ST1)
#endif
#elif defined(STM32H7)
// DMA1 is reserved for timer-based DMA, so the defaults stay on DMA2.
#ifndef ADC1_DMA_OPT
#define ADC1_DMA_OPT 9 // DMA2 ST1
#endif

#ifndef ADC2_DMA_OPT
#define ADC2_DMA_OPT 10 // DMA2 ST2
#endif

#ifndef ADC3_DMA_OPT
#define ADC3_DMA_OPT 11 // DMA2 ST3
#endif
#endif

typedef enum ADCDevice {
    ADCINVALID = -1,
    ADCDEV_1   = 0,
#if defined(STM32F4) || defined(STM32F7) || defined(STM32H7)
    ADCDEV_2,
    ADCDEV_3,
#endif
    ADCDEV_COUNT
} ADCDevice;

#define ADC_CFG_TO_DEV(x) ((x) - 1)
#define ADC_DEV_TO_CFG(x) ((x) + 1)

// H7 reads VREFINT and the temperature sensor in the ADC3 DMA scan; H7A3 has no ADC3.
#if defined(STM32H7) && defined(USE_ADC_INTERNAL) && !(defined(STM32H7A3xx) || defined(STM32H7A3xxQ))
#define ADC_INTERNAL_IN_SCAN
#endif

typedef enum {
    ADC_BATTERY = 0,
    ADC_CURRENT = 1,
    ADC_EXTERNAL1 = 2,
    ADC_RSSI = 3,
#ifdef ADC_INTERNAL_IN_SCAN
    ADC_CHANNEL_INTERNAL_FIRST_ID = 4,
    ADC_TEMPSENSOR = 4,
    ADC_VREFINT = 5,
#endif
    ADC_CHANNEL_COUNT
} AdcChannel;

typedef struct adcOperatingConfig_s {
    ioTag_t tag;
#if defined(STM32H7)
    ADCDevice adcDevice;        // ADCDEV_x serving this input
    uint32_t adcChannel;        // H7 HAL channel constant, wider than 8 bits
#else
    uint8_t adcChannel;         // ADC1_INxx channel number
#endif
    uint8_t dmaIndex;           // index into DMA buffer in case of sparse channels
    bool enabled;
    uint8_t sampleTime;
} adcOperatingConfig_t;

#if defined(STM32H7)
// Keep the configured ADC when it can read the pin, else take the first usable ADC that can.
static inline ADCDevice adcSelectDevice(uint8_t pinDevices, ADCDevice configured, uint8_t usableDevices)
{
    if (configured >= 0 && configured < ADCDEV_COUNT && (pinDevices & (1U << configured))) {
        return configured;
    }
    for (int dev = 0; dev < ADCDEV_COUNT; dev++) {
        if ((pinDevices & usableDevices) & (1U << dev)) {
            return (ADCDevice)dev;
        }
    }
    return ADCINVALID;
}
#endif

struct adcConfig_s;
void adcInit(const struct adcConfig_s *config);
uint16_t adcGetChannel(uint8_t channel);

#ifdef USE_ADC_INTERNAL
extern uint16_t adcVREFINTCAL;
extern uint16_t adcTSCAL1;
extern uint16_t adcTSCAL2;
extern uint16_t adcTSSlopeK;

bool adcInternalIsBusy(void);
void adcInternalStartConversion(void);
uint16_t adcInternalReadVrefint(void);
uint16_t adcInternalReadTempsensor(void);
#endif

#if !defined(SIMULATOR_BUILD)
ADCDevice adcDeviceByInstance(ADC_TypeDef *instance);
#endif
