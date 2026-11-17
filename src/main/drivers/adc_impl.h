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

#include "drivers/adc.h"
#include "drivers/dma.h"
#include "drivers/io_types.h"
#include "drivers/rcc_types.h"

#if defined(STM32F4) || defined(STM32F7)
#define ADC_TAG_MAP_COUNT 16
#elif defined(STM32H7)
#define ADC_TAG_MAP_COUNT 16
#elif defined(AT32F435)
#ifdef USE_ADC_INTERNAL
#define ADC_TAG_MAP_COUNT 18
#else
#define ADC_TAG_MAP_COUNT 16
#endif
#else
#define ADC_TAG_MAP_COUNT 10
#endif

typedef struct adcTagMap_s {
    ioTag_t tag;
    uint8_t devices;
    uint32_t channel;
#if defined(STM32H7) || defined(AT32F435)
    uint8_t channelOrdinal;
#endif
} adcTagMap_t;

// Encoding for adcTagMap_t.devices

#define ADC_DEVICES_1   (1 << ADCDEV_1)
#define ADC_DEVICES_2   (1 << ADCDEV_2)
#define ADC_DEVICES_3   (1 << ADCDEV_3)
#define ADC_DEVICES_12  ((1 << ADCDEV_1)|(1 << ADCDEV_2))
#define ADC_DEVICES_123 ((1 << ADCDEV_1)|(1 << ADCDEV_2)|(1 << ADCDEV_3))

typedef struct adcDevice_s {
    ADC_TypeDef* ADCx;
    rccPeriphTag_t rccADC;
#if defined(STM32H7)
    ADC_HandleTypeDef ADCHandle;
    DMA_HandleTypeDef DmaHandle;
    uint8_t irq;
    uint32_t channelBits;
#elif !(defined(STM32F4) || defined(STM32F7) || defined(AT32F435))
    DMA_Channel_TypeDef* DMAy_Channelx;
#endif
#if defined(STM32F7)
    ADC_HandleTypeDef ADCHandle;
    DMA_HandleTypeDef DmaHandle;
#endif
} adcDevice_t;

extern const adcDevice_t adcHardware[];
extern const adcTagMap_t adcTagMap[ADC_TAG_MAP_COUNT];
extern adcOperatingConfig_t adcOperatingConfig[ADC_CHANNEL_COUNT];
extern volatile uint16_t adcValues[ADC_CHANNEL_COUNT];

uint8_t adcChannelByTag(ioTag_t ioTag);
ADCDevice adcDeviceByInstance(ADC_TypeDef *instance);
bool adcVerifyPin(ioTag_t tag, ADCDevice device);

#ifdef AT32F435
#define VREFINT_EXPECTED                   (1489U)  // The raw ADC reading at 12bit resolution expected for the 1V2 internal ref
#define VREFINT_CAL_VREF                   (3300U)  // The nominal external Vref+ for the above reading
#define TEMPSENSOR_CAL_VREFANALOG          (3300U)
#define TEMPSENSOR_CAL1_TEMP               (25U)
#define TEMPSENSOR_CAL1_V                  (1.27f)
#define TEMPSENSOR_SLOPE                   (-4.13f) //  mV/C
#endif
