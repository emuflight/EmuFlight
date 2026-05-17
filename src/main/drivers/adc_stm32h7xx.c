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

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#ifdef USE_ADC

#include "drivers/accgyro/accgyro.h"
#include "drivers/system.h"
#include "drivers/io.h"
#include "io_impl.h"
#include "rcc.h"
#include "dma.h"
#include "drivers/sensor.h"
#include "adc.h"
#include "adc_impl.h"
#include "pg/adc.h"

#ifndef ADC_INSTANCE
#define ADC_INSTANCE ADC1
#endif

// Default DMA streams; targets can override via target.h
#ifndef ADC1_DMA_STREAM
#define ADC1_DMA_STREAM DMA1_Stream1
#endif
#ifndef ADC2_DMA_STREAM
#define ADC2_DMA_STREAM DMA1_Stream2
#endif
#ifndef ADC3_DMA_STREAM
#define ADC3_DMA_STREAM DMA1_Stream3
#endif

const adcDevice_t adcHardware[] = {
    { .ADCx = ADC1, .rccADC = RCC_AHB1(ADC12), .dmaResource = (dmaResource_t *)ADC1_DMA_STREAM, .channel = DMA_REQUEST_ADC1 },
    { .ADCx = ADC2, .rccADC = RCC_AHB1(ADC12), .dmaResource = (dmaResource_t *)ADC2_DMA_STREAM, .channel = DMA_REQUEST_ADC2 },
#if !(defined(STM32H7A3xx) || defined(STM32H7A3xxQ))
    { .ADCx = ADC3, .rccADC = RCC_AHB4(ADC3),  .dmaResource = (dmaResource_t *)ADC3_DMA_STREAM, .channel = DMA_REQUEST_ADC3 },
#endif
};

// H743 pin-to-ADC channel mappings (RM0433 Table 205)
const adcTagMap_t adcTagMap[] = {
    { DEFIO_TAG_E__PC0,  ADC_DEVICES_123, ADC_CHANNEL_10, 10 },
    { DEFIO_TAG_E__PC1,  ADC_DEVICES_123, ADC_CHANNEL_11, 11 },
    { DEFIO_TAG_E__PC2,  ADC_DEVICES_3,   ADC_CHANNEL_0,   0 },
    { DEFIO_TAG_E__PC3,  ADC_DEVICES_3,   ADC_CHANNEL_1,   1 },
    { DEFIO_TAG_E__PC4,  ADC_DEVICES_12,  ADC_CHANNEL_4,   4 },
    { DEFIO_TAG_E__PC5,  ADC_DEVICES_12,  ADC_CHANNEL_8,   8 },
    { DEFIO_TAG_E__PB0,  ADC_DEVICES_12,  ADC_CHANNEL_9,   9 },
    { DEFIO_TAG_E__PB1,  ADC_DEVICES_12,  ADC_CHANNEL_5,   5 },
    { DEFIO_TAG_E__PA0,  ADC_DEVICES_1,   ADC_CHANNEL_16, 16 },
    { DEFIO_TAG_E__PA1,  ADC_DEVICES_1,   ADC_CHANNEL_17, 17 },
    { DEFIO_TAG_E__PA2,  ADC_DEVICES_12,  ADC_CHANNEL_14, 14 },
    { DEFIO_TAG_E__PA3,  ADC_DEVICES_12,  ADC_CHANNEL_15, 15 },
    { DEFIO_TAG_E__PA4,  ADC_DEVICES_12,  ADC_CHANNEL_18, 18 },
    { DEFIO_TAG_E__PA5,  ADC_DEVICES_12,  ADC_CHANNEL_19, 19 },
    { DEFIO_TAG_E__PA6,  ADC_DEVICES_12,  ADC_CHANNEL_3,   3 },
    { DEFIO_TAG_E__PA7,  ADC_DEVICES_12,  ADC_CHANNEL_7,   7 },
};

// Map 0-based rank index to HAL ADC_REGULAR_RANK_x constant
static const uint32_t adcRegularRankMap[] = {
    ADC_REGULAR_RANK_1,
    ADC_REGULAR_RANK_2,
    ADC_REGULAR_RANK_3,
    ADC_REGULAR_RANK_4,
};

static void adcInitDevice(adcDevice_t *adcdev, int channelCount)
{
    adcdev->ADCHandle.Instance                       = adcdev->ADCx;
    adcdev->ADCHandle.Init.ClockPrescaler            = ADC_CLOCK_ASYNC_DIV2;
    adcdev->ADCHandle.Init.Resolution                = ADC_RESOLUTION_12B;
    adcdev->ADCHandle.Init.ScanConvMode              = ENABLE;
    adcdev->ADCHandle.Init.EOCSelection              = ADC_EOC_SINGLE_CONV;
    adcdev->ADCHandle.Init.LowPowerAutoWait          = DISABLE;
    adcdev->ADCHandle.Init.ContinuousConvMode        = ENABLE;
    adcdev->ADCHandle.Init.NbrOfConversion           = channelCount;
    adcdev->ADCHandle.Init.DiscontinuousConvMode     = DISABLE;
    adcdev->ADCHandle.Init.NbrOfDiscConversion       = 1;
    adcdev->ADCHandle.Init.ExternalTrigConv          = ADC_SOFTWARE_START;
    adcdev->ADCHandle.Init.ExternalTrigConvEdge      = ADC_EXTERNALTRIGCONVEDGE_NONE;
    adcdev->ADCHandle.Init.ConversionDataManagement  = ADC_CONVERSIONDATA_DMA_CIRCULAR;
    adcdev->ADCHandle.Init.Overrun                   = ADC_OVR_DATA_OVERWRITTEN;
    adcdev->ADCHandle.Init.OversamplingMode          = DISABLE;
    if (HAL_ADC_Init(&adcdev->ADCHandle) != HAL_OK) {
        return;
    }
    HAL_ADCEx_Calibration_Start(&adcdev->ADCHandle, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
}

static adcDevice_t adc;

void adcInit(const adcConfig_t *config)
{
    memset(&adcOperatingConfig, 0, sizeof(adcOperatingConfig));

    if (config->vbat.enabled) {
        adcOperatingConfig[ADC_BATTERY].tag = config->vbat.ioTag;
    }
    if (config->rssi.enabled) {
        adcOperatingConfig[ADC_RSSI].tag = config->rssi.ioTag;
    }
    if (config->external1.enabled) {
        adcOperatingConfig[ADC_EXTERNAL1].tag = config->external1.ioTag;
    }
    if (config->current.enabled) {
        adcOperatingConfig[ADC_CURRENT].tag = config->current.ioTag;
    }

    ADCDevice device = adcDeviceByInstance(ADC_INSTANCE);
    if (device == ADCINVALID) {
        return;
    }
    adc = adcHardware[device];

    bool adcActive = false;
    uint8_t configuredAdcChannels = 0;

    for (int i = 0; i < ADC_CHANNEL_COUNT; i++) {
        if (!adcVerifyPin(adcOperatingConfig[i].tag, device)) {
            continue;
        }
        adcActive = true;
        IOInit(IOGetByTag(adcOperatingConfig[i].tag), OWNER_ADC_BATT + i, 0);
        IOConfigGPIO(IOGetByTag(adcOperatingConfig[i].tag), IO_CONFIG(GPIO_MODE_ANALOG, 0, GPIO_NOPULL));
        // adcChannel holds tagmap index — full 32-bit H7 channel constant lives in adcTagMap[].channel
        adcOperatingConfig[i].adcChannel = adcChannelByTag(adcOperatingConfig[i].tag);
        adcOperatingConfig[i].dmaIndex   = configuredAdcChannels++;
        adcOperatingConfig[i].sampleTime = ADC_SAMPLETIME_387CYCLES_5;
        adcOperatingConfig[i].enabled    = true;
    }

    if (!adcActive) {
        return;
    }

    // Enable ADC12 kernel clock (HAL doesn't do this automatically)
    __HAL_RCC_ADC12_CLK_ENABLE();

    adcInitDevice(&adc, configuredAdcChannels);

    uint8_t rank = 0;
    for (int i = 0; i < ADC_CHANNEL_COUNT; i++) {
        if (!adcOperatingConfig[i].enabled) {
            continue;
        }

        // Look up the full 32-bit H7 channel constant from tagmap
        uint32_t h7channel = 0;
        for (int j = 0; j < ADC_TAG_MAP_COUNT; j++) {
            if (adcTagMap[j].tag == adcOperatingConfig[i].tag) {
                h7channel = adcTagMap[j].channel;
                break;
            }
        }

        ADC_ChannelConfTypeDef sConfig;
        sConfig.Channel      = h7channel;
        sConfig.Rank         = adcRegularRankMap[rank++];
        sConfig.SamplingTime = adcOperatingConfig[i].sampleTime;
        sConfig.SingleDiff   = ADC_SINGLE_ENDED;
        sConfig.OffsetNumber = ADC_OFFSET_NONE;
        sConfig.Offset       = 0;
        if (HAL_ADC_ConfigChannel(&adc.ADCHandle, &sConfig) != HAL_OK) {
            return;
        }
    }

    dmaIdentifier_e dmaId = dmaGetIdentifier((DMA_Stream_TypeDef *)adc.dmaResource);
    dmaInit(dmaId, OWNER_ADC, 0);

    adc.DmaHandle.Instance               = (DMA_Stream_TypeDef *)adc.dmaResource;
    adc.DmaHandle.Init.Request           = adc.channel;
    adc.DmaHandle.Init.Direction         = DMA_PERIPH_TO_MEMORY;
    adc.DmaHandle.Init.PeriphInc         = DMA_PINC_DISABLE;
    adc.DmaHandle.Init.MemInc            = configuredAdcChannels > 1 ? DMA_MINC_ENABLE : DMA_MINC_DISABLE;
    adc.DmaHandle.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    adc.DmaHandle.Init.MemDataAlignment  = DMA_MDATAALIGN_HALFWORD;
    adc.DmaHandle.Init.Mode              = DMA_CIRCULAR;
    adc.DmaHandle.Init.Priority          = DMA_PRIORITY_HIGH;
    adc.DmaHandle.Init.FIFOMode          = DMA_FIFOMODE_DISABLE;
    adc.DmaHandle.Init.FIFOThreshold     = DMA_FIFO_THRESHOLD_FULL;
    adc.DmaHandle.Init.MemBurst          = DMA_MBURST_SINGLE;
    adc.DmaHandle.Init.PeriphBurst       = DMA_PBURST_SINGLE;

    if (HAL_DMA_Init(&adc.DmaHandle) != HAL_OK) {
        return;
    }
    __HAL_LINKDMA(&adc.ADCHandle, DMA_Handle, adc.DmaHandle);

    if (HAL_ADC_Start_DMA(&adc.ADCHandle, (uint32_t *)&adcValues, configuredAdcChannels) != HAL_OK) {
        return;
    }
}

#endif // USE_ADC
