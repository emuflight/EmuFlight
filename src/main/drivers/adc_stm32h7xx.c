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
#include "dma_reqmap.h"
#include "drivers/sensor.h"
#include "adc.h"
#include "adc_impl.h"
#include "pg/adc.h"

const adcDevice_t adcHardware[] = {
    { .ADCx = ADC1, .rccADC = RCC_AHB1(ADC12) },
    { .ADCx = ADC2, .rccADC = RCC_AHB1(ADC12) },
#if !(defined(STM32H7A3xx) || defined(STM32H7A3xxQ))
    { .ADCx = ADC3, .rccADC = RCC_AHB4(ADC3) },
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
#define RANK(n) ADC_REGULAR_RANK_ ## n

static const uint32_t adcRegularRankMap[] = {
    RANK(1), RANK(2), RANK(3), RANK(4), RANK(5), RANK(6), RANK(7), RANK(8),
    RANK(9), RANK(10), RANK(11), RANK(12), RANK(13), RANK(14), RANK(15), RANK(16),
};

#undef RANK

static adcDevice_t adcDevice[ADCDEV_COUNT];

static bool adcInitDevice(adcDevice_t *adcdev, int channelCount)
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
    // H723/H725/H730/H735: ADC3 uses DMAContinuousRequests instead of ConversionDataManagement.
#if defined(STM32H723xx) || defined(STM32H725xx) || defined(STM32H730xx) || defined(STM32H735xx)
    if (adcdev->ADCx == ADC3) {
        adcdev->ADCHandle.Init.DMAContinuousRequests = ENABLE;
    } else
#endif
    {
        adcdev->ADCHandle.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR;
    }
    adcdev->ADCHandle.Init.Overrun                   = ADC_OVR_DATA_OVERWRITTEN;
    adcdev->ADCHandle.Init.OversamplingMode          = DISABLE;
    if (HAL_ADC_Init(&adcdev->ADCHandle) != HAL_OK) {
        return false;
    }
    return HAL_ADCEx_Calibration_Start(&adcdev->ADCHandle, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED) == HAL_OK;
}

#ifdef ADC_INTERNAL_IN_SCAN

// H743/H750/H7A3 factory-calibrate VREFINT at 16-bit precision; H723/H725/H730 use 12-bit.
// ADC runs at 12-bit here, so shift 16-bit cal values right by 4 to match the live sample domain.
#if defined(STM32H743xx) || defined(STM32H750xx) || defined(STM32H7A3xx) || defined(STM32H7A3xxQ)
#define VREFINT_CAL_SHIFT 4
#elif defined(STM32H723xx) || defined(STM32H725xx) || defined(STM32H730xx) || defined(STM32H735xx)
#define VREFINT_CAL_SHIFT 0
#else
#error Unknown STM32H7 variant — add VREFINT_CAL_SHIFT definition
#endif

// The temperature sensor needs a long sample time (minimum 9 us); external inputs keep the shorter one.
#define ADC_SAMPLETIME_INTERNAL ADC_SAMPLETIME_810CYCLES_5

// Upper bound for the first scan of all ADC3 ranks, including two 810.5-cycle conversions.
#define ADC_FIRST_SCAN_TIMEOUT_US 10000

static void adcInitCalibrationValues(void)
{
    adcVREFINTCAL = *VREFINT_CAL_ADDR >> VREFINT_CAL_SHIFT;
    adcTSCAL1 = *TEMPSENSOR_CAL1_ADDR >> VREFINT_CAL_SHIFT;
    adcTSCAL2 = *TEMPSENSOR_CAL2_ADDR >> VREFINT_CAL_SHIFT;
    if (adcTSCAL2 != adcTSCAL1) {
        adcTSSlopeK = (TEMPSENSOR_CAL2_TEMP - TEMPSENSOR_CAL1_TEMP) * 1000 / (adcTSCAL2 - adcTSCAL1);
    } else {
        adcTSSlopeK = 0;
    }
}

// The scan runs continuously; there is no conversion to wait for or to start.
bool adcInternalIsBusy(void)
{
    return false;
}

void adcInternalStartConversion(void)
{
}

static uint16_t adcInternalRead(AdcChannel source)
{
    if (!adcOperatingConfig[source].enabled) {
        return 0;
    }
    SCB_InvalidateDCache_by_Addr((uint32_t *)adcValues, (sizeof(adcValues) + 31U) & ~31U);
    return adcValues[adcOperatingConfig[source].dmaIndex];
}

uint16_t adcInternalReadVrefint(void)
{
    return adcInternalRead(ADC_VREFINT);
}

uint16_t adcInternalReadTempsensor(void)
{
    return adcInternalRead(ADC_TEMPSENSOR);
}

// adcinternal.c divides by the first Vref sample, so it must not see the zero-filled buffer.
static void adcWaitForInternalSamples(void)
{
    const timeUs_t start = micros();
    while (cmpTimeUs(micros(), start) < ADC_FIRST_SCAN_TIMEOUT_US) {
        if (adcInternalReadVrefint() && adcInternalReadTempsensor()) {
            return;
        }
    }
}
#endif // ADC_INTERNAL_IN_SCAN

static void adcDisableDevice(ADCDevice dev)
{
    for (int i = 0; i < ADC_CHANNEL_COUNT; i++) {
        if (adcOperatingConfig[i].enabled && adcOperatingConfig[i].adcDevice == dev) {
            adcOperatingConfig[i].enabled = false;
        }
    }
}

void adcInit(const adcConfig_t *config)
{
    memset(&adcOperatingConfig, 0, sizeof(adcOperatingConfig));
    memset(adcDevice, 0, sizeof(adcDevice));
    memcpy(adcDevice, adcHardware, sizeof(adcHardware));

    if (config->vbat.enabled) {
        adcOperatingConfig[ADC_BATTERY].tag = config->vbat.ioTag;
        adcOperatingConfig[ADC_BATTERY].adcDevice = ADC_CFG_TO_DEV(config->vbat.device);
    }
    if (config->rssi.enabled) {
        adcOperatingConfig[ADC_RSSI].tag = config->rssi.ioTag;
        adcOperatingConfig[ADC_RSSI].adcDevice = ADC_CFG_TO_DEV(config->rssi.device);
    }
    if (config->external1.enabled) {
        adcOperatingConfig[ADC_EXTERNAL1].tag = config->external1.ioTag;
        adcOperatingConfig[ADC_EXTERNAL1].adcDevice = ADC_CFG_TO_DEV(config->external1.device);
    }
    if (config->current.enabled) {
        adcOperatingConfig[ADC_CURRENT].tag = config->current.ioTag;
        adcOperatingConfig[ADC_CURRENT].adcDevice = ADC_CFG_TO_DEV(config->current.device);
    }

#ifdef ADC_INTERNAL_IN_SCAN
    adcInitCalibrationValues();
#endif

    // An ADC is usable for a fallback only when its DMA option resolves to a stream.
    uint8_t usableDevices = 0;
    for (int dev = 0; dev < (int)ARRAYLEN(adcHardware); dev++) {
        if (dmaGetChannelSpecByPeripheral(DMA_PERIPH_ADC, dev, config->dmaopt[dev])) {
            usableDevices |= 1U << dev;
        }
    }

    for (int i = 0; i < ADC_CHANNEL_COUNT; i++) {
        adcOperatingConfig_t *input = &adcOperatingConfig[i];
        ADCDevice dev;
        uint32_t channel;
        uint8_t sampleTime;

#ifdef ADC_INTERNAL_IN_SCAN
        if (i >= ADC_CHANNEL_INTERNAL_FIRST_ID) {
            dev = ADCDEV_3;
            channel = (i == ADC_VREFINT) ? ADC_CHANNEL_VREFINT : ADC_CHANNEL_TEMPSENSOR;
            sampleTime = ADC_SAMPLETIME_INTERNAL;
        } else
#endif
        {
            if (!input->tag) {
                continue;
            }
            const adcTagMap_t *map = NULL;
            for (unsigned j = 0; j < ARRAYLEN(adcTagMap); j++) {
                if (adcTagMap[j].tag == input->tag) {
                    map = &adcTagMap[j];
                    break;
                }
            }
            if (!map) {
                continue;
            }
            dev = adcSelectDevice(map->devices, input->adcDevice, usableDevices);
            if (dev == ADCINVALID) {
                continue;
            }
            channel = map->channel;
            sampleTime = ADC_SAMPLETIME_387CYCLES_5;
        }

        input->adcDevice = dev;
        input->adcChannel = channel;
        input->sampleTime = sampleTime;
        input->enabled = true;
    }

    // Claim each stream before touching its ADC so a lost claim leaves that ADC and its pins untouched.
    uint8_t channelCount[ADCDEV_COUNT] = { 0 };
    for (int dev = 0; dev < ADCDEV_COUNT; dev++) {
        for (int i = 0; i < ADC_CHANNEL_COUNT; i++) {
            if (adcOperatingConfig[i].enabled && adcOperatingConfig[i].adcDevice == dev) {
                channelCount[dev]++;
            }
        }
        if (!channelCount[dev]) {
            continue;
        }

        const dmaChannelSpec_t *dmaSpec = adcDevice[dev].ADCx ? dmaGetChannelSpecByPeripheral(DMA_PERIPH_ADC, dev, config->dmaopt[dev]) : NULL;
        const dmaIdentifier_e dmaId = dmaSpec ? dmaGetIdentifier((DMA_Stream_TypeDef *)dmaSpec->ref) : DMA_NONE;
        if (!dmaSpec || !dmaAllocate(dmaId, OWNER_ADC, 0)) {
            adcDisableDevice(dev);
            channelCount[dev] = 0;
            continue;
        }
        // The spec is shared scratch; copy its fields before the next lookup overwrites them.
        adcDevice[dev].DmaHandle.Instance     = (DMA_Stream_TypeDef *)dmaSpec->ref;
        adcDevice[dev].DmaHandle.Init.Request = dmaSpec->channel;
        dmaEnable(dmaId);
    }

    // Each ADC's DMA writes consecutive samples, so inputs index the shared buffer device by device.
    uint8_t bufferOffset[ADCDEV_COUNT] = { 0 };
    uint8_t nextIndex = 0;
    for (int dev = 0; dev < ADCDEV_COUNT; dev++) {
        bufferOffset[dev] = nextIndex;
        for (int i = 0; i < ADC_CHANNEL_COUNT; i++) {
            if (adcOperatingConfig[i].enabled && adcOperatingConfig[i].adcDevice == dev) {
                adcOperatingConfig[i].dmaIndex = nextIndex++;
                if (adcOperatingConfig[i].tag) {
                    IOInit(IOGetByTag(adcOperatingConfig[i].tag), OWNER_ADC_BATT + i, 0);
                    IOConfigGPIO(IOGetByTag(adcOperatingConfig[i].tag), IO_CONFIG(GPIO_MODE_ANALOG, 0, GPIO_NOPULL));
                }
            }
        }
    }

    // Configure every ADC before starting any: internal channels cannot be set up once a sibling ADC is enabled.
    for (int dev = 0; dev < ADCDEV_COUNT; dev++) {
        adcDevice_t *adc = &adcDevice[dev];
        if (!channelCount[dev]) {
            continue;
        }

        RCC_ClockCmd(adc->rccADC, ENABLE);

        bool ok = adcInitDevice(adc, channelCount[dev]);

        unsigned rank = 0;
        for (int i = 0; ok && i < ADC_CHANNEL_COUNT; i++) {
            if (!adcOperatingConfig[i].enabled || adcOperatingConfig[i].adcDevice != dev) {
                continue;
            }

            ADC_ChannelConfTypeDef sConfig;
            memset(&sConfig, 0, sizeof(sConfig));
            sConfig.Channel      = adcOperatingConfig[i].adcChannel;
            sConfig.Rank         = adcRegularRankMap[rank++];
            sConfig.SamplingTime = adcOperatingConfig[i].sampleTime;
            sConfig.SingleDiff   = ADC_SINGLE_ENDED;
            sConfig.OffsetNumber = ADC_OFFSET_NONE;
            sConfig.Offset       = 0;
            ok = HAL_ADC_ConfigChannel(&adc->ADCHandle, &sConfig) == HAL_OK;
        }

        if (ok) {
            adc->DmaHandle.Init.Direction           = DMA_PERIPH_TO_MEMORY;
            adc->DmaHandle.Init.PeriphInc           = DMA_PINC_DISABLE;
            adc->DmaHandle.Init.MemInc              = channelCount[dev] > 1 ? DMA_MINC_ENABLE : DMA_MINC_DISABLE;
            adc->DmaHandle.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
            adc->DmaHandle.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
            adc->DmaHandle.Init.Mode                = DMA_CIRCULAR;
            adc->DmaHandle.Init.Priority            = DMA_PRIORITY_HIGH;
            adc->DmaHandle.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
            adc->DmaHandle.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
            adc->DmaHandle.Init.MemBurst            = DMA_MBURST_SINGLE;
            adc->DmaHandle.Init.PeriphBurst         = DMA_PBURST_SINGLE;
            ok = HAL_DMA_Init(&adc->DmaHandle) == HAL_OK;
        }

        if (ok) {
            __HAL_LINKDMA(&adc->ADCHandle, DMA_Handle, adc->DmaHandle);
        } else {
            adcDisableDevice(dev);
            channelCount[dev] = 0;
        }
    }

    for (int dev = 0; dev < ADCDEV_COUNT; dev++) {
        if (channelCount[dev] && HAL_ADC_Start_DMA(&adcDevice[dev].ADCHandle, (uint32_t *)&adcValues[bufferOffset[dev]], channelCount[dev]) != HAL_OK) {
            adcDisableDevice(dev);
        }
    }

#ifdef ADC_INTERNAL_IN_SCAN
    if (adcOperatingConfig[ADC_VREFINT].enabled && adcOperatingConfig[ADC_TEMPSENSOR].enabled) {
        adcWaitForInternalSamples();
    }
#endif
}

#endif // USE_ADC
