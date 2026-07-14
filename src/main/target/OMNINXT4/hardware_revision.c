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
#include <stdlib.h>

#include "platform.h"

#ifdef USE_HARDWARE_REVISION_DETECTION

#include "build/debug.h"

#include "drivers/adc_impl.h"
#include "drivers/io_types.h"
#include "drivers/io.h"
#include "drivers/io_impl.h"
#include "drivers/rcc.h"
#include "drivers/time.h"

#include "hardware_revision.h"

#undef DEBUG_HARDWARE_REVISION_ADC
#undef DEBUG_HARDWARE_REVISION_TABLE

uint8_t hardwareRevision = 0;

// Do ADC on IDDetectPin and determine revision
// If VREFINT is used, we can (probably) get a pretty good precision
// that we can distinguish tens of different voltages.

#define ADC_ID_DETECT_PIN PC2

typedef struct idDetect_s {
    uint32_t ratio;
    uint8_t revision;
} idDetect_t;

// To deploy the analog ID detection in production:
// - Need some theoretical evaluation and experimentation to determine
//   IDDET_ERROR value (ADC with VREFINT compensation is quite accurate).
// - Do some planning on revision numbering scheme.
// - Divider value planning for the scheme (separation).

#define IDDET_RATIO(highside, lowside) ((lowside) * 1000 / ((lowside) + (highside)))
#define IDDET_ERROR 12

static idDetect_t idDetectTable[] = {
    { IDDET_RATIO(10000, 10000), 1 },
};

ioTag_t idDetectTag;

#define VREFINT_CAL_ADDR  0x1FFF7A2A

static void adcIDDetectInit(void) {
    idDetectTag = IO_TAG(ADC_ID_DETECT_PIN);
    IOConfigGPIO(IOGetByTag(idDetectTag), IO_CONFIG(GPIO_Mode_AN, 0, GPIO_OType_OD, GPIO_PuPd_NOPULL));
    RCC_ClockCmd(RCC_APB2(ADC1), ENABLE);
    ADC_CommonInitTypeDef ADC_CommonInitStructure;
    ADC_CommonStructInit(&ADC_CommonInitStructure);
    ADC_CommonInitStructure.ADC_Mode             = ADC_Mode_Independent;
    ADC_CommonInitStructure.ADC_Prescaler        = ADC_Prescaler_Div8;
    ADC_CommonInitStructure.ADC_DMAAccessMode    = ADC_DMAAccessMode_Disabled;
    ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
    ADC_CommonInit(&ADC_CommonInitStructure);
    ADC_InitTypeDef ADC_InitStructure;
    ADC_StructInit(&ADC_InitStructure);
    ADC_InitStructure.ADC_ContinuousConvMode       = ENABLE;
    ADC_InitStructure.ADC_Resolution               = ADC_Resolution_12b;
    ADC_InitStructure.ADC_ExternalTrigConv         = ADC_ExternalTrigConv_T1_CC1;
    ADC_InitStructure.ADC_ExternalTrigConvEdge     = ADC_ExternalTrigConvEdge_None;
    ADC_InitStructure.ADC_DataAlign                = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfConversion          = 2; // Not used
    ADC_InitStructure.ADC_ScanConvMode             = ENABLE;
    ADC_Init(ADC1, &ADC_InitStructure);
    ADC_Cmd(ADC1, ENABLE);
    ADC_TempSensorVrefintCmd(ENABLE);
    delayMicroseconds(10); // Maximum startup time for internal sensors (DM00037051 5.3.22 & 24)
    uint32_t channel = adcChannelByTag(idDetectTag);
    ADC_InjectedDiscModeCmd(ADC1, DISABLE);
    ADC_InjectedSequencerLengthConfig(ADC1, 2);
    ADC_InjectedChannelConfig(ADC1, channel, 1, ADC_SampleTime_480Cycles);
    ADC_InjectedChannelConfig(ADC1, ADC_Channel_Vrefint, 2, ADC_SampleTime_480Cycles);
}

static void adcIDDetectDeinit(void) {
    ADC_Cmd(ADC1, DISABLE);
    ADC_DeInit();
    IOConfigGPIO(IOGetByTag(idDetectTag), IOCFG_IPU);
}

static void adcIDDetectStart(void) {
    ADC_ClearFlag(ADC1, ADC_FLAG_JEOC);
    ADC_SoftwareStartInjectedConv(ADC1);
}

static void adcIDDetectWait(void) {
    while (ADC_GetFlagStatus(ADC1, ADC_FLAG_JEOC) == RESET) {
        // Empty
    }
}

static uint16_t adcIDDetectReadIDDet(void) {
    return ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_1);
}

static uint16_t adcIDDetectReadVrefint(void) {
    return ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_2);
}

void detectHardwareRevision(void) {
    adcIDDetectInit();
    uint32_t vrefintValue = 0;
    uint32_t iddetValue = 0;
    for (int i = 0 ; i < 16 ; i++) {
        adcIDDetectStart();
        adcIDDetectWait();
        iddetValue += adcIDDetectReadIDDet();
        vrefintValue += adcIDDetectReadVrefint();
    }
    vrefintValue /= 16;
    iddetValue /= 16;
    uint32_t iddetRatio = (iddetValue * vrefintValue) / *(uint16_t *)VREFINT_CAL_ADDR;
    iddetRatio = iddetRatio * 1000 / 4096;
#ifdef DEBUG_HARDWARE_REVISION_ADC
    debug[0] = *(uint16_t *)VREFINT_CAL_ADDR;
    debug[1] = vrefintValue;
    debug[2] = iddetValue;
    debug[3] = iddetRatio;
#endif
    for (size_t entry = 0; entry < ARRAYLEN(idDetectTable); entry++) {
#ifdef DEBUG_HARDWARE_REVISION_TABLE
        debug[0] = iddetRatio;
        debug[1] = idDetectTable[entry].ratio - IDDET_ERROR;
        debug[2] = idDetectTable[entry].ratio + IDDET_ERROR;
#endif
        if (idDetectTable[entry].ratio - IDDET_ERROR < iddetRatio && iddetRatio < idDetectTable[entry].ratio + IDDET_ERROR) {
            hardwareRevision = idDetectTable[entry].revision;
            break;
        }
    }
    adcIDDetectDeinit();
}

void updateHardwareRevision(void) {
    // Empty
}

ioTag_t selectMPUIntExtiConfigByHardwareRevision(void) {
    return IO_TAG_NONE;
}
#endif
