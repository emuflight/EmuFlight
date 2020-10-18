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

#include <string.h>

#include "platform.h"

#if defined(USE_PWM) || defined(USE_PPM)

#include "common/utils.h"

#include "config/feature.h"

#include "pg/rx.h"

#include "drivers/rx/rx_pwm.h"

#include "fc/config.h"

#include "rx/rx.h"
#include "rx/pwm.h"

static uint16_t pwmReadRawRC(const rxRuntimeConfig_t *rxRuntimeConfig, uint8_t channel) {
    UNUSED(rxRuntimeConfig);
    return pwmRead(channel);
}

static uint16_t ppmReadRawRC(const rxRuntimeConfig_t *rxRuntimeConfig, uint8_t channel) {
    UNUSED(rxRuntimeConfig);
    return ppmRead(channel);
}

void rxPwmInit(const rxConfig_t *rxConfig, rxRuntimeConfig_t *rxRuntimeConfig) {
    UNUSED(rxConfig);
    rxRuntimeConfig->rxRefreshRate = 20000;
    // configure PWM/CPPM read function and max number of channels. serial rx below will override both of these, if enabled
    if (feature(FEATURE_RX_PARALLEL_PWM)) {
        rxRuntimeConfig->channelCount = MAX_SUPPORTED_RC_PARALLEL_PWM_CHANNEL_COUNT;
        rxRuntimeConfig->rcReadRawFn = pwmReadRawRC;
    } else if (feature(FEATURE_RX_PPM)) {
        rxRuntimeConfig->channelCount = MAX_SUPPORTED_RC_PPM_CHANNEL_COUNT;
        rxRuntimeConfig->rcReadRawFn = ppmReadRawRC;
    }
}
#endif
