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

#include "config/feature.h"
#include "pg/pg.h"
#include "pg/pg_ids.h"


static uint32_t activeFeaturesLatch = 0;

PG_REGISTER_WITH_RESET_TEMPLATE(featureConfig_t, featureConfig, PG_FEATURE_CONFIG, 0);

#ifndef USE_GYRO_IMUF9001
PG_RESET_TEMPLATE(featureConfig_t, featureConfig,
                  .enabledFeatures = DEFAULT_FEATURES | DEFAULT_RX_FEATURE | FEATURE_AIRMODE | FEATURE_DYNAMIC_FILTER,
                 );
#endif

#ifdef USE_GYRO_IMUF9001
PG_RESET_TEMPLATE(featureConfig_t, featureConfig,
                  .enabledFeatures = DEFAULT_FEATURES | DEFAULT_RX_FEATURE | FEATURE_AIRMODE,
                 );
#endif

void intFeatureSet(uint32_t mask, uint32_t *features) {
    *features |= mask;
}

void intFeatureClear(uint32_t mask, uint32_t *features) {
    *features &= ~(mask);
}

void intFeatureClearAll(uint32_t *features) {
    *features = 0;
}

void latchActiveFeatures(void) {
    activeFeaturesLatch = featureConfig()->enabledFeatures;
}

bool featureConfigured(uint32_t mask) {
    return featureConfig()->enabledFeatures & mask;
}

bool feature(uint32_t mask) {
    return activeFeaturesLatch & mask;
}

void featureSet(uint32_t mask) {
    intFeatureSet(mask, &featureConfigMutable()->enabledFeatures);
}

void featureClear(uint32_t mask) {
    intFeatureClear(mask, &featureConfigMutable()->enabledFeatures);
}

void featureClearAll(void) {
    intFeatureClearAll(&featureConfigMutable()->enabledFeatures);
}

uint32_t featureMask(void) {
    return featureConfig()->enabledFeatures;
}
