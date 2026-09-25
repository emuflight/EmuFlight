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

#include "drivers/io.h"
#include "timer.h"
#ifdef USE_TIMER_MGMT
#include "pg/timerio.h"
#endif

uint8_t timerIndexByTag(ioTag_t ioTag) {
#ifdef USE_TIMER_MGMT
    for (unsigned i = 0; i < MAX_TIMER_PINMAP_COUNT; i++) {
        if (timerIOConfig(i)->ioTag == ioTag) {
            return timerIOConfig(i)->index;
        }
    }
#else
    UNUSED(ioTag);
#endif
    return 0;
}

#ifdef USE_TIMER_MGMT
const timerHardware_t *timerGetByTagAndIndex(ioTag_t ioTag, unsigned timerIndex) {
    if (!ioTag || !timerIndex) {
        return NULL;
    }
    uint8_t index = 1;
    for (unsigned i = 0; i < TIMER_CHANNEL_COUNT; i++) {
        if (TIMER_HARDWARE[i].tag == ioTag) {
            if (index == timerIndex) {
                return &TIMER_HARDWARE[i];
            }
            index++;
        }
    }
    return NULL;
}
#endif

const timerHardware_t *timerGetByTag(ioTag_t ioTag) {
    if (!ioTag) {
        return NULL;
    }
#ifdef USE_TIMER_MGMT
    return timerGetByTagAndIndex(ioTag, timerIndexByTag(ioTag));
#else
    for (int i = 0; i < (int)TIMER_CHANNEL_COUNT; i++) {
        if (TIMER_HARDWARE[i].tag == ioTag) {
            return &TIMER_HARDWARE[i];
        }
    }
    return NULL;
#endif
}

#ifdef USE_TIMER_MGMT
static resourceOwner_e timerOwners[MAX_TIMER_PINMAP_COUNT];
static uint8_t timerOwnerResourceIndices[MAX_TIMER_PINMAP_COUNT];

const timerHardware_t *timerAllocate(ioTag_t ioTag, resourceOwner_e owner, uint8_t resourceIndex) {
    if (!ioTag) {
        return NULL;
    }
    for (unsigned i = 0; i < MAX_TIMER_PINMAP_COUNT; i++) {
        if (timerIOConfig(i)->ioTag == ioTag) {
            if (timerOwners[i]) {
                return NULL;
            }
            const timerHardware_t *timer = timerGetByTag(ioTag);
            if (!timer) {
                return NULL;
            }
            timerOwners[i] = owner;
            timerOwnerResourceIndices[i] = resourceIndex;
            return timer;
        }
    }
    return NULL;
}

const timerHardware_t *timerGetAllocatedByNumberAndChannel(int8_t timerNumber, uint16_t timerChannel) {
    for (unsigned i = 0; i < MAX_TIMER_PINMAP_COUNT; i++) {
        const timerHardware_t *timer = timerGetByTagAndIndex(timerIOConfig(i)->ioTag, timerIOConfig(i)->index);
        if (timer && timerGetTIMNumber(timer->tim) == timerNumber && timer->channel == timerChannel && timerOwners[i]) {
            return timer;
        }
    }
    return NULL;
}

resourceOwner_e timerGetOwner(ioTag_t ioTag) {
    if (!ioTag) {
        return OWNER_FREE;
    }
    for (unsigned i = 0; i < MAX_TIMER_PINMAP_COUNT; i++) {
        if (timerIOConfig(i)->ioTag == ioTag) {
            return timerOwners[i];
        }
    }
    return OWNER_FREE;
}

uint8_t timerGetOwnerResourceIndex(ioTag_t ioTag) {
    if (!ioTag) {
        return 0;
    }
    for (unsigned i = 0; i < MAX_TIMER_PINMAP_COUNT; i++) {
        if (timerIOConfig(i)->ioTag == ioTag) {
            return timerOwnerResourceIndices[i];
        }
    }
    return 0;
}
#else
const timerHardware_t *timerAllocate(ioTag_t ioTag, resourceOwner_e owner, uint8_t resourceIndex) {
    UNUSED(owner);
    UNUSED(resourceIndex);
    return timerGetByTag(ioTag);
}

resourceOwner_e timerGetOwner(ioTag_t ioTag) {
    UNUSED(ioTag);
    return OWNER_FREE;
}

uint8_t timerGetOwnerResourceIndex(ioTag_t ioTag) {
    UNUSED(ioTag);
    return 0;
}
#endif

ioTag_t timerioTagGetByUsage(timerUsageFlag_e usageFlag, uint8_t index) {
    uint8_t currentIndex = 0;
    for (int i = 0; i < (int)USABLE_TIMER_CHANNEL_COUNT; i++) {
        if ((timerHardware[i].usageFlags & usageFlag) == usageFlag) {
            if (currentIndex == index) {
                return timerHardware[i].tag;
            }
            currentIndex++;
        }
    }
    return IO_TAG_NONE;
}
