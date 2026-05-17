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

#include "platform.h"

#include "drivers/memprot.h"

// Defined in linker script
extern uint8_t dmaram_start;
extern uint8_t dmaram_end;

extern uint8_t dmarwaxi_start;
extern uint8_t dmarwaxi_end;

#if defined(USE_ITCM_RAM) || defined(USE_DMA_RAM)
mpuRegion_t mpuRegions[] = {
#ifdef USE_ITCM_RAM
    {
        //  Mark ITCM-RAM as read-only
        .start      = 0x00000000,
        .end        = 0,
        .size       = MPU_REGION_SIZE_64KB,
        .perm       = MPU_REGION_PRIV_RO_URO,
        .exec       = MPU_INSTRUCTION_ACCESS_ENABLE,
        .shareable  = MPU_ACCESS_NOT_SHAREABLE,
        .cacheable  = MPU_ACCESS_NOT_CACHEABLE,
        .bufferable = MPU_ACCESS_BUFFERABLE,
    },
#endif
#ifdef USE_DMA_RAM
    {
        .start      = (uint32_t)&dmaram_start,
        .end        = (uint32_t)&dmaram_end,
        .size       = 0,
        .perm       = MPU_REGION_FULL_ACCESS,
        .exec       = MPU_INSTRUCTION_ACCESS_ENABLE,
        .shareable  = MPU_ACCESS_SHAREABLE,
        .cacheable  = MPU_ACCESS_CACHEABLE,
        .bufferable = MPU_ACCESS_NOT_BUFFERABLE,
    },
    {
        .start      = (uint32_t)&dmarwaxi_start,
        .end        = (uint32_t)&dmarwaxi_end,
        .size       = 0,
        .perm       = MPU_REGION_FULL_ACCESS,
        .exec       = MPU_INSTRUCTION_ACCESS_ENABLE,
        .shareable  = MPU_ACCESS_NOT_SHAREABLE,
        .cacheable  = MPU_ACCESS_CACHEABLE,
        .bufferable = MPU_ACCESS_NOT_BUFFERABLE,
    },
#endif
};
unsigned mpuRegionCount = ARRAYLEN(mpuRegions);
STATIC_ASSERT(ARRAYLEN(mpuRegions) <= MAX_MPU_REGIONS, MPU_region_count_exceeds_limit);
#else
mpuRegion_t mpuRegions[1];
unsigned mpuRegionCount = 0;
#endif
