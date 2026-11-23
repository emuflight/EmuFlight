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
#include <stddef.h>
#include <stdint.h>

#include "platform.h"

#include "common/utils.h"

#include "drivers/exti.h"
#include "drivers/nvic.h"
#include "drivers/system.h"
#include "drivers/persistent.h"
#include "at32f435_437_clock.h"

// See RM_AT32F435_437_EN_V2.05.pdf reference manual table 5-6 for more info.
#if 256 < TARGET_FLASH_SIZE
#define USD_EOPB0_SRAM_CONFIG_MASK 0x7
#else
#define USD_EOPB0_SRAM_CONFIG_MASK 0x3
#endif

static flash_usd_eopb0_type get_sram_config(void)
{
    extern uint32_t _SRAM_SIZE; // Defined in linker file
    switch ((uint32_t)&_SRAM_SIZE) {
#if 256 == TARGET_FLASH_SIZE
    case 448:
        return FLASH_EOPB0_SRAM_448K;
    case 512:
        return FLASH_EOPB0_SRAM_512K;
    case 384:
    default:
        return FLASH_EOPB0_SRAM_384K;
#elif 448 == TARGET_FLASH_SIZE
    case 256:
        return FLASH_EOPB0_SRAM_256K;
    case 320:
        return FLASH_EOPB0_SRAM_320K;
    case 384:
        return FLASH_EOPB0_SRAM_384K;
    case 448:
        return FLASH_EOPB0_SRAM_448K;
    case 512:
        return FLASH_EOPB0_SRAM_512K;
    case 192:
    default:
        return FLASH_EOPB0_SRAM_192K;
#elif 1024 <= TARGET_FLASH_SIZE
    case 128:
        return FLASH_EOPB0_SRAM_128K;
    case 256:
        return FLASH_EOPB0_SRAM_256K;
    case 320:
        return FLASH_EOPB0_SRAM_320K;
    case 384:
        return FLASH_EOPB0_SRAM_384K;
    case 448:
        return FLASH_EOPB0_SRAM_448K;
    case 512:
        return FLASH_EOPB0_SRAM_512K;
    case 192:
    default:
        return FLASH_EOPB0_SRAM_192K;
#endif
    }
}

// Physical USD size: 512 bytes on 256 KB and 1024 KB parts, 4096 bytes on the others (RM V2.09 flash organization tables).
// Each data byte takes a halfword with its inverse, so Table 5-7 notes 1 and 2 count 256 and 2048 data bytes.
#if (TARGET_FLASH_SIZE == 256) || (TARGET_FLASH_SIZE == 1024)
#define USD_SIZE_BYTES 512
#else
#define USD_SIZE_BYTES 4096
#endif
#define USD_PROGRAM_TRIES 3

static bool updateSramConfig(flash_usd_eopb0_type sramCfg)
{
    // Erase clears every user system data field, so save and restore all but FAP (restored by the erase) and EOPB0.
    uint16_t saved[USD_SIZE_BYTES / sizeof(uint16_t)];
    volatile const uint16_t *usd = (volatile const uint16_t *)USD_BASE;
    const unsigned eopb0Index = offsetof(usd_type, eopb0) / sizeof(uint16_t);

    for (unsigned i = 0; i < ARRAYLEN(saved); i++) {
        saved[i] = usd[i];
    }

    flash_unlock();
    // The erase status may report only the FAP restore, so restoring the other fields does not depend on it.
    flash_user_system_data_erase();
    for (unsigned i = 1; i < ARRAYLEN(saved); i++) {
        if (i == eopb0Index || saved[i] == 0xFFFF) {
            continue;
        }
        // The hardware writes the inverse byte; the retry is bounded because a failed restore cannot be repaired after the reset.
        bool written = false;
        for (unsigned tries = 0; !written && tries < USD_PROGRAM_TRIES; tries++) {
            flash_flag_clear(FLASH_PRGMERR_FLAG | FLASH_EPPERR_FLAG | FLASH_ODF_FLAG);
            written = flash_user_system_data_program(USD_BASE + i * sizeof(uint16_t), saved[i] & 0xFF) == FLASH_OPERATE_DONE
                && (usd[i] & 0xFF) == (saved[i] & 0xFF);
        }
    }
    // The EOPB0 readback below decides success, so the status is not needed here.
    flash_eopb0_config(sramCfg);
    flash_lock();

    return ((USD->eopb0) & USD_EOPB0_SRAM_CONFIG_MASK) == sramCfg;
}

static void init_sram_config(void)
{
    // Make sure the SRAM config is correct
    const flash_usd_eopb0_type sram_cfg = get_sram_config();
    if (((USD->eopb0) & USD_EOPB0_SRAM_CONFIG_MASK) != sram_cfg) {
        // Reset only when the new value reads back, otherwise a failed update would reset forever.
        if (updateSramConfig(sram_cfg)) {
            systemReset();
        }
    }
}

void systemReset(void)
{
    __disable_irq();
    NVIC_SystemReset();
}

void systemResetToBootloader(bootloaderRequestType_e requestType)
{
    switch (requestType) {
    case BOOTLOADER_REQUEST_ROM:
    default:
        persistentObjectWrite(PERSISTENT_OBJECT_RESET_REASON, RESET_BOOTLOADER_REQUEST_ROM);

        break;
    }

    __disable_irq();
    NVIC_SystemReset();
}

typedef void resetHandler_t(void);

typedef struct isrVector_s {
    __I uint32_t    stackEnd;
    resetHandler_t *resetHandler;
} isrVector_t;

void checkForBootLoaderRequest(void)
{
    volatile uint32_t bootloaderRequest = persistentObjectRead(PERSISTENT_OBJECT_RESET_REASON);

    if (bootloaderRequest != RESET_BOOTLOADER_REQUEST_ROM) {
        return;
    }
    persistentObjectWrite(PERSISTENT_OBJECT_RESET_REASON, RESET_NONE);

    extern isrVector_t system_isr_vector_table_base;

    __set_MSP(system_isr_vector_table_base.stackEnd);
    system_isr_vector_table_base.resetHandler();
    while (1);
}

void enableGPIOPowerUsageAndNoiseReductions(void)
{
    // CRM_*_PERIPH_CLOCK values encode register and bit; enable each one on its own
    static const crm_periph_clock_type clocks[] = {
        CRM_GPIOA_PERIPH_CLOCK, CRM_GPIOB_PERIPH_CLOCK, CRM_GPIOC_PERIPH_CLOCK,
        CRM_GPIOD_PERIPH_CLOCK, CRM_GPIOE_PERIPH_CLOCK,
        CRM_DMA1_PERIPH_CLOCK, CRM_DMA2_PERIPH_CLOCK,
    };

    for (unsigned i = 0; i < sizeof(clocks) / sizeof(clocks[0]); i++) {
        crm_periph_clock_enable(clocks[i], TRUE);
    }
}

bool isMPUSoftReset(void)
{
    if (cachedRccCsrValue & CRM_REG_BIT(CRM_SW_RESET_FLAG))
        return true;
    else
        return false;
}

void systemInit(void)
{
    init_sram_config();

    persistentObjectInit();

    checkForBootLoaderRequest();

    system_clock_config();//config system clock to 288mhz usb 48mhz

    // Configure NVIC preempt/priority groups
    nvic_priority_group_config(NVIC_PRIORITY_GROUPING);

    // cache RCC->CSR value to use it in isMPUSoftReset() and others
    cachedRccCsrValue = CRM->ctrlsts;

    // Although VTOR is already loaded with a possible vector table in RAM,
    // removing the call to NVIC_SetVectorTable causes USB not to become active,
    extern uint8_t isr_vector_table_base;
    nvic_vector_table_set((uint32_t)&isr_vector_table_base, 0x0);

    crm_periph_clock_enable(CRM_OTGFS2_PERIPH_CLOCK, FALSE);
    crm_periph_clock_enable(CRM_OTGFS1_PERIPH_CLOCK, FALSE);

    CRM->ctrlsts_bit.rstfc = TRUE;

    enableGPIOPowerUsageAndNoiseReductions();

    // Init cycle counter
    cycleCounterInit();

    // SysTick
    SysTick_Config(system_core_clock / 1000);
}
