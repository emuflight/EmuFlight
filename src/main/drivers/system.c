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

#include "platform.h"

#include "build/atomic.h"

#include "drivers/light_led.h"
#include "drivers/nvic.h"
#include "drivers/sound_beeper.h"

#include "system.h"

// cycles per microsecond
static uint32_t usTicks = 0;
// current uptime for 1kHz systick timer. will rollover after 49 days. hopefully we won't care.
static volatile uint32_t sysTickUptime = 0;
static volatile uint32_t sysTickValStamp = 0;
// cached value of RCC->CSR
uint32_t cachedRccCsrValue;

void cycleCounterInit(void) {
#if defined(USE_HAL_DRIVER)
    usTicks = HAL_RCC_GetSysClockFreq() / 1000000;
#else
    RCC_ClocksTypeDef clocks;
    RCC_GetClocksFreq(&clocks);
    usTicks = clocks.SYSCLK_Frequency / 1000000;
#endif
}

// SysTick

static volatile int sysTickPending = 0;

void SysTick_Handler(void) {
    ATOMIC_BLOCK(NVIC_PRIO_MAX) {
        sysTickUptime++;
        sysTickValStamp = SysTick->VAL;
        sysTickPending = 0;
        (void)(SysTick->CTRL);
    }
#ifdef USE_HAL_DRIVER
    // used by the HAL for some timekeeping and timeouts, should always be 1ms
    HAL_IncTick();
#endif
}

// Return system uptime in microseconds (rollover in 70minutes)

uint32_t microsISR(void) {
    register uint32_t ms, pending, cycle_cnt;
    ATOMIC_BLOCK(NVIC_PRIO_MAX) {
        cycle_cnt = SysTick->VAL;
        if (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) {
            // Update pending.
            // Record it for multiple calls within the same rollover period
            // (Will be cleared when serviced).
            // Note that multiple rollovers are not considered.
            sysTickPending = 1;
            // Read VAL again to ensure the value is read after the rollover.
            cycle_cnt = SysTick->VAL;
        }
        ms = sysTickUptime;
        pending = sysTickPending;
    }
    return ((ms + pending) * 1000) + (usTicks * 1000 - cycle_cnt) / usTicks;
}

uint32_t micros(void) {
    register uint32_t ms, cycle_cnt;
    // Call microsISR() in interrupt and elevated (non-zero) BASEPRI context
    if ((SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk) || (__get_BASEPRI())) {
        return microsISR();
    }
    do {
        ms = sysTickUptime;
        cycle_cnt = SysTick->VAL;
    } while (ms != sysTickUptime || cycle_cnt > sysTickValStamp);
    return (ms * 1000) + (usTicks * 1000 - cycle_cnt) / usTicks;
}

// Return system uptime in milliseconds (rollover in 49 days)
uint32_t millis(void) {
    return sysTickUptime;
}

#if 1
void delayMicroseconds(uint32_t us) {
    uint32_t now = micros();
    while (micros() - now < us);
}
#else
void delayMicroseconds(uint32_t us) {
    uint32_t elapsed = 0;
    uint32_t lastCount = SysTick->VAL;
    for (;;) {
        register uint32_t current_count = SysTick->VAL;
        uint32_t elapsed_us;
        // measure the time elapsed since the last time we checked
        elapsed += current_count - lastCount;
        lastCount = current_count;
        // convert to microseconds
        elapsed_us = elapsed / usTicks;
        if (elapsed_us >= us)
            break;
        // reduce the delay by the elapsed time
        us -= elapsed_us;
        // keep fractional microseconds for the next iteration
        elapsed %= usTicks;
    }
}
#endif

void delay(uint32_t ms) {
    while (ms--)
        delayMicroseconds(1000);
}

static void indicate(uint8_t count, uint16_t duration) {
    if (count) {
        LED1_ON;
        LED0_OFF;
        while (count--) {
            LED1_TOGGLE;
            LED0_TOGGLE;
            BEEP_ON;
            delay(duration);
            LED1_TOGGLE;
            LED0_TOGGLE;
            BEEP_OFF;
            delay(duration);
        }
    }
}

void indicateFailure(failureMode_e mode, int codeRepeatsRemaining) {
    while (codeRepeatsRemaining--) {
        indicate(WARNING_FLASH_COUNT, WARNING_FLASH_DURATION_MS);
        delay(WARNING_PAUSE_DURATION_MS);
        indicate(mode + 1, WARNING_CODE_DURATION_LONG_MS);
        delay(1000);
    }
}

void failureMode(failureMode_e mode) {
    indicateFailure(mode, 10);
#ifdef DEBUG
    systemReset();
#else
    systemResetToBootloader();
#endif
}
