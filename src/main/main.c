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

#include "fc/fc_init.h"

#include "scheduler/scheduler.h"

void main_step(void)
{
    scheduler();
    processLoopback();
}

#ifndef NOMAIN
#if !defined(USE_CHIBIOS)
void run(void);

int main(void)
{
    init();

    run();

    return 0;
}

void FAST_CODE FAST_CODE_NOINLINE run(void)
{
    while (true) {
        scheduler();
        processLoopback();
#ifdef SIMULATOR_BUILD
        delayMicroseconds_real(50); // max rate 20kHz
#endif
    }
}
#endif

#if defined(USE_CHIBIOS)
#include "ch.h"
#include "hal.h"
#include "nvic.h"

volatile bool idleCounterClear = 0;
volatile uint32_t idleCounter = 0;

void appIdleHook(void)
{
    // Called when the scheduler has no tasks to run
    if (idleCounterClear) {
        idleCounter = 0;
        idleCounterClear = 0;
    } else {
        ++idleCounter;
    }
}

static THD_WORKING_AREA(waBetaFlightThread, 2 * 1024);
static THD_FUNCTION(BetaFlightThread, arg)
{
    (void)arg;
    chRegSetThreadName("Betaflight");
    while(1) {
        main_step();
    }
}

#if defined(USE_BRAINFPV_OSD)
#include "brainfpv/brainfpv_osd.h"
#include "drivers/display.h"
#include "io/displayport_max7456.h"
#include "pg/vcd.h"
#include "config/config_eeprom.h"
#include "config/feature.h"

void osdInit(displayPort_t *osdDisplayPortToUse);

static THD_WORKING_AREA(waOSDThread, 1 * 1024);
static THD_FUNCTION(OSDThread, arg)
{
    (void)arg;
    chRegSetThreadName("OSD");
    vcdProfile_t vcdProfile_ = {
        .video_system=VIDEO_SYSTEM_AUTO,
        .h_offset = 0,
        .v_offset = 0};
    displayPort_t *osdDisplayPort = max7456DisplayPortInit(&vcdProfile_);
    osdInit(osdDisplayPort);
    brainFpvOsdInit();
    osdMain();
}
#endif

#if defined(USE_BRAINFPV_SPECTROGRAPH)
#include "brainfpv/spectrograph.h"
extern binary_semaphore_t spectrographDataReadySemaphore;

static THD_WORKING_AREA(waSpecThread, 512);
static THD_FUNCTION(SpecThread, arg)
{
    (void)arg;
    chRegSetThreadName("Spectrograph");
    while (1) {
        // wait for data ready
        chBSemWait(&spectrographDataReadySemaphore);
        spectrographMain();
    }
}
#endif /* defined(USE_BRAINFPV_SPECTROGRAPH) */

uint8_t safe_boot = 0;

int main()
{
  halInit();
  chSysInit();

  init();

#if defined(USE_BRAINFPV_OSD)
  Video_Init();
#endif /* USE_BRAINFPV_OSD */

  /* re-init timer irq to make sure it works */
  //extern void *isr_vector_table_base;
  //NVIC_SetVectorTable((uint32_t)&isr_vector_table_base, 0x0);
  st_lld_init();

  chThdCreateStatic(waBetaFlightThread, sizeof(waBetaFlightThread), HIGHPRIO, BetaFlightThread, NULL);

#if defined(USE_BRAINFPV_OSD)
  chThdCreateStatic(waOSDThread, sizeof(waOSDThread), NORMALPRIO, OSDThread, NULL);
#endif /* USE_BRAINFPV_OSD */

#if defined(USE_BRAINFPV_SPECTROGRAPH)
  if (bfOsdConfig()->spec_enabled) {
    spectrographInit();
    chThdCreateStatic(waSpecThread, sizeof(waSpecThread), LOWPRIO, SpecThread, NULL);
  }
#endif /* USE_BRAINFPV_SPECTROGRAPH */

  // sleep forever
  chThdSleep(TIME_INFINITE);
}

#endif /* defined(USE_CHIBIOS) */
#endif /* NOMAIN */
