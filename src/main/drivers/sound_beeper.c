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

#include "drivers/io.h"
#include "drivers/pwm_output.h"

#include "pg/beeper_dev.h"

#ifdef USE_BRAINFPV_FPGA
#include "fpga_drv.h"
#endif

#include "sound_beeper.h"


#ifndef USE_BRAINFPV_FPGA
static IO_t beeperIO = DEFIO_IO(NONE);
static bool beeperInverted = false;
static uint16_t beeperFrequency = 0;
#endif

void systemBeep(bool onoff)
{
#ifndef USE_BRAINFPV_FPGA
    if (beeperFrequency == 0) {
        IOWrite(beeperIO, beeperInverted ? onoff : !onoff);
    } else {
        pwmWriteBeeper(onoff);
    }
#else
    BRAINFPVFPGA_Buzzer(onoff);
#endif
}

void systemBeepToggle(void)
{
#ifndef USE_BRAINFPV_FPGA
    if (beeperFrequency == 0) {
        IOToggle(beeperIO);
    } else {
        pwmToggleBeeper();
    }
#else
     BRAINFPVFPGA_BuzzerToggle();
#endif
}

void beeperInit(const beeperDevConfig_t *config)
{
#ifdef USE_BRAINFPV_FPGA
    UNUSED(config);
#else
#ifdef USE_BEEPER
    beeperFrequency = config->frequency;
    if (beeperFrequency == 0) {
        beeperIO = IOGetByTag(config->ioTag);
        beeperInverted = config->isInverted;
        if (beeperIO) {
            IOInit(beeperIO, OWNER_BEEPER, 0);
            IOConfigGPIO(beeperIO, config->isOpenDrain ? IOCFG_OUT_OD : IOCFG_OUT_PP);
        }
        systemBeep(false);
    } else {
        const ioTag_t beeperTag = config->ioTag;
        beeperPwmInit(beeperTag, beeperFrequency);
    }
#else
    UNUSED(config);
#endif
#endif
}
