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

#ifdef USE_OSD_BEESIGN

#include "common/utils.h"

#include "drivers/beesign.h"
#include "drivers/time.h"
#include "drivers/display.h"

#include "fc/config.h"

#include "io/displayport_beesign.h"
#include "osd/osd.h"

#include "pg/vcd.h"

displayPort_t beesignDisplayPort;

static int grab(displayPort_t *displayPort)
{
    // FIXME this should probably not have a dependency on the OSD or OSD slave code
    UNUSED(displayPort);
#ifdef USE_OSD
    resumeRefreshAt = 0;
#endif

    return 0;
}

static int release(displayPort_t *displayPort)
{
    UNUSED(displayPort);

    return 0;
}

static int clearScreen(displayPort_t *displayPort)
{
    UNUSED(displayPort);

    bsClearScreenBuff();

    return 0;
}

static int drawScreen(displayPort_t *displayPort)
{
    UNUSED(displayPort);
    bsDisplay();
    return 0;
}

static int screenSize(const displayPort_t *displayPort)
{
    UNUSED(displayPort);
    return BEESIGN_CHARS_PER_SCREEN;
}

static int writeString(displayPort_t *displayPort, uint8_t x, uint8_t y, const char *s)
{
    UNUSED(displayPort);
    bsWriteBuffRow(x, y, s);

    return 0;
}

static int writeChar(displayPort_t *displayPort, uint8_t x, uint8_t y, uint8_t c)
{
    UNUSED(displayPort);
    bsWriteBuffChar(x, y, c);

    return 0;
}

static bool isTransferInProgress(const displayPort_t *displayPort)
{
    UNUSED(displayPort);
    return false;
}

static bool isSynced(const displayPort_t *displayPort)
{
    UNUSED(displayPort);
    return bsBuffersSynced();
}

static void resync(displayPort_t *displayPort)
{
    bsDisplay();
    UNUSED(displayPort);
    displayPort->rows = BEESIGN_LINES_PER_SCREEN;
    displayPort->cols = BEESIGN_CHARS_PER_LINE;
}

static int heartbeat(displayPort_t *displayPort)
{
    UNUSED(displayPort);
    return 0;
}

static uint32_t txBytesFree(const displayPort_t *displayPort)
{
    UNUSED(displayPort);
    return UINT32_MAX;
}

static const displayPortVTable_t beesignVTable = {
    .grab = grab,
    .release = release,
    .clearScreen = clearScreen,
    .drawScreen = drawScreen,
    .screenSize = screenSize,
    .writeString = writeString,
    .writeChar = writeChar,
    .isTransferInProgress = isTransferInProgress,
    .heartbeat = heartbeat,
    .resync = resync,
    .isSynced = isSynced,
    .txBytesFree = txBytesFree,
};

displayPort_t *beesignDisplayPortInit(const vcdProfile_t *vcdProfile)
{
    if (!checkBeesignSerialPort()) {
        return NULL;
    }

    bsSetOsdHosOffset(vcdProfile -> h_offset);
    bsSetOsdVosOffset(vcdProfile -> v_offset);
    bsSetOsdMode(BEESIGN_OSD_MODE_CUSTOM);
    delayMicroseconds(1000000);
    displayInit(&beesignDisplayPort, &beesignVTable);

    // resync(&beesignDisplayPort);
    return &beesignDisplayPort;
}
#endif // USE_OSD_BEESIGN
