/*
 * This file is part of EmuFlight.
 *
 * EmuFlight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * EmuFlight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include "interface/msp.c"

void testMspReboot(uint8_t mode)
{
    rebootMode = mode;
    mspRebootFn(NULL);
}

uint8_t testMspRebootFirmwareMode(void)
{
    return MSP_REBOOT_FIRMWARE;
}

mspResult_e testMspSetRtc(uint8_t *payload, int length)
{
    sbuf_t src;
    sbufInit(&src, payload, payload + length);
    return mspSetRtc(&src);
}
