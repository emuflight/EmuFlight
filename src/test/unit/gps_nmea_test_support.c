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

#include "io/gps.c"

void testGpsNmeaReset(void)
{
    DISABLE_STATE(GPS_FIX);
    gpsSol.numSat = 0;
    gpsSol.llh.lat = 0;
    gpsSol.llh.lon = 0;
    gpsSol.groundSpeed = 0;
    gpsSol.groundCourse = 0;
    GPS_packetCount = 0;
    // End any half-fed frame: CR clears the checksum phase, '$' restarts the field state.
    gpsNewFrameNMEA('\r');
    gpsNewFrameNMEA('$');
    rtcClearForTest();
}

// Feed one NMEA text to the parser; return the count of frames it accepted.
unsigned testGpsNmeaFeed(const char *text)
{
    unsigned accepted = 0;
    while (*text) {
        if (gpsNewFrameNMEA(*text++)) {
            accepted++;
        }
    }
    return accepted;
}

uint32_t testGpsNmeaPacketCount(void)
{
    return GPS_packetCount;
}

bool testGpsNmeaFix(void)
{
    return STATE(GPS_FIX);
}

// RTC stub: keep the last time the parser stored, so tests read what it parsed.
static dateTime_t storedTime;
static bool storedValid;

void rtcClearForTest(void)
{
    storedValid = false;
}

bool rtcHasTime(void)
{
    return storedValid;
}

bool rtcSetDateTime(dateTime_t *dt)
{
    storedTime = *dt;
    storedValid = true;
    return true;
}

bool testGpsNmeaRtc(dateTime_t *dt)
{
    *dt = storedTime;
    return storedValid;
}
