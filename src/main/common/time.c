/*
 * This file is part of INAV.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 * Alternatively, the contents of this file may be used under the terms
 * of the GNU General Public License Version 3, as described below:
 *
 * This file is free software: you may copy, redistribute and/or modify
 * it under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or (at your
 * option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see http://www.gnu.org/licenses/.
 *
 * @author Alberto Garcia Hierro <alberto@garciahierro.com>
 */

#include <stdint.h>

#include "common/maths.h"
#include "common/printf.h"
#include "common/time.h"

#include "pg/pg_ids.h"

#include "drivers/time.h"

#ifdef USE_RTC_TIME

// For the "modulo 4" arithmetic to work, we need a leap base year
#define REFERENCE_YEAR 2000
// Offset (seconds) from the UNIX epoch (1970-01-01) to 2000-01-01
#define EPOCH_2000_OFFSET 946684800
// Last supported year. The day table covers 4-year cycles from 2000 and is exact up to 2099.
#define LAST_YEAR (REFERENCE_YEAR + 99)
// Supported rtcTime_t range in milliseconds: 2000-01-01T00:00:00.000 up to 2100-01-01T00:00:00.000 (exclusive).
// 100 years from 2000 hold 36525 days (25 leap years).
#define RTC_TIME_MIN ((rtcTime_t)EPOCH_2000_OFFSET * 1000)
#define RTC_TIME_END (((rtcTime_t)EPOCH_2000_OFFSET + 36525LL * 86400LL) * 1000)

#define MILLIS_PER_SECOND 1000

// rtcTime_t when the system was started.
// Calculated in rtcSet().
static rtcTime_t started = 0;

static const uint16_t days[4][12] = {
    {   0,  31,     60,     91,     121,    152,    182,    213,    244,    274,    305,    335},
    { 366,  397,    425,    456,    486,    517,    547,    578,    609,    639,    670,    700},
    { 731,  762,    790,    821,    851,    882,    912,    943,    974,    1004,   1035,   1065},
    {1096,  1127,   1155,   1186,   1216,   1247,   1277,   1308,   1339,   1369,   1400,   1430},
};

PG_REGISTER_WITH_RESET_TEMPLATE(timeConfig_t, timeConfig, PG_TIME_CONFIG, 0);

PG_RESET_TEMPLATE(timeConfig_t, timeConfig,
                  .tz_offsetMinutes = 0,
                 );

static void rtcGetDefaultDateTime(dateTime_t *dateTime);

static bool rtcIsDateTimeValid(const dateTime_t *dateTime) {
    if (!((dateTime->year >= REFERENCE_YEAR && dateTime->year <= LAST_YEAR) &&
          (dateTime->month >= 1 && dateTime->month <= 12) &&
          (dateTime->day >= 1 && dateTime->day <= 31) &&
          (dateTime->hours <= 23) &&
          (dateTime->minutes <= 59) &&
          (dateTime->seconds <= 59) &&
          (dateTime->millis <= 999))) {
        return false;
    }
    // Month length from the day table, leap years included (exact for 2000 to 2099).
    unsigned int year = dateTime->year - REFERENCE_YEAR;
    unsigned int month = dateTime->month - 1;
    unsigned int monthLength = (month == 11) ? 31 : days[year % 4][month + 1] - days[year % 4][month];
    return dateTime->day <= monthLength;
}

// Convert a calendar date to rtcTime_t. Returns false and leaves *t unchanged when the date
// is outside 2000-01-01 to 2099-12-31 or has a field out of range. The day table below is
// exact only in that range: 2100 is not a leap year.
static bool dateTimeToRtcTime(const dateTime_t *dt, rtcTime_t *t) {
    if (!rtcIsDateTimeValid(dt)) {
        return false;
    }
    unsigned int year = dt->year - REFERENCE_YEAR; // 0-99
    unsigned int month = dt->month - 1;            // 0-11
    unsigned int day = dt->day - 1;                // 0-30
    // 64-bit: the seconds exceed INT32_MAX from 2038-01-19 03:14:08 UTC.
    int64_t unixTime = (((((int64_t)(year / 4 * (365 * 4 + 1) + days[year % 4][month] + day)) * 24 + dt->hours) * 60 + dt->minutes) * 60) + dt->seconds + EPOCH_2000_OFFSET;
    *t = rtcTimeMake(unixTime, dt->millis);
    return true;
}

static bool rtcTimeIsInRange(rtcTime_t t) {
    return t >= RTC_TIME_MIN && t < RTC_TIME_END;
}

// Convert rtcTime_t to a calendar date. Returns false and leaves *dt unchanged when t is
// outside 2000-01-01 to 2099-12-31.
static bool rtcTimeToDateTime(dateTime_t *dt, rtcTime_t t) {
    if (!rtcTimeIsInRange(t)) {
        return false;
    }
    int64_t unixTime = t / MILLIS_PER_SECOND - EPOCH_2000_OFFSET; // 0 or more
    dt->seconds = unixTime % 60;
    unixTime /= 60;
    dt->minutes = unixTime % 60;
    unixTime /= 60;
    dt->hours = unixTime % 24;
    unixTime /= 24;
    unsigned int years = unixTime / (365 * 4 + 1) * 4;
    unixTime %= 365 * 4 + 1;
    unsigned int year;
    for (year = 3; year > 0; year--) {
        if (unixTime >= days[year][0]) {
            break;
        }
    }
    unsigned int month;
    for (month = 11; month > 0; month--) {
        if (unixTime >= days[year][month]) {
            break;
        }
    }
    dt->year = years + year + REFERENCE_YEAR;
    dt->month = month + 1;
    dt->day = unixTime - days[year][month] + 1;
    dt->millis = t % MILLIS_PER_SECOND;
    return true;
}

static void rtcGetDefaultDateTime(dateTime_t *dateTime) {
    dateTime->year = 0;
    dateTime->month = 1;
    dateTime->day = 1;
    dateTime->hours = 0;
    dateTime->minutes = 0;
    dateTime->seconds = 0;
    dateTime->millis = 0;
}

// On failure (input or result outside the supported range) dateTimeOffset is the default,
// invalid date (year 0) and the function returns false.
static bool dateTimeWithOffset(dateTime_t *dateTimeOffset, const dateTime_t *dateTimeInitial, int16_t minutes) {
    rtcTime_t initialTime;
    if (dateTimeToRtcTime(dateTimeInitial, &initialTime)) {
        rtcTime_t offsetTime = rtcTimeMake(rtcTimeGetSeconds(&initialTime) + minutes * 60, rtcTimeGetMillis(&initialTime));
        if (rtcTimeToDateTime(dateTimeOffset, offsetTime)) {
            return true;
        }
    }
    rtcGetDefaultDateTime(dateTimeOffset);
    return false;
}

static bool dateTimeFormat(char *buf, const dateTime_t *dateTime, int16_t offsetMinutes, bool shortVersion) {
    dateTime_t local;
    int tz_hours = 0;
    int tz_minutes = 0;
    bool retVal = true;
    // Apply offset if necessary
    if (offsetMinutes != 0) {
        tz_hours = offsetMinutes / 60;
        tz_minutes = ABS(offsetMinutes % 60);
        dateTimeWithOffset(&local, dateTime, offsetMinutes);
        dateTime = &local;
    }
    if (!rtcIsDateTimeValid(dateTime)) {
        rtcGetDefaultDateTime(&local);
        dateTime = &local;
        retVal = false;
    }
    if (shortVersion) {
        tfp_sprintf(buf, "%04u-%02u-%02u %02u:%02u:%02u",
                    dateTime->year, dateTime->month, dateTime->day,
                    dateTime->hours, dateTime->minutes, dateTime->seconds);
    } else {
        // Changes to this format might require updates in
        // dateTimeSplitFormatted()
        // Datetime is in ISO_8601 format, https://en.wikipedia.org/wiki/ISO_8601
        tfp_sprintf(buf, "%04u-%02u-%02uT%02u:%02u:%02u.%03u%c%02d:%02d",
                    dateTime->year, dateTime->month, dateTime->day,
                    dateTime->hours, dateTime->minutes, dateTime->seconds, dateTime->millis,
                    tz_hours >= 0 ? '+' : '-', ABS(tz_hours), tz_minutes);
    }
    return retVal;
}

rtcTime_t rtcTimeMake(int64_t secs, uint16_t millis) {
    return secs * MILLIS_PER_SECOND + millis;
}

int64_t rtcTimeGetSeconds(rtcTime_t *t) {
    return *t / MILLIS_PER_SECOND;
}

uint16_t rtcTimeGetMillis(rtcTime_t *t) {
    return *t % MILLIS_PER_SECOND;
}

bool dateTimeFormatUTC(char *buf, dateTime_t *dt) {
    return dateTimeFormat(buf, dt, 0, false);
}

bool dateTimeFormatLocal(char *buf, dateTime_t *dt) {
    return dateTimeFormat(buf, dt, timeConfig()->tz_offsetMinutes, false);
}

bool dateTimeFormatLocalShort(char *buf, dateTime_t *dt) {
    return dateTimeFormat(buf, dt, timeConfig()->tz_offsetMinutes, true);
}

void dateTimeUTCToLocal(dateTime_t *utcDateTime, dateTime_t *localDateTime) {
    dateTimeWithOffset(localDateTime, utcDateTime, timeConfig()->tz_offsetMinutes);
}

bool dateTimeSplitFormatted(char *formatted, char **date, char **time) {
    // Just look for the T and replace it with a zero
    // XXX: Keep in sync with dateTimeFormat()
    for (char *p = formatted; *p; p++) {
        if (*p == 'T') {
            *date = formatted;
            *time = (p + 1);
            *p = '\0';
            return true;
        }
    }
    return false;
}

bool rtcHasTime(void) {
    return started != 0;
}

#ifdef UNIT_TEST
void rtcClearForTest(void) {
    started = 0;
}
#endif

bool rtcGet(rtcTime_t *t) {
    if (!rtcHasTime()) {
        return false;
    }
    *t = started + millis();
    return true;
}

bool rtcSet(rtcTime_t *t) {
    if (!rtcTimeIsInRange(*t)) {
        return false;
    }
    started = *t - millis();
    return true;
}

bool rtcGetDateTime(dateTime_t *dt) {
    rtcTime_t t;
    if (rtcGet(&t) && rtcTimeToDateTime(dt, t)) {
        return true;
    }
    // No time stored or stored time out of range, fill dt with 0000-01-01T00:00:00.000
    rtcGetDefaultDateTime(dt);
    return false;
}

bool rtcSetDateTime(dateTime_t *dt) {
    rtcTime_t t;
    if (!dateTimeToRtcTime(dt, &t)) {
        return false;
    }
    return rtcSet(&t);
}

#endif
