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
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include <ctype.h>

#include "platform.h"

// FIXME remove this for targets that don't need a CLI.  Perhaps use a no-op macro when USE_CLI is not enabled
// signal that we're in cli mode
uint8_t cliMode = 0;
#ifndef EEPROM_IN_RAM
extern uint8_t __config_start;   // configured via linker script when building binaries.
extern uint8_t __config_end;
#endif

#ifdef USE_CLI

#include "blackbox/blackbox.h"

#include "build/build_config.h"
#include "build/debug.h"
#include "build/version.h"

#include "cms/cms.h"

#include "common/axis.h"
#include "common/color.h"
#include "common/maths.h"
#include "common/printf.h"
#include "common/strtol.h"
#include "common/time.h"
#include "common/typeconversion.h"
#include "common/utils.h"

#include "config/config_eeprom.h"
#include "config/feature.h"

#include "drivers/accgyro/accgyro.h"
#ifdef USE_GYRO_IMUF9001
#include "drivers/accgyro/accgyro_imuf9001.h"
#endif
#include "drivers/adc.h"
#include "drivers/buf_writer.h"
#include "drivers/bus_spi.h"
#include "drivers/camera_control.h"
#include "drivers/compass/compass.h"
#include "drivers/display.h"
#include "drivers/dma.h"
#include "drivers/dma_spi.h"
#include "drivers/flash.h"
#include "drivers/inverter.h"
#include "drivers/io.h"
#include "drivers/io_impl.h"
#include "drivers/light_led.h"
#include "drivers/rangefinder/rangefinder_hcsr04.h"
#include "drivers/sdcard.h"
#include "drivers/sensor.h"
#include "drivers/serial.h"
#include "drivers/serial_escserial.h"
#include "drivers/sound_beeper.h"
#include "drivers/stack_check.h"
#include "drivers/system.h"
#include "drivers/time.h"
#include "drivers/timer.h"
#include "drivers/transponder_ir.h"
#include "drivers/usb_msc.h"
#include "drivers/vtx_common.h"

#include "fc/board_info.h"
#include "fc/config.h"
#include "fc/controlrate_profile.h"
#include "fc/fc_core.h"
#include "fc/fc_rc.h"
#include "fc/rc_adjustments.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#include "flight/failsafe.h"
#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/position.h"
#include "flight/servos.h"

#include "interface/cli.h"
#include "interface/msp.h"
#include "interface/msp_box.h"
#include "interface/msp_protocol.h"
#include "interface/settings.h"
#ifdef MSP_OVER_CLI
#include "msp/msp_serial.h"
#endif

#include "io/asyncfatfs/asyncfatfs.h"
#include "io/beeper.h"
#include "io/flashfs.h"
#include "io/gimbal.h"
#include "io/gps.h"
#include "io/ledstrip.h"
#include "io/osd.h"
#include "io/serial.h"
#include "io/transponder_ir.h"
#include "io/usb_msc.h"
#include "io/vtx_control.h"
#include "io/vtx.h"

#include "pg/adc.h"
#include "pg/beeper.h"
#include "pg/beeper_dev.h"
#include "pg/board.h"
#include "pg/bus_i2c.h"
#include "pg/bus_spi.h"
#include "pg/max7456.h"
#include "pg/pinio.h"
#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/rx.h"
#include "pg/rx_spi.h"
#include "pg/rx_pwm.h"
#include "pg/timerio.h"
#include "pg/usb.h"

#include "rx/rx.h"
#include "rx/spektrum.h"
#include "rx/cc2500_frsky_common.h"
#include "rx/cc2500_frsky_x.h"
#include "rx/cc2500_common.h"

#include "scheduler/scheduler.h"

#include "sensors/acceleration.h"
#include "sensors/adcinternal.h"
#include "sensors/barometer.h"
#include "sensors/battery.h"
#include "sensors/boardalignment.h"
#include "sensors/compass.h"
#include "sensors/esc_sensor.h"
#include "sensors/gyro.h"
#include "sensors/sensors.h"

#include "telemetry/frsky_hub.h"
#include "telemetry/telemetry.h"


static serialPort_t *cliPort;

#ifdef STM32F1
#define CLI_IN_BUFFER_SIZE 128
#else
// Space required to set array parameters
#define CLI_IN_BUFFER_SIZE 256
#endif
#define CLI_OUT_BUFFER_SIZE 64

static bufWriter_t *cliWriter;
static uint8_t cliWriteBuffer[sizeof(*cliWriter) + CLI_OUT_BUFFER_SIZE];

static char cliBuffer[CLI_IN_BUFFER_SIZE];
static uint32_t bufferIndex = 0;

static bool configIsInCopy = false;

#define CURRENT_PROFILE_INDEX -1
static int8_t pidProfileIndexToUse = CURRENT_PROFILE_INDEX;
static int8_t rateProfileIndexToUse = CURRENT_PROFILE_INDEX;

#if defined(USE_BOARD_INFO)
static bool boardInformationUpdated = false;
#if defined(USE_SIGNATURE)
static bool signatureUpdated = false;
#endif
#endif // USE_BOARD_INFO

#ifdef USE_GYRO_IMUF9001
#define IMUF_CUSTOM_BUFF_LENGTH 26000
static   uint8_t  imuf_custom_buff[IMUF_CUSTOM_BUFF_LENGTH];
static   uint32_t imuf_buff_ptr = 0;
static   uint32_t imuf_checksum = 0;
static   int      imuf_bin_safe = 0;

#endif

static const char* const emptyName = "-";
static const char* const emptyString = "";

int cliSmartMode = 0;

#ifndef USE_QUAD_MIXER_ONLY
// sync this with mixerMode_e
static const char * const mixerNames[] = {
    "TRI", "QUADP", "QUADX", "BI",
    "GIMBAL", "Y6", "HEX6",
    "FLYING_WING", "Y4", "HEX6X", "OCTOX8", "OCTOFLATP", "OCTOFLATX",
    "AIRPLANE", "HELI_120_CCPM", "HELI_90_DEG", "VTAIL4",
    "HEX6H", "PPM_TO_SERVO", "DUALCOPTER", "SINGLECOPTER",
    "ATAIL4", "CUSTOM", "CUSTOMAIRPLANE", "CUSTOMTRI", "QUADX1234", NULL
};
#endif

// sync this with features_e
static const char * const featureNames[] = {
    "RX_PPM", "", "INFLIGHT_ACC_CAL", "RX_SERIAL", "MOTOR_STOP",
    "SERVO_TILT", "SOFTSERIAL", "GPS", "",
    "RANGEFINDER", "TELEMETRY", "", "3D", "RX_PARALLEL_PWM",
    "RX_MSP", "RSSI_ADC", "LED_STRIP", "DISPLAY", "OSD",
    "", "CHANNEL_FORWARDING", "TRANSPONDER", "AIRMODE",
    "", "", "RX_SPI", "SOFTSPI", "ESC_SENSOR", "ANTI_GRAVITY", "DYNAMIC_FILTER", "LEGACY_SA_SUPPORT", NULL
};

// sync this with rxFailsafeChannelMode_e
static const char rxFailsafeModeCharacters[] = "ahs";

static const rxFailsafeChannelMode_e rxFailsafeModesTable[RX_FAILSAFE_TYPE_COUNT][RX_FAILSAFE_MODE_COUNT] = {
    { RX_FAILSAFE_MODE_AUTO, RX_FAILSAFE_MODE_HOLD, RX_FAILSAFE_MODE_INVALID },
    { RX_FAILSAFE_MODE_INVALID, RX_FAILSAFE_MODE_HOLD, RX_FAILSAFE_MODE_SET }
};

#if defined(USE_SENSOR_NAMES)
// sync this with sensors_e
static const char * const sensorTypeNames[] = {
    "GYRO", "ACC", "BARO", "MAG", "RANGEFINDER", "GPS", "GPS+MAG", NULL
};

#define SENSOR_NAMES_MASK (SENSOR_GYRO | SENSOR_ACC | SENSOR_BARO | SENSOR_MAG | SENSOR_RANGEFINDER)

static const char * const *sensorHardwareNames[] = {
    lookupTableGyroHardware, lookupTableAccHardware, lookupTableBaroHardware, lookupTableMagHardware, lookupTableRangefinderHardware
};
#endif // USE_SENSOR_NAMES

static void backupPgConfig(const pgRegistry_t *pg)
{
    memcpy(pg->copy, pg->address, pg->size);
}

static void restorePgConfig(const pgRegistry_t *pg)
{
    memcpy(pg->address, pg->copy, pg->size);
}

static void backupConfigs(void)
{
    // make copies of configs to do differencing
    PG_FOREACH(pg) {
        backupPgConfig(pg);
    }

    configIsInCopy = true;
}

static void restoreConfigs(void)
{
    PG_FOREACH(pg) {
        restorePgConfig(pg);
    }

    configIsInCopy = false;
}

static void backupAndResetConfigs(void)
{
    backupConfigs();
    // reset all configs to defaults to do differencing
    resetConfigs();
}

static void cliPrint(const char *str)
{
    while (*str) {
        if(cliSmartMode)
        {
            //no carriage returns. Those are dumb.
            if(*str == '\r')
            {
                (void)(*str++);
            }
            else
            {
                bufWriterAppend(cliWriter, *str++);
            }
        }
        else
        {
            bufWriterAppend(cliWriter, *str++);
        }
    }
    bufWriterFlush(cliWriter);
}

static void cliPrintLinefeed(void)
{

    cliPrint("\r\n");

    if(cliSmartMode)
    {
        bufWriterFlush(cliWriter);
    }

}

static void cliPrintLine(const char *str)
{
    cliPrint(str);
    cliPrintLinefeed();
}

#ifdef MINIMAL_CLI
#define cliPrintHashLine(str)
#else
static void cliPrintHashLine(const char *str)
{
    cliPrint("\r\n# ");
    cliPrintLine(str);
}
#endif

static void cliPutp(void *p, char ch)
{
    bufWriterAppend(p, ch);
}

typedef enum {
    DUMP_MASTER = (1 << 0),
    DUMP_PROFILE = (1 << 1),
    DUMP_RATES = (1 << 2),
    DUMP_ALL = (1 << 3),
    DO_DIFF = (1 << 4),
    SHOW_DEFAULTS = (1 << 5),
    HIDE_UNUSED = (1 << 6)
} dumpFlags_e;

static void cliPrintfva(const char *format, va_list va)
{
    tfp_format(cliWriter, cliPutp, format, va);
    bufWriterFlush(cliWriter);
}

static bool cliDumpPrintLinef(uint8_t dumpMask, bool equalsDefault, const char *format, ...)
{
    if (!((dumpMask & DO_DIFF) && equalsDefault)) {
        va_list va;
        va_start(va, format);
        cliPrintfva(format, va);
        va_end(va);
        cliPrintLinefeed();
        return true;
    } else {
        return false;
    }
}

static void cliWrite(uint8_t ch)
{
    bufWriterAppend(cliWriter, ch);
}

static bool cliDefaultPrintLinef(uint8_t dumpMask, bool equalsDefault, const char *format, ...)
{
    if ((dumpMask & SHOW_DEFAULTS) && !equalsDefault) {
        cliWrite('#');

        va_list va;
        va_start(va, format);
        cliPrintfva(format, va);
        va_end(va);
        cliPrintLinefeed();
        return true;
    } else {
        return false;
    }
}

static void cliPrintf(const char *format, ...)
{
    va_list va;
    va_start(va, format);
    cliPrintfva(format, va);
    va_end(va);
}


static void cliPrintLinef(const char *format, ...)
{
    va_list va;
    va_start(va, format);
    cliPrintfva(format, va);
    va_end(va);
    cliPrintLinefeed();
}

static void cliPrintErrorLinef(const char *format, ...)
{
    cliPrint("###ERROR### ");
    va_list va;
    va_start(va, format);
    cliPrintfva(format, va);
    va_end(va);
    cliPrintLinefeed();
}


static void printValuePointer(const clivalue_t *var, const void *valuePointer, bool full)
{
    if ((var->type & VALUE_MODE_MASK) == MODE_ARRAY) {
        for (int i = 0; i < var->config.array.length; i++) {
            switch (var->type & VALUE_TYPE_MASK) {
            default:
            case VAR_UINT8:
                // uint8_t array
                cliPrintf("%d", ((uint8_t *)valuePointer)[i]);
                break;

            case VAR_INT8:
                // int8_t array
                cliPrintf("%d", ((int8_t *)valuePointer)[i]);
                break;

            case VAR_UINT16:
                // uin16_t array
                cliPrintf("%d", ((uint16_t *)valuePointer)[i]);
                break;

            case VAR_INT16:
                // int16_t array
                cliPrintf("%d", ((int16_t *)valuePointer)[i]);
                break;
            }

            if (i < var->config.array.length - 1) {
                cliPrint(",");
            }
        }
    } else {
        int value = 0;

        switch (var->type & VALUE_TYPE_MASK) {
        case VAR_UINT8:
            value = *(uint8_t *)valuePointer;
            break;

        case VAR_INT8:
            value = *(int8_t *)valuePointer;
            break;

        case VAR_UINT16:
        case VAR_INT16:
            value = *(int16_t *)valuePointer;
            break;
        case VAR_UINT32:
            value = *(uint32_t *)valuePointer;
            break;
        }

        switch (var->type & VALUE_MODE_MASK) {
        case MODE_DIRECT:
            cliPrintf("%d", value);
            if (full) {
                cliPrintf(" %d %d", var->config.minmax.min, var->config.minmax.max);
            }
            break;
        case MODE_LOOKUP:
            cliPrint(lookupTables[var->config.lookup.tableIndex].values[value]);
            break;
        case MODE_BITSET:
            if (value & 1 << var->config.bitpos) {
                cliPrintf("ON");
            } else {
                cliPrintf("OFF");
            }
        }
    }
}


static bool valuePtrEqualsDefault(const clivalue_t *var, const void *ptr, const void *ptrDefault)
{
    bool result = true;
    int elementCount = 1;
    uint32_t mask = 0xffffffff;

    if ((var->type & VALUE_MODE_MASK) == MODE_ARRAY) {
        elementCount = var->config.array.length;
    }
    if ((var->type & VALUE_MODE_MASK) == MODE_BITSET) {
        mask = 1 << var->config.bitpos;
    }
    for (int i = 0; i < elementCount; i++) {
        switch (var->type & VALUE_TYPE_MASK) {
        case VAR_UINT8:
            result = result && (((uint8_t *)ptr)[i] & mask) == (((uint8_t *)ptrDefault)[i] & mask);
            break;

        case VAR_INT8:
            result = result && ((int8_t *)ptr)[i] == ((int8_t *)ptrDefault)[i];
            break;

        case VAR_UINT16:
            result = result && (((int16_t *)ptr)[i] & mask) == (((int16_t *)ptrDefault)[i] & mask);
            break;
        case VAR_INT16:
            result = result && ((int16_t *)ptr)[i] == ((int16_t *)ptrDefault)[i];
            break;
        case VAR_UINT32:
            result = result && (((uint32_t *)ptr)[i] & mask) == (((uint32_t *)ptrDefault)[i] & mask);
            break;
        }
    }
    for (int i = 0; i < elementCount; i++) {
        switch (var->type & VALUE_TYPE_MASK) {
        case VAR_UINT8:
            result = result && ((uint8_t *)ptr)[i] == ((uint8_t *)ptrDefault)[i];
            break;

        case VAR_INT8:
            result = result && ((int8_t *)ptr)[i] == ((int8_t *)ptrDefault)[i];
            break;

        case VAR_UINT16:
        case VAR_INT16:
            result = result && ((int16_t *)ptr)[i] == ((int16_t *)ptrDefault)[i];
            break;
        }
    }
    return (result);
}

static uint8_t getPidProfileIndexToUse()
{
    return pidProfileIndexToUse == CURRENT_PROFILE_INDEX ? getCurrentPidProfileIndex() : pidProfileIndexToUse;
}

static uint8_t getRateProfileIndexToUse()
{
    return rateProfileIndexToUse == CURRENT_PROFILE_INDEX ? getCurrentControlRateProfileIndex() : rateProfileIndexToUse;
}


static uint16_t getValueOffset(const clivalue_t *value)
{
    switch (value->type & VALUE_SECTION_MASK) {
    case MASTER_VALUE:
        return value->offset;
    case PROFILE_VALUE:
        return value->offset + sizeof(pidProfile_t) * getPidProfileIndexToUse();
    case PROFILE_RATE_VALUE:
        return value->offset + sizeof(controlRateConfig_t) * getRateProfileIndexToUse();
    }
    return 0;
}

void *cliGetValuePointer(const clivalue_t *value)
{
    const pgRegistry_t* rec = pgFind(value->pgn);
    if (configIsInCopy) {
        return CONST_CAST(void *, rec->copy + getValueOffset(value));
    } else {
        return CONST_CAST(void *, rec->address + getValueOffset(value));
    }
}

const void *cliGetDefaultPointer(const clivalue_t *value)
{
    const pgRegistry_t* rec = pgFind(value->pgn);
    return rec->address + getValueOffset(value);
}

static void dumpPgValue(const clivalue_t *value, uint8_t dumpMask)
{
    const pgRegistry_t *pg = pgFind(value->pgn);
#ifdef DEBUG
    if (!pg) {
        cliPrintLinef("VALUE %s ERROR", value->name);
        return; // if it's not found, the pgn shouldn't be in the value table!
    }
#endif

    const char *format = "set %s = ";
    const char *defaultFormat = "#set %s = ";
    const int valueOffset = getValueOffset(value);
    const bool equalsDefault = valuePtrEqualsDefault(value, pg->copy + valueOffset, pg->address + valueOffset);

    if (((dumpMask & DO_DIFF) == 0) || !equalsDefault) {
        if (dumpMask & SHOW_DEFAULTS && !equalsDefault) {
            cliPrintf(defaultFormat, value->name);
            printValuePointer(value, (uint8_t*)pg->address + valueOffset, false);
            cliPrintLinefeed();
        }
        cliPrintf(format, value->name);
        printValuePointer(value, pg->copy + valueOffset, false);
        cliPrintLinefeed();
    }
}

static void dumpAllValues(uint16_t valueSection, uint8_t dumpMask)
{
    for (uint32_t i = 0; i < valueTableEntryCount; i++) {
        const clivalue_t *value = &valueTable[i];
        bufWriterFlush(cliWriter);
        if ((value->type & VALUE_SECTION_MASK) == valueSection) {
            dumpPgValue(value, dumpMask);
        }
    }
}

static void cliPrintVar(const clivalue_t *var, bool full)
{
    const void *ptr = cliGetValuePointer(var);

    printValuePointer(var, ptr, full);
}

static void cliPrintVarRange(const clivalue_t *var)
{
    switch (var->type & VALUE_MODE_MASK) {
    case (MODE_DIRECT): {
        cliPrintLinef("Allowed range: %d - %d", var->config.minmax.min, var->config.minmax.max);
    }
    break;
    case (MODE_LOOKUP): {
        const lookupTableEntry_t *tableEntry = &lookupTables[var->config.lookup.tableIndex];
        cliPrint("Allowed values: ");
        bool firstEntry = true;
        for (unsigned i = 0; i < tableEntry->valueCount; i++) {
            if (tableEntry->values[i]) {
                if (!firstEntry) {
                    cliPrint(", ");
                }
                cliPrintf("%s", tableEntry->values[i]);
                firstEntry = false;
            }
        }
        cliPrintLinefeed();
    }
    break;
    case (MODE_ARRAY): {
        cliPrintLinef("Array length: %d", var->config.array.length);
    }
    break;
    case (MODE_BITSET): {
        cliPrintLinef("Allowed values: OFF, ON");
    }
    break;
    }
}

static void cliSetVar(const clivalue_t *var, const int16_t value)
{
    void *ptr = cliGetValuePointer(var);
    uint32_t workValue;
    uint32_t mask;

    if ((var->type & VALUE_MODE_MASK) == MODE_BITSET) {
        switch (var->type & VALUE_TYPE_MASK) {
        case VAR_UINT8:
            mask = (1 << var->config.bitpos) & 0xff;
            if (value) {
                workValue = *(uint8_t *)ptr | mask;
            } else {
                workValue = *(uint8_t *)ptr & ~mask;
            }
            *(uint8_t *)ptr = workValue;
            break;

        case VAR_UINT16:
            mask = (1 << var->config.bitpos) & 0xffff;
            if (value) {
                workValue = *(uint16_t *)ptr | mask;
            } else {
                workValue = *(uint16_t *)ptr & ~mask;
            }
            *(uint16_t *)ptr = workValue;
            break;

        case VAR_UINT32:
            mask = 1 << var->config.bitpos;
            if (value) {
                workValue = *(uint32_t *)ptr | mask;
            } else {
                workValue = *(uint32_t *)ptr & ~mask;
            }
            *(uint32_t *)ptr = workValue;
            break;

        }
    } else {
        switch (var->type & VALUE_TYPE_MASK) {
        case VAR_UINT8:
            *(uint8_t *)ptr = value;
            break;

        case VAR_INT8:
            *(int8_t *)ptr = value;
            break;

        case VAR_UINT16:
        case VAR_INT16:
            *(int16_t *)ptr = value;
            break;
        }
    }
}

#if defined(USE_RESOURCE_MGMT) && !defined(MINIMAL_CLI)
static void cliRepeat(char ch, uint8_t len)
{
    for (int i = 0; i < len; i++) {
        bufWriterAppend(cliWriter, ch);
    }
    cliPrintLinefeed();
}
#endif

static void cliPrompt(void)
{
    cliPrint("\r\n# ");
}

static void cliShowParseError(void)
{
    cliPrintErrorLinef("Parse error");
}

static void cliShowArgumentRangeError(char *name, int min, int max)
{
    cliPrintErrorLinef("%s not between %d and %d", name, min, max);
}

static const char *nextArg(const char *currentArg)
{
    const char *ptr = strchr(currentArg, ' ');
    while (ptr && *ptr == ' ') {
        ptr++;
    }

    return ptr;
}

static const char *processChannelRangeArgs(const char *ptr, channelRange_t *range, uint8_t *validArgumentCount)
{
    for (uint32_t argIndex = 0; argIndex < 2; argIndex++) {
        ptr = nextArg(ptr);
        if (ptr) {
            int val = atoi(ptr);
            val = CHANNEL_VALUE_TO_STEP(val);
            if (val >= MIN_MODE_RANGE_STEP && val <= MAX_MODE_RANGE_STEP) {
                if (argIndex == 0) {
                    range->startStep = val;
                } else {
                    range->endStep = val;
                }
                (*validArgumentCount)++;
            }
        }
    }

    return ptr;
}

// Check if a string's length is zero
static bool isEmpty(const char *string)
{
    return (string == NULL || *string == '\0') ? true : false;
}

static void printRxFailsafe(uint8_t dumpMask, const rxFailsafeChannelConfig_t *rxFailsafeChannelConfigs, const rxFailsafeChannelConfig_t *defaultRxFailsafeChannelConfigs)
{
    // print out rxConfig failsafe settings
    for (uint32_t channel = 0; channel < MAX_SUPPORTED_RC_CHANNEL_COUNT; channel++) {
        const rxFailsafeChannelConfig_t *channelFailsafeConfig = &rxFailsafeChannelConfigs[channel];
        const rxFailsafeChannelConfig_t *defaultChannelFailsafeConfig = &defaultRxFailsafeChannelConfigs[channel];
        const bool equalsDefault = !memcmp(channelFailsafeConfig, defaultChannelFailsafeConfig, sizeof(*channelFailsafeConfig));
        const bool requireValue = channelFailsafeConfig->mode == RX_FAILSAFE_MODE_SET;
        if (requireValue) {
            const char *format = "rxfail %u %c %d";
            cliDefaultPrintLinef(dumpMask, equalsDefault, format,
                channel,
                rxFailsafeModeCharacters[defaultChannelFailsafeConfig->mode],
                RXFAIL_STEP_TO_CHANNEL_VALUE(defaultChannelFailsafeConfig->step)
            );
            cliDumpPrintLinef(dumpMask, equalsDefault, format,
                channel,
                rxFailsafeModeCharacters[channelFailsafeConfig->mode],
                RXFAIL_STEP_TO_CHANNEL_VALUE(channelFailsafeConfig->step)
            );
        } else {
            const char *format = "rxfail %u %c";
            cliDefaultPrintLinef(dumpMask, equalsDefault, format,
                channel,
                rxFailsafeModeCharacters[defaultChannelFailsafeConfig->mode]
            );
            cliDumpPrintLinef(dumpMask, equalsDefault, format,
                channel,
                rxFailsafeModeCharacters[channelFailsafeConfig->mode]
            );
        }
    }
}

static void cliRxFailsafe(char *cmdline)
{
    uint8_t channel;
    char buf[3];

    if (isEmpty(cmdline)) {
        // print out rxConfig failsafe settings
        for (channel = 0; channel < MAX_SUPPORTED_RC_CHANNEL_COUNT; channel++) {
            cliRxFailsafe(itoa(channel, buf, 10));
        }
    } else {
        const char *ptr = cmdline;
        channel = atoi(ptr++);
        if ((channel < MAX_SUPPORTED_RC_CHANNEL_COUNT)) {

            rxFailsafeChannelConfig_t *channelFailsafeConfig = rxFailsafeChannelConfigsMutable(channel);

            const rxFailsafeChannelType_e type = (channel < NON_AUX_CHANNEL_COUNT) ? RX_FAILSAFE_TYPE_FLIGHT : RX_FAILSAFE_TYPE_AUX;
            rxFailsafeChannelMode_e mode = channelFailsafeConfig->mode;
            bool requireValue = channelFailsafeConfig->mode == RX_FAILSAFE_MODE_SET;

            ptr = nextArg(ptr);
            if (ptr) {
                const char *p = strchr(rxFailsafeModeCharacters, *(ptr));
                if (p) {
                    const uint8_t requestedMode = p - rxFailsafeModeCharacters;
                    mode = rxFailsafeModesTable[type][requestedMode];
                } else {
                    mode = RX_FAILSAFE_MODE_INVALID;
                }
                if (mode == RX_FAILSAFE_MODE_INVALID) {
                    cliShowParseError();
                    return;
                }

                requireValue = mode == RX_FAILSAFE_MODE_SET;

                ptr = nextArg(ptr);
                if (ptr) {
                    if (!requireValue) {
                        cliShowParseError();
                        return;
                    }
                    uint16_t value = atoi(ptr);
                    value = CHANNEL_VALUE_TO_RXFAIL_STEP(value);
                    if (value > MAX_RXFAIL_RANGE_STEP) {
                        cliPrintLine("Value out of range");
                        return;
                    }

                    channelFailsafeConfig->step = value;
                } else if (requireValue) {
                    cliShowParseError();
                    return;
                }
                channelFailsafeConfig->mode = mode;
            }

            char modeCharacter = rxFailsafeModeCharacters[channelFailsafeConfig->mode];

            // double use of cliPrintf below
            // 1. acknowledge interpretation on command,
            // 2. query current setting on single item,

            if (requireValue) {
                cliPrintLinef("rxfail %u %c %d",
                    channel,
                    modeCharacter,
                    RXFAIL_STEP_TO_CHANNEL_VALUE(channelFailsafeConfig->step)
                );
            } else {
                cliPrintLinef("rxfail %u %c",
                    channel,
                    modeCharacter
                );
            }
        } else {
            cliShowArgumentRangeError("channel", 0, MAX_SUPPORTED_RC_CHANNEL_COUNT - 1);
        }
    }
}

static void printAux(uint8_t dumpMask, const modeActivationCondition_t *modeActivationConditions, const modeActivationCondition_t *defaultModeActivationConditions)
{
    const char *format = "aux %u %u %u %u %u %u %u";
    // print out aux channel settings
    for (uint32_t i = 0; i < MAX_MODE_ACTIVATION_CONDITION_COUNT; i++) {
        const modeActivationCondition_t *mac = &modeActivationConditions[i];
        bool equalsDefault = false;
        if (defaultModeActivationConditions) {
            const modeActivationCondition_t *macDefault = &defaultModeActivationConditions[i];
            equalsDefault = !memcmp(mac, macDefault, sizeof(*mac));
            const box_t *box = findBoxByBoxId(macDefault->modeId);
            const box_t *linkedTo = findBoxByBoxId(macDefault->linkedTo);
            if (box) {
                cliDefaultPrintLinef(dumpMask, equalsDefault, format,
                    i,
                    box->permanentId,
                    macDefault->auxChannelIndex,
                    MODE_STEP_TO_CHANNEL_VALUE(macDefault->range.startStep),
                    MODE_STEP_TO_CHANNEL_VALUE(macDefault->range.endStep),
                    macDefault->modeLogic,
                    linkedTo ? linkedTo->permanentId : 0
                );
            }
        }
        const box_t *box = findBoxByBoxId(mac->modeId);
        const box_t *linkedTo = findBoxByBoxId(mac->linkedTo);
        if (box) {
            cliDumpPrintLinef(dumpMask, equalsDefault, format,
                i,
                box->permanentId,
                mac->auxChannelIndex,
                MODE_STEP_TO_CHANNEL_VALUE(mac->range.startStep),
                MODE_STEP_TO_CHANNEL_VALUE(mac->range.endStep),
                mac->modeLogic,
                linkedTo ? linkedTo->permanentId : 0
            );
        }
    }
}

static void cliAux(char *cmdline)
{
    int i, val = 0;
    const char *ptr;

    if (isEmpty(cmdline)) {
        printAux(DUMP_MASTER, modeActivationConditions(0), NULL);
    } else {
        ptr = cmdline;
        i = atoi(ptr++);
        if (i < MAX_MODE_ACTIVATION_CONDITION_COUNT) {
            modeActivationCondition_t *mac = modeActivationConditionsMutable(i);
            uint8_t validArgumentCount = 0;
            ptr = nextArg(ptr);
            if (ptr) {
                val = atoi(ptr);
                const box_t *box = findBoxByPermanentId(val);
                if (box) {
                    mac->modeId = box->boxId;
                    validArgumentCount++;
                }
            }
            ptr = nextArg(ptr);
            if (ptr) {
                val = atoi(ptr);
                if (val >= 0 && val < MAX_AUX_CHANNEL_COUNT) {
                    mac->auxChannelIndex = val;
                    validArgumentCount++;
                }
            }
            ptr = processChannelRangeArgs(ptr, &mac->range, &validArgumentCount);
            ptr = nextArg(ptr);
            if (ptr) {
                val = atoi(ptr);
                if (val == MODELOGIC_OR || val == MODELOGIC_AND) {
                    mac->modeLogic = val;
                    validArgumentCount++;
                }
            }
            ptr = nextArg(ptr);
            if (ptr) {
                val = atoi(ptr);
                const box_t *box = findBoxByPermanentId(val);
                if (box) {
                    mac->linkedTo = box->boxId;
                    validArgumentCount++;
                }
            }
            if (validArgumentCount == 4) { // for backwards compatibility
                mac->modeLogic = MODELOGIC_OR;
            } else if (validArgumentCount == 5) { // for backwards compatibility
                mac->linkedTo = 0;
            } else if (validArgumentCount != 6) {
                memset(mac, 0, sizeof(modeActivationCondition_t));
            }
            cliPrintLinef( "aux %u %u %u %u %u %u %u",
                i,
                mac->modeId,
                mac->auxChannelIndex,
                MODE_STEP_TO_CHANNEL_VALUE(mac->range.startStep),
                MODE_STEP_TO_CHANNEL_VALUE(mac->range.endStep),
                mac->modeLogic,
                mac->linkedTo
            );
        } else {
            cliShowArgumentRangeError("index", 0, MAX_MODE_ACTIVATION_CONDITION_COUNT - 1);
        }
    }
}

static void printSerial(uint8_t dumpMask, const serialConfig_t *serialConfig, const serialConfig_t *serialConfigDefault)
{
    const char *format = "serial %d %d %ld %ld %ld %ld";
    for (uint32_t i = 0; i < SERIAL_PORT_COUNT; i++) {
        if (!serialIsPortAvailable(serialConfig->portConfigs[i].identifier)) {
            continue;
        };
        bool equalsDefault = false;
        if (serialConfigDefault) {
            equalsDefault = !memcmp(&serialConfig->portConfigs[i], &serialConfigDefault->portConfigs[i], sizeof(serialConfig->portConfigs[i]));
            cliDefaultPrintLinef(dumpMask, equalsDefault, format,
                serialConfigDefault->portConfigs[i].identifier,
                serialConfigDefault->portConfigs[i].functionMask,
                baudRates[serialConfigDefault->portConfigs[i].msp_baudrateIndex],
                baudRates[serialConfigDefault->portConfigs[i].gps_baudrateIndex],
                baudRates[serialConfigDefault->portConfigs[i].telemetry_baudrateIndex],
                baudRates[serialConfigDefault->portConfigs[i].blackbox_baudrateIndex]
            );
        }
        cliDumpPrintLinef(dumpMask, equalsDefault, format,
            serialConfig->portConfigs[i].identifier,
            serialConfig->portConfigs[i].functionMask,
            baudRates[serialConfig->portConfigs[i].msp_baudrateIndex],
            baudRates[serialConfig->portConfigs[i].gps_baudrateIndex],
            baudRates[serialConfig->portConfigs[i].telemetry_baudrateIndex],
            baudRates[serialConfig->portConfigs[i].blackbox_baudrateIndex]
            );
    }
}

static void cliSerial(char *cmdline)
{
    const char *format = "serial %d %d %ld %ld %ld %ld";
    if (isEmpty(cmdline)) {
        printSerial(DUMP_MASTER, serialConfig(), NULL);
        return;
    }
    serialPortConfig_t portConfig;
    memset(&portConfig, 0 , sizeof(portConfig));

    serialPortConfig_t *currentConfig;

    uint8_t validArgumentCount = 0;

    const char *ptr = cmdline;

    int val = atoi(ptr++);
    currentConfig = serialFindPortConfiguration(val);
    if (currentConfig) {
        portConfig.identifier = val;
        validArgumentCount++;
    }

    ptr = nextArg(ptr);
    if (ptr) {
        val = atoi(ptr);
        portConfig.functionMask = val & 0xFFFF;
        validArgumentCount++;
    }

    for (int i = 0; i < 4; i ++) {
        ptr = nextArg(ptr);
        if (!ptr) {
            break;
        }

        val = atoi(ptr);

        uint8_t baudRateIndex = lookupBaudRateIndex(val);
        if (baudRates[baudRateIndex] != (uint32_t) val) {
            break;
        }

        switch (i) {
        case 0:
            if (baudRateIndex < BAUD_9600 || baudRateIndex > BAUD_1000000) {
                continue;
            }
            portConfig.msp_baudrateIndex = baudRateIndex;
            break;
        case 1:
            if (baudRateIndex < BAUD_9600 || baudRateIndex > BAUD_115200) {
                continue;
            }
            portConfig.gps_baudrateIndex = baudRateIndex;
            break;
        case 2:
            if (baudRateIndex != BAUD_AUTO && baudRateIndex > BAUD_115200) {
                continue;
            }
            portConfig.telemetry_baudrateIndex = baudRateIndex;
            break;
        case 3:
            if (baudRateIndex < BAUD_19200 || baudRateIndex > BAUD_2470000) {
                continue;
            }
            portConfig.blackbox_baudrateIndex = baudRateIndex;
            break;
        }

        validArgumentCount++;
    }

    if (validArgumentCount < 6) {
        cliShowParseError();
        return;
    }

    memcpy(currentConfig, &portConfig, sizeof(portConfig));

    cliDumpPrintLinef(0, false, format,
        portConfig.identifier,
        portConfig.functionMask,
        baudRates[portConfig.msp_baudrateIndex],
        baudRates[portConfig.gps_baudrateIndex],
        baudRates[portConfig.telemetry_baudrateIndex],
        baudRates[portConfig.blackbox_baudrateIndex]
        );

}

#ifndef SKIP_SERIAL_PASSTHROUGH
#ifdef USE_PINIO
static void cbCtrlLine(void *context, uint16_t ctrl)
{
    int pinioDtr = (int)(long)context;

    pinioSet(pinioDtr, !(ctrl & CTRL_LINE_STATE_DTR));
}
#endif /* USE_PINIO */

static void cliSerialPassthrough(char *cmdline)
{
    if (isEmpty(cmdline)) {
        cliShowParseError();
        return;
    }

    int id = -1;
    uint32_t baud = 0;
    bool enableBaudCb = false;
#ifdef USE_PINIO
    int pinioDtr = 0;
#endif /* USE_PINIO */
    unsigned mode = 0;
    char *saveptr;
    char* tok = strtok_r(cmdline, " ", &saveptr);
    int index = 0;

    while (tok != NULL) {
        switch (index) {
        case 0:
            id = atoi(tok);
            break;
        case 1:
            baud = atoi(tok);
            break;
        case 2:
            if (strstr(tok, "rx") || strstr(tok, "RX"))
                mode |= MODE_RX;
            if (strstr(tok, "tx") || strstr(tok, "TX"))
                mode |= MODE_TX;
            break;
#ifdef USE_PINIO
        case 3:
            pinioDtr = atoi(tok);
            break;
#endif /* USE_PINIO */
        }
        index++;
        tok = strtok_r(NULL, " ", &saveptr);
    }

    if (baud == 0) {
        enableBaudCb = true;
    }

    cliPrintf("Port %d ", id);
    serialPort_t *passThroughPort;
    serialPortUsage_t *passThroughPortUsage = findSerialPortUsageByIdentifier(id);
    if (!passThroughPortUsage || passThroughPortUsage->serialPort == NULL) {
        if (enableBaudCb) {
            // Set default baud
            baud = 57600;
        }

        if (!mode) {
            mode = MODE_RXTX;
        }

        passThroughPort = openSerialPort(id, FUNCTION_NONE, NULL, NULL,
                                         baud, mode,
                                         SERIAL_NOT_INVERTED);
        if (!passThroughPort) {
            cliPrintLine("could not be opened.");
            return;
        }

        if (enableBaudCb) {
            cliPrintf("opened, default baud = %d.\r\n", baud);
        } else {
            cliPrintf("opened, baud = %d.\r\n", baud);
        }
    } else {
        passThroughPort = passThroughPortUsage->serialPort;
        // If the user supplied a mode, override the port's mode, otherwise
        // leave the mode unchanged. serialPassthrough() handles one-way ports.
        // Set the baud rate if specified
        if (baud) {
            cliPrintf("already open, setting baud = %d.\n\r", baud);
            serialSetBaudRate(passThroughPort, baud);
        } else {
            cliPrintf("already open, baud = %d.\n\r", passThroughPort->baudRate);
        }

        if (mode && passThroughPort->mode != mode) {
            cliPrintf("Mode changed from %d to %d.\r\n",
                   passThroughPort->mode, mode);
            serialSetMode(passThroughPort, mode);
        }

        // If this port has a rx callback associated we need to remove it now.
        // Otherwise no data will be pushed in the serial port buffer!
        if (passThroughPort->rxCallback) {
            passThroughPort->rxCallback = 0;
        }
    }

    // If no baud rate is specified allow to be set via USB
    if (enableBaudCb) {
        cliPrintLine("Baud rate change over USB enabled.");
        // Register the right side baud rate setting routine with the left side which allows setting of the UART
        // baud rate over USB without setting it using the serialpassthrough command
        serialSetBaudRateCb(cliPort, serialSetBaudRate, passThroughPort);
    }

    cliPrintLine("Forwarding, power cycle to exit.");

#ifdef USE_PINIO
    // Register control line state callback
    if (pinioDtr) {
        serialSetCtrlLineStateCb(cliPort, cbCtrlLine, (void *)(intptr_t)(pinioDtr - 1));
    }
#endif /* USE_PINIO */

    serialPassthrough(cliPort, passThroughPort, NULL, NULL);
}
#endif

static void printAdjustmentRange(uint8_t dumpMask, const adjustmentRange_t *adjustmentRanges, const adjustmentRange_t *defaultAdjustmentRanges)
{
    const char *format = "adjrange %u %u %u %u %u %u %u %u %u";
    // print out adjustment ranges channel settings
    for (uint32_t i = 0; i < MAX_ADJUSTMENT_RANGE_COUNT; i++) {
        const adjustmentRange_t *ar = &adjustmentRanges[i];
        bool equalsDefault = false;
        if (defaultAdjustmentRanges) {
            const adjustmentRange_t *arDefault = &defaultAdjustmentRanges[i];
            equalsDefault = !memcmp(ar, arDefault, sizeof(*ar));
            cliDefaultPrintLinef(dumpMask, equalsDefault, format,
                i,
                arDefault->adjustmentIndex,
                arDefault->auxChannelIndex,
                MODE_STEP_TO_CHANNEL_VALUE(arDefault->range.startStep),
                MODE_STEP_TO_CHANNEL_VALUE(arDefault->range.endStep),
                arDefault->adjustmentFunction,
                arDefault->auxSwitchChannelIndex,
                arDefault->adjustmentCenter,
                arDefault->adjustmentScale
            );
        }
        cliDumpPrintLinef(dumpMask, equalsDefault, format,
            i,
            ar->adjustmentIndex,
            ar->auxChannelIndex,
            MODE_STEP_TO_CHANNEL_VALUE(ar->range.startStep),
            MODE_STEP_TO_CHANNEL_VALUE(ar->range.endStep),
            ar->adjustmentFunction,
            ar->auxSwitchChannelIndex,
            ar->adjustmentCenter,
            ar->adjustmentScale
        );
    }
}

static void cliAdjustmentRange(char *cmdline)
{
    const char *format = "adjrange %u %u %u %u %u %u %u %u %u";
    int i, val = 0;
    const char *ptr;

    if (isEmpty(cmdline)) {
        printAdjustmentRange(DUMP_MASTER, adjustmentRanges(0), NULL);
    } else {
        ptr = cmdline;
        i = atoi(ptr++);
        if (i < MAX_ADJUSTMENT_RANGE_COUNT) {
            adjustmentRange_t *ar = adjustmentRangesMutable(i);
            uint8_t validArgumentCount = 0;

            ptr = nextArg(ptr);
            if (ptr) {
                val = atoi(ptr);
                if (val >= 0 && val < MAX_SIMULTANEOUS_ADJUSTMENT_COUNT) {
                    ar->adjustmentIndex = val;
                    validArgumentCount++;
                }
            }
            ptr = nextArg(ptr);
            if (ptr) {
                val = atoi(ptr);
                if (val >= 0 && val < MAX_AUX_CHANNEL_COUNT) {
                    ar->auxChannelIndex = val;
                    validArgumentCount++;
                }
            }

            ptr = processChannelRangeArgs(ptr, &ar->range, &validArgumentCount);

            ptr = nextArg(ptr);
            if (ptr) {
                val = atoi(ptr);
                if (val >= 0 && val < ADJUSTMENT_FUNCTION_COUNT) {
                    ar->adjustmentFunction = val;
                    validArgumentCount++;
                }
            }
            ptr = nextArg(ptr);
            if (ptr) {
                val = atoi(ptr);
                if (val >= 0 && val < MAX_AUX_CHANNEL_COUNT) {
                    ar->auxSwitchChannelIndex = val;
                    validArgumentCount++;
                }
            }

            if (validArgumentCount != 6) {
                memset(ar, 0, sizeof(adjustmentRange_t));
                cliShowParseError();
                return;
            }

            // Optional arguments
            ar->adjustmentCenter = 0;
            ar->adjustmentScale = 0;

            ptr = nextArg(ptr);
            if (ptr) {
                val = atoi(ptr);
                ar->adjustmentCenter = val;
                validArgumentCount++;
            }
            ptr = nextArg(ptr);
            if (ptr) {
                val = atoi(ptr);
                ar->adjustmentScale = val;
                validArgumentCount++;
            }
            cliDumpPrintLinef(0, false, format,
                i,
                ar->adjustmentIndex,
                ar->auxChannelIndex,
                MODE_STEP_TO_CHANNEL_VALUE(ar->range.startStep),
                MODE_STEP_TO_CHANNEL_VALUE(ar->range.endStep),
                ar->adjustmentFunction,
                ar->auxSwitchChannelIndex,
                ar->adjustmentCenter,
                ar->adjustmentScale
            );

        } else {
            cliShowArgumentRangeError("index", 0, MAX_ADJUSTMENT_RANGE_COUNT - 1);
        }
    }
}

#ifndef USE_QUAD_MIXER_ONLY
static void printMotorMix(uint8_t dumpMask, const motorMixer_t *customMotorMixer, const motorMixer_t *defaultCustomMotorMixer)
{
    const char *format = "mmix %d %s %s %s %s";
    char buf0[FTOA_BUFFER_LENGTH];
    char buf1[FTOA_BUFFER_LENGTH];
    char buf2[FTOA_BUFFER_LENGTH];
    char buf3[FTOA_BUFFER_LENGTH];
    for (uint32_t i = 0; i < MAX_SUPPORTED_MOTORS; i++) {
        if (customMotorMixer[i].throttle == 0.0f)
            break;
        const float thr = customMotorMixer[i].throttle;
        const float roll = customMotorMixer[i].roll;
        const float pitch = customMotorMixer[i].pitch;
        const float yaw = customMotorMixer[i].yaw;
        bool equalsDefault = false;
        if (defaultCustomMotorMixer) {
            const float thrDefault = defaultCustomMotorMixer[i].throttle;
            const float rollDefault = defaultCustomMotorMixer[i].roll;
            const float pitchDefault = defaultCustomMotorMixer[i].pitch;
            const float yawDefault = defaultCustomMotorMixer[i].yaw;
            const bool equalsDefault = thr == thrDefault && roll == rollDefault && pitch == pitchDefault && yaw == yawDefault;

            cliDefaultPrintLinef(dumpMask, equalsDefault, format,
                i,
                ftoa(thrDefault, buf0),
                ftoa(rollDefault, buf1),
                ftoa(pitchDefault, buf2),
                ftoa(yawDefault, buf3));
        }
        cliDumpPrintLinef(dumpMask, equalsDefault, format,
            i,
            ftoa(thr, buf0),
            ftoa(roll, buf1),
            ftoa(pitch, buf2),
            ftoa(yaw, buf3));
    }
}
#endif // USE_QUAD_MIXER_ONLY

static void cliMotorMix(char *cmdline)
{
#ifdef USE_QUAD_MIXER_ONLY
    UNUSED(cmdline);
#else
    int check = 0;
    uint8_t len;
    const char *ptr;

    if (isEmpty(cmdline)) {
        printMotorMix(DUMP_MASTER, customMotorMixer(0), NULL);
    } else if (strncasecmp(cmdline, "reset", 5) == 0) {
        // erase custom mixer
        for (uint32_t i = 0; i < MAX_SUPPORTED_MOTORS; i++) {
            customMotorMixerMutable(i)->throttle = 0.0f;
        }
    } else if (strncasecmp(cmdline, "load", 4) == 0) {
        ptr = nextArg(cmdline);
        if (ptr) {
            len = strlen(ptr);
            for (uint32_t i = 0; ; i++) {
                if (mixerNames[i] == NULL) {
                    cliPrintErrorLinef("Invalid name");
                    break;
                }
                if (strncasecmp(ptr, mixerNames[i], len) == 0) {
                    mixerLoadMix(i, customMotorMixerMutable(0));
                    cliPrintLinef("Loaded %s", mixerNames[i]);
                    cliMotorMix("");
                    break;
                }
            }
        }
    } else {
        ptr = cmdline;
        uint32_t i = atoi(ptr); // get motor number
        if (i < MAX_SUPPORTED_MOTORS) {
            ptr = nextArg(ptr);
            if (ptr) {
                customMotorMixerMutable(i)->throttle = fastA2F(ptr);
                check++;
            }
            ptr = nextArg(ptr);
            if (ptr) {
                customMotorMixerMutable(i)->roll = fastA2F(ptr);
                check++;
            }
            ptr = nextArg(ptr);
            if (ptr) {
                customMotorMixerMutable(i)->pitch = fastA2F(ptr);
                check++;
            }
            ptr = nextArg(ptr);
            if (ptr) {
                customMotorMixerMutable(i)->yaw = fastA2F(ptr);
                check++;
            }
            if (check != 4) {
                cliShowParseError();
            } else {
                printMotorMix(DUMP_MASTER, customMotorMixer(0), NULL);
            }
        } else {
            cliShowArgumentRangeError("index", 0, MAX_SUPPORTED_MOTORS - 1);
        }
    }
#endif
}

static void printRxRange(uint8_t dumpMask, const rxChannelRangeConfig_t *channelRangeConfigs, const rxChannelRangeConfig_t *defaultChannelRangeConfigs)
{
    const char *format = "rxrange %u %u %u";
    for (uint32_t i = 0; i < NON_AUX_CHANNEL_COUNT; i++) {
        bool equalsDefault = false;
        if (defaultChannelRangeConfigs) {
            equalsDefault = !memcmp(&channelRangeConfigs[i], &defaultChannelRangeConfigs[i], sizeof(channelRangeConfigs[i]));
            cliDefaultPrintLinef(dumpMask, equalsDefault, format,
                i,
                defaultChannelRangeConfigs[i].min,
                defaultChannelRangeConfigs[i].max
            );
        }
        cliDumpPrintLinef(dumpMask, equalsDefault, format,
            i,
            channelRangeConfigs[i].min,
            channelRangeConfigs[i].max
        );
    }
}

static void cliRxRange(char *cmdline)
{
    const char *format = "rxrange %u %u %u";
    int i, validArgumentCount = 0;
    const char *ptr;

    if (isEmpty(cmdline)) {
        printRxRange(DUMP_MASTER, rxChannelRangeConfigs(0), NULL);
    } else if (strcasecmp(cmdline, "reset") == 0) {
        resetAllRxChannelRangeConfigurations(rxChannelRangeConfigsMutable(0));
    } else {
        ptr = cmdline;
        i = atoi(ptr);
        if (i >= 0 && i < NON_AUX_CHANNEL_COUNT) {
            int rangeMin = PWM_PULSE_MIN, rangeMax = PWM_PULSE_MAX;

            ptr = nextArg(ptr);
            if (ptr) {
                rangeMin = atoi(ptr);
                validArgumentCount++;
            }

            ptr = nextArg(ptr);
            if (ptr) {
                rangeMax = atoi(ptr);
                validArgumentCount++;
            }

            if (validArgumentCount != 2) {
                cliShowParseError();
            } else if (rangeMin < PWM_PULSE_MIN || rangeMin > PWM_PULSE_MAX || rangeMax < PWM_PULSE_MIN || rangeMax > PWM_PULSE_MAX) {
                cliShowParseError();
            } else {
                rxChannelRangeConfig_t *channelRangeConfig = rxChannelRangeConfigsMutable(i);
                channelRangeConfig->min = rangeMin;
                channelRangeConfig->max = rangeMax;
                cliDumpPrintLinef(0, false, format,
                    i,
                    channelRangeConfig->min,
                    channelRangeConfig->max
                );

            }
        } else {
            cliShowArgumentRangeError("channel", 0, NON_AUX_CHANNEL_COUNT - 1);
        }
    }
}

#ifdef USE_LED_STRIP
static void printLed(uint8_t dumpMask, const ledConfig_t *ledConfigs, const ledConfig_t *defaultLedConfigs)
{
    const char *format = "led %u %s";
    char ledConfigBuffer[20];
    char ledConfigDefaultBuffer[20];
    for (uint32_t i = 0; i < LED_MAX_STRIP_LENGTH; i++) {
        ledConfig_t ledConfig = ledConfigs[i];
        generateLedConfig(&ledConfig, ledConfigBuffer, sizeof(ledConfigBuffer));
        bool equalsDefault = false;
        if (defaultLedConfigs) {
            ledConfig_t ledConfigDefault = defaultLedConfigs[i];
            equalsDefault = ledConfig == ledConfigDefault;
            generateLedConfig(&ledConfigDefault, ledConfigDefaultBuffer, sizeof(ledConfigDefaultBuffer));
            cliDefaultPrintLinef(dumpMask, equalsDefault, format, i, ledConfigDefaultBuffer);
        }
        cliDumpPrintLinef(dumpMask, equalsDefault, format, i, ledConfigBuffer);
    }
}

static void cliLed(char *cmdline)
{
    const char *format = "led %u %s";
    char ledConfigBuffer[20];
    int i;
    const char *ptr;

    if (isEmpty(cmdline)) {
        printLed(DUMP_MASTER, ledStripConfig()->ledConfigs, NULL);
    } else {
        ptr = cmdline;
        i = atoi(ptr);
        if (i < LED_MAX_STRIP_LENGTH) {
            ptr = nextArg(cmdline);
            if (parseLedStripConfig(i, ptr)) {
                generateLedConfig((ledConfig_t *)&ledStripConfig()->ledConfigs[i], ledConfigBuffer, sizeof(ledConfigBuffer));
                cliDumpPrintLinef(0, false, format, i, ledConfigBuffer);
            } else {
                cliShowParseError();
            }
        } else {
            cliShowArgumentRangeError("index", 0, LED_MAX_STRIP_LENGTH - 1);
        }
    }
}

static void printColor(uint8_t dumpMask, const hsvColor_t *colors, const hsvColor_t *defaultColors)
{
    const char *format = "color %u %d,%u,%u";
    for (uint32_t i = 0; i < LED_CONFIGURABLE_COLOR_COUNT; i++) {
        const hsvColor_t *color = &colors[i];
        bool equalsDefault = false;
        if (defaultColors) {
            const hsvColor_t *colorDefault = &defaultColors[i];
            equalsDefault = !memcmp(color, colorDefault, sizeof(*color));
            cliDefaultPrintLinef(dumpMask, equalsDefault, format, i,colorDefault->h, colorDefault->s, colorDefault->v);
        }
        cliDumpPrintLinef(dumpMask, equalsDefault, format, i, color->h, color->s, color->v);
    }
}

static void cliColor(char *cmdline)
{
    const char *format = "color %u %d,%u,%u";
    if (isEmpty(cmdline)) {
        printColor(DUMP_MASTER, ledStripConfig()->colors, NULL);
    } else {
        const char *ptr = cmdline;
        const int i = atoi(ptr);
        if (i < LED_CONFIGURABLE_COLOR_COUNT) {
            ptr = nextArg(cmdline);
            if (parseColor(i, ptr)) {
                const hsvColor_t *color = &ledStripConfig()->colors[i];
                cliDumpPrintLinef(0, false, format, i, color->h, color->s, color->v);
            } else {
                cliShowParseError();
            }
        } else {
            cliShowArgumentRangeError("index", 0, LED_CONFIGURABLE_COLOR_COUNT - 1);
        }
    }
}

static void printModeColor(uint8_t dumpMask, const ledStripConfig_t *ledStripConfig, const ledStripConfig_t *defaultLedStripConfig)
{
    const char *format = "mode_color %u %u %u";
    for (uint32_t i = 0; i < LED_MODE_COUNT; i++) {
        for (uint32_t j = 0; j < LED_DIRECTION_COUNT; j++) {
            int colorIndex = ledStripConfig->modeColors[i].color[j];
            bool equalsDefault = false;
            if (defaultLedStripConfig) {
                int colorIndexDefault = defaultLedStripConfig->modeColors[i].color[j];
                equalsDefault = colorIndex == colorIndexDefault;
                cliDefaultPrintLinef(dumpMask, equalsDefault, format, i, j, colorIndexDefault);
            }
            cliDumpPrintLinef(dumpMask, equalsDefault, format, i, j, colorIndex);
        }
    }

    for (uint32_t j = 0; j < LED_SPECIAL_COLOR_COUNT; j++) {
        const int colorIndex = ledStripConfig->specialColors.color[j];
        bool equalsDefault = false;
        if (defaultLedStripConfig) {
            const int colorIndexDefault = defaultLedStripConfig->specialColors.color[j];
            equalsDefault = colorIndex == colorIndexDefault;
            cliDefaultPrintLinef(dumpMask, equalsDefault, format, LED_SPECIAL, j, colorIndexDefault);
        }
        cliDumpPrintLinef(dumpMask, equalsDefault, format, LED_SPECIAL, j, colorIndex);
    }

    const int ledStripAuxChannel = ledStripConfig->ledstrip_aux_channel;
    bool equalsDefault = false;
    if (defaultLedStripConfig) {
        const int ledStripAuxChannelDefault = defaultLedStripConfig->ledstrip_aux_channel;
        equalsDefault = ledStripAuxChannel == ledStripAuxChannelDefault;
        cliDefaultPrintLinef(dumpMask, equalsDefault, format, LED_AUX_CHANNEL, 0, ledStripAuxChannelDefault);
    }
    cliDumpPrintLinef(dumpMask, equalsDefault, format, LED_AUX_CHANNEL, 0, ledStripAuxChannel);
}

static void cliModeColor(char *cmdline)
{
    if (isEmpty(cmdline)) {
        printModeColor(DUMP_MASTER, ledStripConfig(), NULL);
    } else {
        enum {MODE = 0, FUNCTION, COLOR, ARGS_COUNT};
        int args[ARGS_COUNT];
        int argNo = 0;
        char *saveptr;
        const char* ptr = strtok_r(cmdline, " ", &saveptr);
        while (ptr && argNo < ARGS_COUNT) {
            args[argNo++] = atoi(ptr);
            ptr = strtok_r(NULL, " ", &saveptr);
        }

        if (ptr != NULL || argNo != ARGS_COUNT) {
            cliShowParseError();
            return;
        }

        int modeIdx  = args[MODE];
        int funIdx = args[FUNCTION];
        int color = args[COLOR];
        if (!setModeColor(modeIdx, funIdx, color)) {
            cliShowParseError();
            return;
        }
        // values are validated
        cliPrintLinef("mode_color %u %u %u", modeIdx, funIdx, color);
    }
}
#endif

#ifdef USE_SERVOS
static void printServo(uint8_t dumpMask, const servoParam_t *servoParams, const servoParam_t *defaultServoParams)
{
    // print out servo settings
    const char *format = "servo %u %d %d %d %d %d";
    for (uint32_t i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
        const servoParam_t *servoConf = &servoParams[i];
        bool equalsDefault = false;
        if (defaultServoParams) {
            const servoParam_t *defaultServoConf = &defaultServoParams[i];
            equalsDefault = !memcmp(servoConf, defaultServoConf, sizeof(*servoConf));
            cliDefaultPrintLinef(dumpMask, equalsDefault, format,
                i,
                defaultServoConf->min,
                defaultServoConf->max,
                defaultServoConf->middle,
                defaultServoConf->rate,
                defaultServoConf->forwardFromChannel
            );
        }
        cliDumpPrintLinef(dumpMask, equalsDefault, format,
            i,
            servoConf->min,
            servoConf->max,
            servoConf->middle,
            servoConf->rate,
            servoConf->forwardFromChannel
        );
    }
    // print servo directions
    for (uint32_t i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
        const char *format = "smix reverse %d %d r";
        const servoParam_t *servoConf = &servoParams[i];
        const servoParam_t *servoConfDefault = &defaultServoParams[i];
        if (defaultServoParams) {
            bool equalsDefault = servoConf->reversedSources == servoConfDefault->reversedSources;
            for (uint32_t channel = 0; channel < INPUT_SOURCE_COUNT; channel++) {
                equalsDefault = ~(servoConf->reversedSources ^ servoConfDefault->reversedSources) & (1 << channel);
                if (servoConfDefault->reversedSources & (1 << channel)) {
                    cliDefaultPrintLinef(dumpMask, equalsDefault, format, i , channel);
                }
                if (servoConf->reversedSources & (1 << channel)) {
                    cliDumpPrintLinef(dumpMask, equalsDefault, format, i , channel);
                }
            }
        } else {
            for (uint32_t channel = 0; channel < INPUT_SOURCE_COUNT; channel++) {
                if (servoConf->reversedSources & (1 << channel)) {
                    cliDumpPrintLinef(dumpMask, true, format, i , channel);
                }
            }
        }
    }
}

static void cliServo(char *cmdline)
{
    const char *format = "servo %u %d %d %d %d %d";
    enum { SERVO_ARGUMENT_COUNT = 6 };
    int16_t arguments[SERVO_ARGUMENT_COUNT];

    servoParam_t *servo;

    int i;
    char *ptr;

    if (isEmpty(cmdline)) {
        printServo(DUMP_MASTER, servoParams(0), NULL);
    } else {
        int validArgumentCount = 0;

        ptr = cmdline;

        // Command line is integers (possibly negative) separated by spaces, no other characters allowed.

        // If command line doesn't fit the format, don't modify the config
        while (*ptr) {
            if (*ptr == '-' || (*ptr >= '0' && *ptr <= '9')) {
                if (validArgumentCount >= SERVO_ARGUMENT_COUNT) {
                    cliShowParseError();
                    return;
                }

                arguments[validArgumentCount++] = atoi(ptr);

                do {
                    ptr++;
                } while (*ptr >= '0' && *ptr <= '9');
            } else if (*ptr == ' ') {
                ptr++;
            } else {
                cliShowParseError();
                return;
            }
        }

        enum {INDEX = 0, MIN, MAX, MIDDLE, RATE, FORWARD};

        i = arguments[INDEX];

        // Check we got the right number of args and the servo index is correct (don't validate the other values)
        if (validArgumentCount != SERVO_ARGUMENT_COUNT || i < 0 || i >= MAX_SUPPORTED_SERVOS) {
            cliShowParseError();
            return;
        }

        servo = servoParamsMutable(i);

        if (
            arguments[MIN] < PWM_PULSE_MIN || arguments[MIN] > PWM_PULSE_MAX ||
            arguments[MAX] < PWM_PULSE_MIN || arguments[MAX] > PWM_PULSE_MAX ||
            arguments[MIDDLE] < arguments[MIN] || arguments[MIDDLE] > arguments[MAX] ||
            arguments[MIN] > arguments[MAX] || arguments[MAX] < arguments[MIN] ||
            arguments[RATE] < -100 || arguments[RATE] > 100 ||
            arguments[FORWARD] >= MAX_SUPPORTED_RC_CHANNEL_COUNT
        ) {
            cliShowParseError();
            return;
        }

        servo->min = arguments[MIN];
        servo->max = arguments[MAX];
        servo->middle = arguments[MIDDLE];
        servo->rate = arguments[RATE];
        servo->forwardFromChannel = arguments[FORWARD];

        cliDumpPrintLinef(0, false, format,
            i,
            servo->min,
            servo->max,
            servo->middle,
            servo->rate,
            servo->forwardFromChannel
        );

    }
}
#endif

#ifdef USE_SERVOS
static void printServoMix(uint8_t dumpMask, const servoMixer_t *customServoMixers, const servoMixer_t *defaultCustomServoMixers)
{
    const char *format = "smix %d %d %d %d %d %d %d %d";
    for (uint32_t i = 0; i < MAX_SERVO_RULES; i++) {
        const servoMixer_t customServoMixer = customServoMixers[i];
        if (customServoMixer.rate == 0) {
            break;
        }

        bool equalsDefault = false;
        if (defaultCustomServoMixers) {
            servoMixer_t customServoMixerDefault = defaultCustomServoMixers[i];
            equalsDefault = !memcmp(&customServoMixer, &customServoMixerDefault, sizeof(customServoMixer));

            cliDefaultPrintLinef(dumpMask, equalsDefault, format,
                i,
                customServoMixerDefault.targetChannel,
                customServoMixerDefault.inputSource,
                customServoMixerDefault.rate,
                customServoMixerDefault.speed,
                customServoMixerDefault.min,
                customServoMixerDefault.max,
                customServoMixerDefault.box
            );
        }
        cliDumpPrintLinef(dumpMask, equalsDefault, format,
            i,
            customServoMixer.targetChannel,
            customServoMixer.inputSource,
            customServoMixer.rate,
            customServoMixer.speed,
            customServoMixer.min,
            customServoMixer.max,
            customServoMixer.box
        );
    }

    cliPrintLinefeed();
}

static void cliServoMix(char *cmdline)
{
    int args[8], check = 0;
    int len = strlen(cmdline);

    if (len == 0) {
        printServoMix(DUMP_MASTER, customServoMixers(0), NULL);
    } else if (strncasecmp(cmdline, "reset", 5) == 0) {
        // erase custom mixer
        memset(customServoMixers_array(), 0, sizeof(*customServoMixers_array()));
        for (uint32_t i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
            servoParamsMutable(i)->reversedSources = 0;
        }
    } else if (strncasecmp(cmdline, "load", 4) == 0) {
        const char *ptr = nextArg(cmdline);
        if (ptr) {
            len = strlen(ptr);
            for (uint32_t i = 0; ; i++) {
                if (mixerNames[i] == NULL) {
                    cliPrintErrorLinef("Invalid name");
                    break;
                }
                if (strncasecmp(ptr, mixerNames[i], len) == 0) {
                    servoMixerLoadMix(i);
                    cliPrintLinef("Loaded %s", mixerNames[i]);
                    cliServoMix("");
                    break;
                }
            }
        }
    } else if (strncasecmp(cmdline, "reverse", 7) == 0) {
        enum {SERVO = 0, INPUT, REVERSE, ARGS_COUNT};
        char *ptr = strchr(cmdline, ' ');

        if (ptr == NULL) {
            cliPrintf("s");
            for (uint32_t inputSource = 0; inputSource < INPUT_SOURCE_COUNT; inputSource++)
                cliPrintf("\ti%d", inputSource);
            cliPrintLinefeed();

            for (uint32_t servoIndex = 0; servoIndex < MAX_SUPPORTED_SERVOS; servoIndex++) {
                cliPrintf("%d", servoIndex);
                for (uint32_t inputSource = 0; inputSource < INPUT_SOURCE_COUNT; inputSource++) {
                    cliPrintf("\t%s  ", (servoParams(servoIndex)->reversedSources & (1 << inputSource)) ? "r" : "n");
                }
                cliPrintLinefeed();
            }
            return;
        }

        char *saveptr;
        ptr = strtok_r(ptr, " ", &saveptr);
        while (ptr != NULL && check < ARGS_COUNT - 1) {
            args[check++] = atoi(ptr);
            ptr = strtok_r(NULL, " ", &saveptr);
        }

        if (ptr == NULL || check != ARGS_COUNT - 1) {
            cliShowParseError();
            return;
        }

        if (args[SERVO] >= 0 && args[SERVO] < MAX_SUPPORTED_SERVOS
                && args[INPUT] >= 0 && args[INPUT] < INPUT_SOURCE_COUNT
                && (*ptr == 'r' || *ptr == 'n')) {
            if (*ptr == 'r') {
                servoParamsMutable(args[SERVO])->reversedSources |= 1 << args[INPUT];
            } else {
                servoParamsMutable(args[SERVO])->reversedSources &= ~(1 << args[INPUT]);
            }
        } else {
            cliShowParseError();
            return;
        }

        cliServoMix("reverse");
    } else {
        enum {RULE = 0, TARGET, INPUT, RATE, SPEED, MIN, MAX, BOX, ARGS_COUNT};
        char *saveptr;
        char *ptr = strtok_r(cmdline, " ", &saveptr);
        while (ptr != NULL && check < ARGS_COUNT) {
            args[check++] = atoi(ptr);
            ptr = strtok_r(NULL, " ", &saveptr);
        }

        if (ptr != NULL || check != ARGS_COUNT) {
            cliShowParseError();
            return;
        }

        int32_t i = args[RULE];
        if (i >= 0 && i < MAX_SERVO_RULES &&
            args[TARGET] >= 0 && args[TARGET] < MAX_SUPPORTED_SERVOS &&
            args[INPUT] >= 0 && args[INPUT] < INPUT_SOURCE_COUNT &&
            args[RATE] >= -100 && args[RATE] <= 100 &&
            args[SPEED] >= 0 && args[SPEED] <= MAX_SERVO_SPEED &&
            args[MIN] >= 0 && args[MIN] <= 100 &&
            args[MAX] >= 0 && args[MAX] <= 100 && args[MIN] < args[MAX] &&
            args[BOX] >= 0 && args[BOX] <= MAX_SERVO_BOXES) {
            customServoMixersMutable(i)->targetChannel = args[TARGET];
            customServoMixersMutable(i)->inputSource = args[INPUT];
            customServoMixersMutable(i)->rate = args[RATE];
            customServoMixersMutable(i)->speed = args[SPEED];
            customServoMixersMutable(i)->min = args[MIN];
            customServoMixersMutable(i)->max = args[MAX];
            customServoMixersMutable(i)->box = args[BOX];
            cliServoMix("");
        } else {
            cliShowParseError();
        }
    }
}
#endif

#ifdef USE_SDCARD

static void cliWriteBytes(const uint8_t *buffer, int count)
{
    while (count > 0) {
        cliWrite(*buffer);
        buffer++;
        count--;
    }
}

static void cliSdInfo(char *cmdline)
{
    UNUSED(cmdline);

    cliPrint("SD card: ");

    if (!sdcard_isInserted()) {
        cliPrintLine("None inserted");
        return;
    }

    if (!sdcard_isFunctional() || !sdcard_isInitialized()) {
        cliPrintLine("Startup failed");
        return;
    }

    const sdcardMetadata_t *metadata = sdcard_getMetadata();

    cliPrintf("Manufacturer 0x%x, %ukB, %02d/%04d, v%d.%d, '",
        metadata->manufacturerID,
        metadata->numBlocks / 2, /* One block is half a kB */
        metadata->productionMonth,
        metadata->productionYear,
        metadata->productRevisionMajor,
        metadata->productRevisionMinor
    );

    cliWriteBytes((uint8_t*)metadata->productName, sizeof(metadata->productName));

    cliPrint("'\r\n" "Filesystem: ");

    switch (afatfs_getFilesystemState()) {
    case AFATFS_FILESYSTEM_STATE_READY:
        cliPrint("Ready");
        break;
    case AFATFS_FILESYSTEM_STATE_INITIALIZATION:
        cliPrint("Initializing");
        break;
    case AFATFS_FILESYSTEM_STATE_UNKNOWN:
    case AFATFS_FILESYSTEM_STATE_FATAL:
        cliPrint("Fatal");

        switch (afatfs_getLastError()) {
        case AFATFS_ERROR_BAD_MBR:
            cliPrint(" - no FAT MBR partitions");
            break;
        case AFATFS_ERROR_BAD_FILESYSTEM_HEADER:
            cliPrint(" - bad FAT header");
            break;
        case AFATFS_ERROR_GENERIC:
        case AFATFS_ERROR_NONE:
            ; // Nothing more detailed to print
            break;
        }
        break;
    }
    cliPrintLinefeed();
}

#endif

#ifdef USE_FLASHFS

static void cliFlashInfo(char *cmdline)
{
    const flashGeometry_t *layout = flashfsGetGeometry();

    UNUSED(cmdline);

    cliPrintLinef("Flash sectors=%u, sectorSize=%u, pagesPerSector=%u, pageSize=%u, totalSize=%u, usedSize=%u",
            layout->sectors, layout->sectorSize, layout->pagesPerSector, layout->pageSize, layout->totalSize, flashfsGetOffset());
}


static void cliFlashErase(char *cmdline)
{
    UNUSED(cmdline);

    if (!flashfsIsSupported()) {
        return;
    }

#ifndef MINIMAL_CLI
    uint32_t i = 0;
    cliPrintLine("Erasing, please wait ... ");
#else
    cliPrintLine("Erasing,");
#endif

    bufWriterFlush(cliWriter);
    flashfsEraseCompletely();

    while (!flashfsIsReady()) {
#ifndef MINIMAL_CLI
        cliPrintf(".");
        if (i++ > 120) {
            i=0;
            cliPrintLinefeed();
        }

        bufWriterFlush(cliWriter);
#endif
        delay(100);
    }
    beeper(BEEPER_BLACKBOX_ERASE);
    cliPrintLinefeed();
    cliPrintLine("Done.");
}

#ifdef USE_FLASH_TOOLS

static void cliFlashWrite(char *cmdline)
{
    const uint32_t address = atoi(cmdline);
    const char *text = strchr(cmdline, ' ');

    if (!text) {
        cliShowParseError();
    } else {
        flashfsSeekAbs(address);
        flashfsWrite((uint8_t*)text, strlen(text), true);
        flashfsFlushSync();

        cliPrintLinef("Wrote %u bytes at %u.", strlen(text), address);
    }
}

static void cliFlashRead(char *cmdline)
{
    uint32_t address = atoi(cmdline);

    const char *nextArg = strchr(cmdline, ' ');

    if (!nextArg) {
        cliShowParseError();
    } else {
        uint32_t length = atoi(nextArg);

        cliPrintLinef("Reading %u bytes at %u:", length, address);

        uint8_t buffer[32];
        while (length > 0) {
            int bytesRead = flashfsReadAbs(address, buffer, length < sizeof(buffer) ? length : sizeof(buffer));

            for (int i = 0; i < bytesRead; i++) {
                cliWrite(buffer[i]);
            }

            length -= bytesRead;
            address += bytesRead;

            if (bytesRead == 0) {
                //Assume we reached the end of the volume or something fatal happened
                break;
            }
        }
        cliPrintLinefeed();
    }
}

#endif
#endif

#ifdef USE_VTX_CONTROL
static void printVtx(uint8_t dumpMask, const vtxConfig_t *vtxConfig, const vtxConfig_t *vtxConfigDefault)
{
    // print out vtx channel settings
    const char *format = "vtx %u %u %u %u %u %u";
    bool equalsDefault = false;
    for (uint32_t i = 0; i < MAX_CHANNEL_ACTIVATION_CONDITION_COUNT; i++) {
        const vtxChannelActivationCondition_t *cac = &vtxConfig->vtxChannelActivationConditions[i];
        if (vtxConfigDefault) {
            const vtxChannelActivationCondition_t *cacDefault = &vtxConfigDefault->vtxChannelActivationConditions[i];
            equalsDefault = !memcmp(cac, cacDefault, sizeof(*cac));
            cliDefaultPrintLinef(dumpMask, equalsDefault, format,
                i,
                cacDefault->auxChannelIndex,
                cacDefault->band,
                cacDefault->channel,
                MODE_STEP_TO_CHANNEL_VALUE(cacDefault->range.startStep),
                MODE_STEP_TO_CHANNEL_VALUE(cacDefault->range.endStep)
            );
        }
        cliDumpPrintLinef(dumpMask, equalsDefault, format,
            i,
            cac->auxChannelIndex,
            cac->band,
            cac->channel,
            MODE_STEP_TO_CHANNEL_VALUE(cac->range.startStep),
            MODE_STEP_TO_CHANNEL_VALUE(cac->range.endStep)
        );
    }
}

static void cliVtx(char *cmdline)
{
    const char *format = "vtx %u %u %u %u %u %u";
    int i, val = 0;
    const char *ptr;

    if (isEmpty(cmdline)) {
        printVtx(DUMP_MASTER, vtxConfig(), NULL);
    } else {
        ptr = cmdline;
        i = atoi(ptr++);
        if (i < MAX_CHANNEL_ACTIVATION_CONDITION_COUNT) {
            vtxChannelActivationCondition_t *cac = &vtxConfigMutable()->vtxChannelActivationConditions[i];
            uint8_t validArgumentCount = 0;
            ptr = nextArg(ptr);
            if (ptr) {
                val = atoi(ptr);
                if (val >= 0 && val < MAX_AUX_CHANNEL_COUNT) {
                    cac->auxChannelIndex = val;
                    validArgumentCount++;
                }
            }
            ptr = nextArg(ptr);
            if (ptr) {
                val = atoi(ptr);
                // FIXME Use VTX API to get max
                // We check for the min value in final validation below
                if (val >= 0 && val <= VTX_SETTINGS_MAX_BAND) {
                    cac->band = val;
                    validArgumentCount++;
                }
            }
            ptr = nextArg(ptr);
            if (ptr) {
                val = atoi(ptr);
                // FIXME Use VTX API to get max
                // We check for the min value in final validation below
                if (val >= 0 && val <= VTX_SETTINGS_MAX_CHANNEL) {
                    cac->channel = val;
                    validArgumentCount++;
                }
            }
            ptr = processChannelRangeArgs(ptr, &cac->range, &validArgumentCount);

            bool parseError = false;
            if (validArgumentCount != 5) {
                parseError = true;
            } else {
                // check for an empty activation condition for reset
                vtxChannelActivationCondition_t emptyCac;
                memset(&emptyCac, 0, sizeof(emptyCac));
                if (memcmp(cac, &emptyCac, sizeof(emptyCac)) != 0
                    // FIXME Use VTX API to get min
                    && ((cac->band < VTX_SETTINGS_MIN_BAND) || (cac->channel < VTX_SETTINGS_MIN_CHANNEL))) {
                    parseError = true;
                }
            }

            if (parseError) {
                memset(cac, 0, sizeof(vtxChannelActivationCondition_t));
                cliShowParseError();
            } else {
                cliDumpPrintLinef(0, false, format,
                    i,
                    cac->auxChannelIndex,
                    cac->band,
                    cac->channel,
                    MODE_STEP_TO_CHANNEL_VALUE(cac->range.startStep),
                    MODE_STEP_TO_CHANNEL_VALUE(cac->range.endStep)
                );
            }
        } else {
            cliShowArgumentRangeError("index", 0, MAX_CHANNEL_ACTIVATION_CONDITION_COUNT - 1);
        }
    }
}

#endif // VTX_CONTROL

static void printName(uint8_t dumpMask, const pilotConfig_t *pilotConfig)
{
    const bool equalsDefault = strlen(pilotConfig->name) == 0;
    cliDumpPrintLinef(dumpMask, equalsDefault, "name %s", equalsDefault ? emptyName : pilotConfig->name);
}

static void cliName(char *cmdline)
{
    const unsigned int len = strlen(cmdline);
    if (len > 0) {
        memset(pilotConfigMutable()->name, 0, ARRAYLEN(pilotConfig()->name));
        if (strncmp(cmdline, emptyName, len)) {
            strncpy(pilotConfigMutable()->name, cmdline, MIN(len, MAX_NAME_LENGTH));
        }
    }
    printName(DUMP_MASTER, pilotConfig());
}

#if defined(USE_BOARD_INFO)

#define ERROR_MESSAGE "%s cannot be changed. Current value: '%s'"

static void cliBoardName(char *cmdline)
{
    const unsigned int len = strlen(cmdline);
    if (len > 0 && boardInformationIsSet() && (len != strlen(getBoardName()) || strncmp(getBoardName(), cmdline, len))) {
        cliPrintErrorLinef(ERROR_MESSAGE, "board_name", getBoardName());
    } else {
        if (len > 0) {
            setBoardName(cmdline);
            boardInformationUpdated = true;
        }
        cliPrintLinef("board_name %s", getBoardName());
    }
}

static void cliManufacturerId(char *cmdline)
{
    const unsigned int len = strlen(cmdline);
    if (len > 0 && boardInformationIsSet() && (len != strlen(getManufacturerId()) || strncmp(getManufacturerId(), cmdline, len))) {
        cliPrintErrorLinef(ERROR_MESSAGE, "manufacturer_id", getManufacturerId());
    } else {
        if (len > 0) {
            setManufacturerId(cmdline);
            boardInformationUpdated = true;
        }
        cliPrintLinef("manufacturer_id %s", getManufacturerId());
    }
}

#if defined(USE_SIGNATURE)
static void writeSignature(char *signatureStr, uint8_t *signature)
{
    for (unsigned i = 0; i < SIGNATURE_LENGTH; i++) {
        tfp_sprintf(&signatureStr[2 * i], "%02x", signature[i]);
    }
}

static void cliSignature(char *cmdline)
{
    const int len = strlen(cmdline);

    uint8_t signature[SIGNATURE_LENGTH] = {0};
    if (len > 0) {
        if (len != 2 * SIGNATURE_LENGTH) {
            cliPrintErrorLinef("Invalid length: %d (expected: %d)", len, 2 * SIGNATURE_LENGTH);

            return;
        }

#define BLOCK_SIZE 2
        for (unsigned i = 0; i < SIGNATURE_LENGTH; i++) {
            char temp[BLOCK_SIZE + 1];
            strncpy(temp, &cmdline[i * BLOCK_SIZE], BLOCK_SIZE);
            temp[BLOCK_SIZE] = '\0';
            char *end;
            unsigned result = strtoul(temp, &end, 16);
            if (end == &temp[BLOCK_SIZE]) {
                signature[i] = result;
            } else {
                cliPrintErrorLinef("Invalid character found: %c", end[0]);

                return;
            }
        }
#undef BLOCK_SIZE
    }

    char signatureStr[SIGNATURE_LENGTH * 2 + 1] = {0};
    if (len > 0 && signatureIsSet() && memcmp(signature, getSignature(), SIGNATURE_LENGTH)) {
        writeSignature(signatureStr, getSignature());
        cliPrintErrorLinef(ERROR_MESSAGE, "signature", signatureStr);
    } else {
        if (len > 0) {
            setSignature(signature);

            signatureUpdated = true;

            writeSignature(signatureStr, getSignature());
        } else if (signatureUpdated || signatureIsSet()) {
            writeSignature(signatureStr, getSignature());
        }

        cliPrintLinef("signature %s", signatureStr);
    }
}
#endif

#undef ERROR_MESSAGE

#endif // USE_BOARD_INFO

static void cliMcuId(char *cmdline)
{
    UNUSED(cmdline);

    cliPrintLinef("mcu_id %08x%08x%08x", U_ID_0, U_ID_1, U_ID_2);
}

static void printFeature(uint8_t dumpMask, const featureConfig_t *featureConfig, const featureConfig_t *featureConfigDefault)
{
    const uint32_t mask = featureConfig->enabledFeatures;
    const uint32_t defaultMask = featureConfigDefault->enabledFeatures;
    for (uint32_t i = 0; featureNames[i]; i++) { // disabled features first
        if (strcmp(featureNames[i], emptyString) != 0) { //Skip unused
            const char *format = "feature -%s";
            cliDefaultPrintLinef(dumpMask, (defaultMask | ~mask) & (1 << i), format, featureNames[i]);
            cliDumpPrintLinef(dumpMask, (~defaultMask | mask) & (1 << i), format, featureNames[i]);
        }
    }
    for (uint32_t i = 0; featureNames[i]; i++) {  // enabled features
        if (strcmp(featureNames[i], emptyString) != 0) { //Skip unused
            const char *format = "feature %s";
            if (defaultMask & (1 << i)) {
                cliDefaultPrintLinef(dumpMask, (~defaultMask | mask) & (1 << i), format, featureNames[i]);
            }
            if (mask & (1 << i)) {
                cliDumpPrintLinef(dumpMask, (defaultMask | ~mask) & (1 << i), format, featureNames[i]);
            }
        }
    }
}

static void cliFeature(char *cmdline)
{
    uint32_t len = strlen(cmdline);
    uint32_t mask = featureMask();

    if (len == 0) {
        cliPrint("Enabled: ");
        for (uint32_t i = 0; ; i++) {
            if (featureNames[i] == NULL)
                break;
            if (mask & (1 << i))
                cliPrintf("%s ", featureNames[i]);
        }
        cliPrintLinefeed();
    } else if (strncasecmp(cmdline, "list", len) == 0) {
        cliPrint("Available:");
        for (uint32_t i = 0; ; i++) {
            if (featureNames[i] == NULL)
                break;
            if (strcmp(featureNames[i], emptyString) != 0) //Skip unused
                cliPrintf(" %s", featureNames[i]);
        }
        cliPrintLinefeed();
        return;
    } else {
        bool remove = false;
        if (cmdline[0] == '-') {
            // remove feature
            remove = true;
            cmdline++; // skip over -
            len--;
        }

        for (uint32_t i = 0; ; i++) {
            if (featureNames[i] == NULL) {
                cliPrintErrorLinef("Invalid name");
                break;
            }

            if (strncasecmp(cmdline, featureNames[i], len) == 0) {

                mask = 1 << i;
#ifndef USE_GPS
                if (mask & FEATURE_GPS) {
                    cliPrintLine("unavailable");
                    break;
                }
#endif
#ifndef USE_RANGEFINDER
                if (mask & FEATURE_RANGEFINDER) {
                    cliPrintLine("unavailable");
                    break;
                }
#endif
                if (remove) {
                    featureClear(mask);
                    cliPrint("Disabled");
                } else {
                    featureSet(mask);
                    cliPrint("Enabled");
                }
                cliPrintLinef(" %s", featureNames[i]);
                break;
            }
        }
    }
}

#if defined(USE_BEEPER)
static void printBeeper(uint8_t dumpMask, const uint32_t offFlags, const uint32_t offFlagsDefault, const char *name, const uint32_t allowedFlags)
{
    const uint8_t beeperCount = beeperTableEntryCount();
    for (int32_t i = 0; i < beeperCount - 1; i++) {
        if (beeperModeMaskForTableIndex(i) & allowedFlags) {
            const char *formatOff = "%s -%s";
            const char *formatOn = "%s %s";
            const uint32_t beeperModeMask = beeperModeMaskForTableIndex(i);
            cliDefaultPrintLinef(dumpMask, ~(offFlags ^ offFlagsDefault) & beeperModeMask, offFlags & beeperModeMask ? formatOn : formatOff, name, beeperNameForTableIndex(i));
            cliDumpPrintLinef(dumpMask, ~(offFlags ^ offFlagsDefault) & beeperModeMask, offFlags & beeperModeMask ? formatOff : formatOn, name, beeperNameForTableIndex(i));
        }
    }
}

static void processBeeperCommand(char *cmdline, uint32_t *offFlags, const uint32_t allowedFlags)
{
    uint32_t len = strlen(cmdline);
    uint8_t beeperCount = beeperTableEntryCount();

    if (len == 0) {
        cliPrintf("Disabled:");
        for (int32_t i = 0; ; i++) {
            if (i == beeperCount - 1) {
                if (*offFlags == 0)
                    cliPrint("  none");
                break;
            }

            if (beeperModeMaskForTableIndex(i) & *offFlags)
                cliPrintf("  %s", beeperNameForTableIndex(i));
        }
        cliPrintLinefeed();
    } else if (strncasecmp(cmdline, "list", len) == 0) {
        cliPrint("Available:");
        for (uint32_t i = 0; i < beeperCount; i++) {
            if (beeperModeMaskForTableIndex(i) & allowedFlags) {
                cliPrintf(" %s", beeperNameForTableIndex(i));
            }
        }
        cliPrintLinefeed();
    } else {
        bool remove = false;
        if (cmdline[0] == '-') {
            remove = true;     // this is for beeper OFF condition
            cmdline++;
            len--;
        }

        for (uint32_t i = 0; ; i++) {
            if (i == beeperCount) {
                cliPrintErrorLinef("Invalid name");
                break;
            }
            if (strncasecmp(cmdline, beeperNameForTableIndex(i), len) == 0 && beeperModeMaskForTableIndex(i) & (allowedFlags | BEEPER_GET_FLAG(BEEPER_ALL))) {
                if (remove) { // beeper off
                    if (i == BEEPER_ALL - 1) {
                        *offFlags = allowedFlags;
                    } else {
                        *offFlags |= beeperModeMaskForTableIndex(i);
                    }
                    cliPrint("Disabled");
                }
                else { // beeper on
                    if (i == BEEPER_ALL - 1) {
                        *offFlags = 0;
                    } else {
                        *offFlags &= ~beeperModeMaskForTableIndex(i);
                    }
                    cliPrint("Enabled");
                }
            cliPrintLinef(" %s", beeperNameForTableIndex(i));
            break;
            }
        }
    }
}

#if defined(USE_DSHOT)
static void cliBeacon(char *cmdline)
{
    processBeeperCommand(cmdline, &(beeperConfigMutable()->dshotBeaconOffFlags), DSHOT_BEACON_ALLOWED_MODES);
}
#endif

static void cliBeeper(char *cmdline)
{
    processBeeperCommand(cmdline, &(beeperConfigMutable()->beeper_off_flags), BEEPER_ALLOWED_MODES);
}
#endif

#ifdef USE_RX_SPI
    void cliRxBind(char *cmdline){
    UNUSED(cmdline);
    switch (rxSpiConfig()->rx_spi_protocol) {
#ifdef USE_RX_CC2500_BIND
    case RX_SPI_FRSKY_D:
    case RX_SPI_FRSKY_X:
    case RX_SPI_SFHSS:
        cc2500SpiBind();


        cliPrint("Binding...");

        break;
#endif
    default:
        cliPrint("Not supported.");

        break;
    }
}
#endif

static void printMap(uint8_t dumpMask, const rxConfig_t *rxConfig, const rxConfig_t *defaultRxConfig)
{
    bool equalsDefault = true;
    char buf[16];
    char bufDefault[16];
    uint32_t i;
    for (i = 0; i < RX_MAPPABLE_CHANNEL_COUNT; i++) {
        buf[rxConfig->rcmap[i]] = rcChannelLetters[i];
        if (defaultRxConfig) {
            bufDefault[defaultRxConfig->rcmap[i]] = rcChannelLetters[i];
            equalsDefault = equalsDefault && (rxConfig->rcmap[i] == defaultRxConfig->rcmap[i]);
        }
    }
    buf[i] = '\0';

    const char *formatMap = "map %s";
    if (defaultRxConfig) {
        bufDefault[i] = '\0';
        cliDefaultPrintLinef(dumpMask, equalsDefault, formatMap, bufDefault);
    }
    cliDumpPrintLinef(dumpMask, equalsDefault, formatMap, buf);
}


static void cliMap(char *cmdline)
{
    uint32_t i;
    char buf[RX_MAPPABLE_CHANNEL_COUNT + 1];

    uint32_t len = strlen(cmdline);
    if (len == RX_MAPPABLE_CHANNEL_COUNT) {

        for (i = 0; i < RX_MAPPABLE_CHANNEL_COUNT; i++) {
            buf[i] = toupper((unsigned char)cmdline[i]);
        }
        buf[i] = '\0';

        for (i = 0; i < RX_MAPPABLE_CHANNEL_COUNT; i++) {
            buf[i] = toupper((unsigned char)cmdline[i]);

            if (strchr(rcChannelLetters, buf[i]) && !strchr(buf + i + 1, buf[i]))
                continue;

            cliShowParseError();
            return;
        }
        parseRcChannels(buf, rxConfigMutable());
    } else if (len > 0) {
        cliShowParseError();
        return;
    }

    for (i = 0; i < RX_MAPPABLE_CHANNEL_COUNT; i++) {
        buf[rxConfig()->rcmap[i]] = rcChannelLetters[i];
    }

    buf[i] = '\0';
    cliPrintLinef("map %s", buf);
}

static char *skipSpace(char *buffer)
{
    while (*(buffer) == ' ') {
        buffer++;
    }

    return buffer;
}

static char *checkCommand(char *cmdLine, const char *command)
{
    if (!strncasecmp(cmdLine, command, strlen(command))   // command names match
        && (isspace((unsigned)cmdLine[strlen(command)]) || cmdLine[strlen(command)] == 0)) {
        return skipSpace(cmdLine + strlen(command) + 1);
    } else {
        return 0;
    }
}

static void cliRebootEx(bool bootLoader)
{
    cliPrint("\r\nRebooting");
    bufWriterFlush(cliWriter);
    waitForSerialPortToFinishTransmitting(cliPort);
    stopPwmAllMotors();
    if (bootLoader) {
        systemResetToBootloader();
        return;
    }
    systemReset();
}

static void cliReboot(void)
{
    cliRebootEx(false);
}

static void cliBootloader(char *cmdLine)
{
    UNUSED(cmdLine);

    cliPrintHashLine("restarting in bootloader mode");
    cliRebootEx(true);
}


#ifdef MSP_OVER_CLI
static void hex2byte(char *string, uint8_t *output)
{
    char tempBuff[3];
    tempBuff[0] = string[0];
    tempBuff[1] = string[1];
    tempBuff[2] = 0;
    *output = (uint8_t)strtol(tempBuff, NULL, 16);
}
#endif
#ifdef USE_GYRO_IMUF9001


static void cliImufBootloaderMode(char *cmdline)
{
    (void)(cmdline);
    if(imufBootloader())
    {
        cliPrintLine("BOOTLOADER");
    }
    else
    {
        cliPrintLine("FAIL");
    }
}


static void cliImufLoadBin(char *cmdline)
{
    #define TEMP_BUFF 256
    uint32_t dataSize;
    uint8_t output;
    uint8_t dataBuff[TEMP_BUFF] = {0,};
    uint32_t x;

    if(cmdline[0] == '!')
    {
        imuf_bin_safe = 1;
        imuf_buff_ptr = 0;
        imuf_checksum = 0;
        memset(imuf_custom_buff, 0, IMUF_CUSTOM_BUFF_LENGTH);
        cliPrintLine("SUCCESS");
    }
    else if(cmdline[0] == '.')
    {
        cliPrintLinef("%d", imuf_buff_ptr);
    }
    else if(cmdline[0] == 'c')
    {
        cliPrintLinef("%d", imuf_checksum);
    }
    else if(cmdline[0] == 'l')
    {
        if (imuf_bin_safe)
        {
            //get the datasize
            hex2byte(&cmdline[1], &output);
            dataSize  = ((output & 0xff) << 0 );
            hex2byte(&cmdline[3], &output);
            dataSize += ((output & 0xff) << 8 );
            hex2byte(&cmdline[5], &output);
            dataSize += ((output & 0xff) << 16);
            hex2byte(&cmdline[7], &output);
            dataSize += ((output & 0xff) << 24);

            if(dataSize < TEMP_BUFF)
            {
                //fill the temp buffer
                for(x=0; x< dataSize; x++)
                {
                    hex2byte(&cmdline[(x*2)+9], &output);
                    dataBuff[x] = output;
                    imuf_checksum += output;
                    //cliPrintLinef("out:%d:%d:%d:%d", dataSize, x, (x*2)+9, output, checksum);
                }
                if ( (imuf_buff_ptr+dataSize) < IMUF_CUSTOM_BUFF_LENGTH )
                {
                    memcpy(imuf_custom_buff+imuf_buff_ptr, dataBuff, dataSize);
                    imuf_buff_ptr += dataSize;
                    cliPrintLine("LOADED");
                }
                else
                {
                    cliPrintLine("WOAH!");
                }
            }
            else
            {
                cliPrintLine("CRAP!");
            }
        }
        else
        {
            cliPrintLine("PFFFT!");
        }
    }
}

static void cliImufFlashBin(char *cmdline)
{
    (void)(cmdline);
    if (imufUpdate(imuf_custom_buff, imuf_buff_ptr))
    {
        cliPrintLine("SUCCESS");
        bufWriterFlush(cliWriter);
        delay(5000);

        *cliBuffer = '\0';
        bufferIndex = 0;
        cliMode = 0;
        // incase a motor was left running during motortest, clear it here
        mixerResetDisarmedMotors();
        cliReboot();

        cliWriter = NULL;
    }
}
#endif

#ifdef MSP_OVER_CLI
sbuf_t buft;
uint8_t bufPtr[256];

void cliMsp(char *cmdline){
    int len = strlen(cmdline);
    if (len == 0) {
        cliPrintLine("No MSP command present");

        return;
    } else {
        uint8_t mspCommand = atoi(cmdline);
        uint8_t start = 2;
        if (mspCommand > 99) {
            start = 4;
        } else if (mspCommand > 9) {
            start= 3;
        }
        uint8_t inBuff[len];
        uint8_t output;
        for (int i = 0; i < len; i++) {
            hex2byte(&cmdline[(i*2) + start], &output);
            inBuff[i] = output;
        }
        sbuf_t inBuf = {.ptr = inBuff, .end = &inBuff[len-1]};
        //TODO need to fill inPtr with the rest of the bytes from the command line

        buft.ptr = buft.end = bufPtr;
        if (mspCommonProcessOutCommand(mspCommand, &buft, NULL) || mspProcessOutCommand(mspCommand, &buft)
          || mspCommonProcessInCommand(mspCommand, &inBuf, NULL) > -1 || mspProcessInCommand(mspCommand, &inBuf) > -1)
        {

            bufWriterAppend(cliWriter, '.');                 //"." is success
            bufWriterAppend(cliWriter, mspCommand);          //msp command sent
            bufWriterAppend(cliWriter, inBuf.ptr - inBuf.end);                  //msp command sent
            bufWriterAppend(cliWriter, buft.ptr - buft.end); //number of chars

            while (buft.end <= buft.ptr)
                bufWriterAppend(cliWriter, *(buft.end)++); //send data
        }
        else
        {
            bufWriterAppend(cliWriter, '!'); //"!" is failure
        }
    }
}
#endif

static void cliExit(char *cmdline)
{
    UNUSED(cmdline);

    cliPrintHashLine("leaving CLI mode, unsaved changes lost");
    bufWriterFlush(cliWriter);

    *cliBuffer = '\0';
    bufferIndex = 0;
    cliMode = 0;
    // incase a motor was left running during motortest, clear it here
    mixerResetDisarmedMotors();
    cliReboot();

    cliWriter = NULL;
}

#ifdef USE_GPS
static void cliGpsPassthrough(char *cmdline)
{
    UNUSED(cmdline);

    gpsEnablePassthrough(cliPort);
}
#endif

#if defined(USE_GYRO_REGISTER_DUMP) && !defined(SIMULATOR_BUILD)
static void cliPrintGyroRegisters(uint8_t whichSensor)
{
    cliPrintLinef("# WHO_AM_I    0x%X", gyroReadRegister(whichSensor, MPU_RA_WHO_AM_I));
    cliPrintLinef("# CONFIG      0x%X", gyroReadRegister(whichSensor, MPU_RA_CONFIG));
    cliPrintLinef("# GYRO_CONFIG 0x%X", gyroReadRegister(whichSensor, MPU_RA_GYRO_CONFIG));
}

static void cliDumpGyroRegisters(char *cmdline)
{
#ifdef USE_DUAL_GYRO
    if ((gyroConfig()->gyro_to_use == GYRO_CONFIG_USE_GYRO_1) || (gyroConfig()->gyro_to_use == GYRO_CONFIG_USE_GYRO_BOTH)) {
        cliPrintLinef("\r\n# Gyro 1");
        cliPrintGyroRegisters(GYRO_CONFIG_USE_GYRO_1);
    }
    if ((gyroConfig()->gyro_to_use == GYRO_CONFIG_USE_GYRO_2) || (gyroConfig()->gyro_to_use == GYRO_CONFIG_USE_GYRO_BOTH)) {
        cliPrintLinef("\r\n# Gyro 2");
        cliPrintGyroRegisters(GYRO_CONFIG_USE_GYRO_2);
    }
#else
    cliPrintGyroRegisters(GYRO_CONFIG_USE_GYRO_1);
#endif // USE_DUAL_GYRO
    UNUSED(cmdline);
}
#endif


static int parseOutputIndex(char *pch, bool allowAllEscs) {
    int outputIndex = atoi(pch);
    if ((outputIndex >= 0) && (outputIndex < getMotorCount())) {
        cliPrintLinef("Using output %d.", outputIndex);
    } else if (allowAllEscs && outputIndex == ALL_MOTORS) {
        cliPrintLinef("Using all outputs.");
    } else {
        cliPrintErrorLinef("Invalid output number. Range: 0  %d.", getMotorCount() - 1);

        return -1;
    }

    return outputIndex;
}

#if defined(USE_DSHOT)
#if defined(USE_ESC_SENSOR) && defined(USE_ESC_SENSOR_INFO)

#define ESC_INFO_KISS_V1_EXPECTED_FRAME_SIZE 15
#define ESC_INFO_KISS_V2_EXPECTED_FRAME_SIZE 21
#define ESC_INFO_BLHELI32_EXPECTED_FRAME_SIZE 64

enum {
    ESC_INFO_KISS_V1,
    ESC_INFO_KISS_V2,
    ESC_INFO_BLHELI32
};

#define ESC_INFO_VERSION_POSITION 12

void printEscInfo(const uint8_t *escInfoBuffer, uint8_t bytesRead)
{
    bool escInfoReceived = false;
    if (bytesRead > ESC_INFO_VERSION_POSITION) {
        uint8_t escInfoVersion;
        uint8_t frameLength;
        if (escInfoBuffer[ESC_INFO_VERSION_POSITION] == 254) {
            escInfoVersion = ESC_INFO_BLHELI32;
            frameLength = ESC_INFO_BLHELI32_EXPECTED_FRAME_SIZE;
        } else if (escInfoBuffer[ESC_INFO_VERSION_POSITION] == 255) {
            escInfoVersion = ESC_INFO_KISS_V2;
            frameLength = ESC_INFO_KISS_V2_EXPECTED_FRAME_SIZE;
        } else {
            escInfoVersion = ESC_INFO_KISS_V1;
            frameLength = ESC_INFO_KISS_V1_EXPECTED_FRAME_SIZE;
        }

        if (bytesRead == frameLength) {
            escInfoReceived = true;

            if (calculateCrc8(escInfoBuffer, frameLength - 1) == escInfoBuffer[frameLength - 1]) {
                uint8_t firmwareVersion = 0;
                uint8_t firmwareSubVersion = 0;
                uint8_t escType = 0;
                switch (escInfoVersion) {
                case ESC_INFO_KISS_V1:
                    firmwareVersion = escInfoBuffer[12];
                    firmwareSubVersion = (escInfoBuffer[13] & 0x1f) + 97;
                    escType = (escInfoBuffer[13] & 0xe0) >> 5;

                    break;
                case ESC_INFO_KISS_V2:
                    firmwareVersion = escInfoBuffer[13];
                    firmwareSubVersion = escInfoBuffer[14];
                    escType = escInfoBuffer[15];

                    break;
                case ESC_INFO_BLHELI32:
                    firmwareVersion = escInfoBuffer[13];
                    firmwareSubVersion = escInfoBuffer[14];
                    escType = escInfoBuffer[15];

                    break;
                }

                cliPrint("ESC Type: ");
                switch (escInfoVersion) {
                case ESC_INFO_KISS_V1:
                case ESC_INFO_KISS_V2:
                    switch (escType) {
                    case 1:
                        cliPrintLine("KISS8A");

                        break;
                    case 2:
                        cliPrintLine("KISS16A");

                        break;
                    case 3:
                        cliPrintLine("KISS24A");

                        break;
                    case 5:
                        cliPrintLine("KISS Ultralite");

                        break;
                    default:
                        cliPrintLine("unknown");

                        break;
                    }

                    break;
                case ESC_INFO_BLHELI32:
                    {
                        char *escType = (char *)(escInfoBuffer + 31);
                        escType[32] = 0;
                        cliPrintLine(escType);
                    }

                    break;
                }

                cliPrint("MCU Serial No: 0x");
                for (int i = 0; i < 12; i++) {
                    if (i && (i % 3 == 0)) {
                        cliPrint("-");
                    }
                    cliPrintf("%02x", escInfoBuffer[i]);
                }
                cliPrintLinefeed();

                switch (escInfoVersion) {
                case ESC_INFO_KISS_V1:
                case ESC_INFO_KISS_V2:
                    cliPrintLinef("Firmware Version: %d.%02d%c", firmwareVersion / 100, firmwareVersion % 100, (char)firmwareSubVersion);

                    break;
                case ESC_INFO_BLHELI32:
                    cliPrintLinef("Firmware Version: %d.%02d%", firmwareVersion, firmwareSubVersion);

                    break;
                }
                if (escInfoVersion == ESC_INFO_KISS_V2 || escInfoVersion == ESC_INFO_BLHELI32) {
                    cliPrintLinef("Rotation Direction: %s", escInfoBuffer[16] ? "reversed" : "normal");
                    cliPrintLinef("3D: %s", escInfoBuffer[17] ? "on" : "off");
                    if (escInfoVersion == ESC_INFO_BLHELI32) {
                        uint8_t setting = escInfoBuffer[18];
                        cliPrint("Low voltage Limit: ");
                        switch (setting) {
                        case 0:
                            cliPrintLine("off");

                            break;
                        case 255:
                            cliPrintLine("unsupported");

                            break;
                        default:
                            cliPrintLinef("%d.%01d", setting / 10, setting % 10);

                            break;
                        }

                        setting = escInfoBuffer[19];
                        cliPrint("Current Limit: ");
                        switch (setting) {
                        case 0:
                            cliPrintLine("off");

                            break;
                        case 255:
                            cliPrintLine("unsupported");

                            break;
                        default:
                            cliPrintLinef("%d", setting);

                            break;
                        }

                        for (int i = 0; i < 4; i++) {
                            setting = escInfoBuffer[i + 20];
                            cliPrintLinef("LED %d: %s", i, setting ? (setting == 255) ? "unsupported" : "on" : "off");
                        }
                    }
                }
            } else {
                cliPrintErrorLinef("Checksum Error.");
            }
        }
    }

    if (!escInfoReceived) {
        cliPrintLine("No Info.");
    }
}

static void executeEscInfoCommand(uint8_t escIndex)
{
    cliPrintLinef("Info for ESC %d:", escIndex);

    uint8_t escInfoBuffer[ESC_INFO_BLHELI32_EXPECTED_FRAME_SIZE];

    startEscDataRead(escInfoBuffer, ESC_INFO_BLHELI32_EXPECTED_FRAME_SIZE);

    pwmWriteDshotCommand(escIndex, getMotorCount(), DSHOT_CMD_ESC_INFO, true);

    delay(10);

    printEscInfo(escInfoBuffer, getNumberEscBytesRead());
}
#endif // USE_ESC_SENSOR && USE_ESC_SENSOR_INFO

static void cliDshotProg(char *cmdline)
{
    if (isEmpty(cmdline) || motorConfig()->dev.motorPwmProtocol < PWM_TYPE_DSHOT150) {
        cliShowParseError();

        return;
    }

    char *saveptr;
    char *pch = strtok_r(cmdline, " ", &saveptr);
    int pos = 0;
    int escIndex = 0;
    bool firstCommand = true;
    while (pch != NULL) {
        switch (pos) {
        case 0:
            escIndex = parseOutputIndex(pch, true);
            if (escIndex == -1) {
                return;
            }

            break;
        default:
            {
                int command = atoi(pch);
                if (command >= 0 && command < DSHOT_MIN_THROTTLE) {
                    if (firstCommand) {
                        pwmDisableMotors();

                        if (command == DSHOT_CMD_ESC_INFO) {
                            delay(5); // Wait for potential ESC telemetry transmission to finish
                        } else {
                            delay(1);
                        }

                        firstCommand = false;
                    }

                    if (command != DSHOT_CMD_ESC_INFO) {
                        pwmWriteDshotCommand(escIndex, getMotorCount(), command, true);
                    } else {
#if defined(USE_ESC_SENSOR) && defined(USE_ESC_SENSOR_INFO)
                        if (feature(FEATURE_ESC_SENSOR)) {
                            if (escIndex != ALL_MOTORS) {
                                executeEscInfoCommand(escIndex);
                            } else {
                                for (uint8_t i = 0; i < getMotorCount(); i++) {
                                    executeEscInfoCommand(i);
                                }
                            }
                        } else
#endif
                        {
                            cliPrintLine("Not supported.");
                        }
                    }

                    cliPrintLinef("Command Sent: %d", command);

                } else {
                    cliPrintErrorLinef("Invalid command. Range: 1 - %d.", DSHOT_MIN_THROTTLE - 1);
                }
            }

            break;
        }

        pos++;
        pch = strtok_r(NULL, " ", &saveptr);
    }

    pwmEnableMotors();
}
#endif // USE_DSHOT

#ifdef USE_ESCSERIAL
static void cliEscPassthrough(char *cmdline)
{
    if (isEmpty(cmdline)) {
        cliShowParseError();

        return;
    }

    char *saveptr;
    char *pch = strtok_r(cmdline, " ", &saveptr);
    int pos = 0;
    uint8_t mode = 0;
    int escIndex = 0;
    while (pch != NULL) {
        switch (pos) {
        case 0:
            if (strncasecmp(pch, "sk", strlen(pch)) == 0) {
                mode = PROTOCOL_SIMONK;
            } else if (strncasecmp(pch, "bl", strlen(pch)) == 0) {
                mode = PROTOCOL_BLHELI;
            } else if (strncasecmp(pch, "ki", strlen(pch)) == 0) {
                mode = PROTOCOL_KISS;
            } else if (strncasecmp(pch, "cc", strlen(pch)) == 0) {
                mode = PROTOCOL_KISSALL;
            } else {
                cliShowParseError();

                return;
            }
            break;
        case 1:
            escIndex = parseOutputIndex(pch, mode == PROTOCOL_KISS);
            if (escIndex == -1) {
                return;
            }

            break;
        default:
            cliShowParseError();

            return;

            break;

        }
        pos++;
        pch = strtok_r(NULL, " ", &saveptr);
    }

    escEnablePassthrough(cliPort, escIndex, mode);
}
#endif

#ifndef USE_QUAD_MIXER_ONLY
static void cliMixer(char *cmdline)
{
    int len;

    len = strlen(cmdline);

    if (len == 0) {
        cliPrintLinef("Mixer: %s", mixerNames[mixerConfig()->mixerMode - 1]);
        return;
    } else if (strncasecmp(cmdline, "list", len) == 0) {
        cliPrint("Available:");
        for (uint32_t i = 0; ; i++) {
            if (mixerNames[i] == NULL)
                break;
            cliPrintf(" %s", mixerNames[i]);
        }
        cliPrintLinefeed();
        return;
    }

    for (uint32_t i = 0; ; i++) {
        if (mixerNames[i] == NULL) {
            cliPrintErrorLinef("Invalid name");
            return;
        }
        if (strncasecmp(cmdline, mixerNames[i], len) == 0) {
            mixerConfigMutable()->mixerMode = i + 1;
            break;
        }
    }

    cliMixer("");
}
#endif

static void cliMotor(char *cmdline)
{
    if (isEmpty(cmdline)) {
        cliShowParseError();

        return;
    }

    int motorIndex = 0;
    int motorValue = 0;

    char *saveptr;
    char *pch = strtok_r(cmdline, " ", &saveptr);
    int index = 0;
    while (pch != NULL) {
        switch (index) {
        case 0:
            motorIndex = parseOutputIndex(pch, true);
            if (motorIndex == -1) {
                return;
            }

            break;
        case 1:
            motorValue = atoi(pch);

            break;
        }
        index++;
        pch = strtok_r(NULL, " ", &saveptr);
    }

    if (index == 2) {
        if (motorValue < PWM_RANGE_MIN || motorValue > PWM_RANGE_MAX) {
            cliShowArgumentRangeError("value", 1000, 2000);
        } else {
            uint32_t motorOutputValue = convertExternalToMotor(motorValue);

            if (motorIndex != ALL_MOTORS) {
                motor_disarmed[motorIndex] = motorOutputValue;

                cliPrintLinef("motor %d: %d", motorIndex, motorOutputValue);
            } else  {
                for (int i = 0; i < getMotorCount(); i++) {
                    motor_disarmed[i] = motorOutputValue;
                }

                cliPrintLinef("all motors: %d", motorOutputValue);
            }
        }
    } else {
        cliShowParseError();
    }
}

#ifndef MINIMAL_CLI
static void cliPlaySound(char *cmdline)
{
    int i;
    const char *name;
    static int lastSoundIdx = -1;

    if (isEmpty(cmdline)) {
        i = lastSoundIdx + 1;     //next sound index
        if ((name=beeperNameForTableIndex(i)) == NULL) {
            while (true) {   //no name for index; try next one
                if (++i >= beeperTableEntryCount())
                    i = 0;   //if end then wrap around to first entry
                if ((name=beeperNameForTableIndex(i)) != NULL)
                    break;   //if name OK then play sound below
                if (i == lastSoundIdx + 1) {     //prevent infinite loop
                    cliPrintErrorLinef("Error playing sound");
                    return;
                }
            }
        }
    } else {       //index value was given
        i = atoi(cmdline);
        if ((name=beeperNameForTableIndex(i)) == NULL) {
            cliPrintLinef("No sound for index %d", i);
            return;
        }
    }
    lastSoundIdx = i;
    beeperSilence();
    cliPrintLinef("Playing sound %d: %s", i, name);
    beeper(beeperModeForTableIndex(i));
}
#endif

static void cliProfile(char *cmdline)
{
    if (isEmpty(cmdline)) {
        cliPrintLinef("profile %d", getPidProfileIndexToUse());
        return;
    } else {
        const int i = atoi(cmdline);
        if (i >= 0 && i < PID_PROFILE_COUNT) {
            changePidProfile(i);
            cliProfile("");
        } else {
            cliPrintErrorLinef("PROFILE OUTSIDE OF [0..%d]", PID_PROFILE_COUNT - 1);
        }
    }
}

static void cliRateProfile(char *cmdline)
{
    if (isEmpty(cmdline)) {
        cliPrintLinef("rateprofile %d", getRateProfileIndexToUse());
        return;
    } else {
        const int i = atoi(cmdline);
        if (i >= 0 && i < CONTROL_RATE_PROFILE_COUNT) {
            changeControlRateProfile(i);
            cliRateProfile("");
        } else {
            cliPrintErrorLinef("RATE PROFILE OUTSIDE OF [0..%d]", CONTROL_RATE_PROFILE_COUNT - 1);
        }
    }
}

static void cliDumpPidProfile(uint8_t pidProfileIndex, uint8_t dumpMask)
{
    if (pidProfileIndex >= PID_PROFILE_COUNT) {
        // Faulty values
        return;
    }

    pidProfileIndexToUse = pidProfileIndex;

    cliPrintHashLine("profile");
    cliProfile("");
    cliPrintLinefeed();
    dumpAllValues(PROFILE_VALUE, dumpMask);

    pidProfileIndexToUse = CURRENT_PROFILE_INDEX;
}

static void cliDumpRateProfile(uint8_t rateProfileIndex, uint8_t dumpMask)
{
    if (rateProfileIndex >= CONTROL_RATE_PROFILE_COUNT) {
        // Faulty values
        return;
    }

    rateProfileIndexToUse = rateProfileIndex;

    cliPrintHashLine("rateprofile");
    cliRateProfile("");
    cliPrintLinefeed();
    dumpAllValues(PROFILE_RATE_VALUE, dumpMask);

    rateProfileIndexToUse = CURRENT_PROFILE_INDEX;
}

static void cliSave(char *cmdline)
{
    UNUSED(cmdline);

    cliPrintHashLine("saving");

#if defined(USE_BOARD_INFO)
    if (boardInformationUpdated) {
        persistBoardInformation();
    }
#if defined(USE_SIGNATURE)
    if (signatureUpdated) {
        persistSignature();
    }
#endif
#endif // USE_BOARD_INFO

    writeEEPROM();

    cliReboot();
}

static void cliDefaults(char *cmdline)
{
    bool saveConfigs;

    if (isEmpty(cmdline)) {
        saveConfigs = true;
    } else if (strncasecmp(cmdline, "nosave", 6) == 0) {
        saveConfigs = false;
    } else {
        return;
    }

    cliPrintHashLine("resetting to defaults");

    resetConfigs();

    if (saveConfigs) {
        cliSave(NULL);
    }
}

void cliPrintVarDefault(const clivalue_t *value)
{
    const pgRegistry_t *pg = pgFind(value->pgn);
    if (pg) {
        const char *defaultFormat = "Default value: ";
        const int valueOffset = getValueOffset(value);
        const bool equalsDefault = valuePtrEqualsDefault(value, pg->copy + valueOffset, pg->address + valueOffset);
        if (!equalsDefault) {
            cliPrintf(defaultFormat, value->name);
            printValuePointer(value, (uint8_t*)pg->address + valueOffset, false);
            cliPrintLinefeed();
        }
    }
}

STATIC_UNIT_TESTED void cliGet(char *cmdline)
{
    const clivalue_t *val;
    int matchedCommands = 0;

    pidProfileIndexToUse = getCurrentPidProfileIndex();
    rateProfileIndexToUse = getCurrentControlRateProfileIndex();

    backupAndResetConfigs();

    for (uint32_t i = 0; i < valueTableEntryCount; i++) {
        if (strcasestr(valueTable[i].name, cmdline)) {
            val = &valueTable[i];
            if (matchedCommands > 0) {
                cliPrintLinefeed();
            }
            cliPrintf("%s = ", valueTable[i].name);
            cliPrintVar(val, 0);
            cliPrintLinefeed();
            switch (val->type & VALUE_SECTION_MASK) {
            case PROFILE_VALUE:
                cliProfile("");

                break;
            case PROFILE_RATE_VALUE:
                cliRateProfile("");

                break;
            default:

                break;
            }
            cliPrintVarRange(val);
            cliPrintVarDefault(val);
            matchedCommands++;
        }
    }

    restoreConfigs();

    pidProfileIndexToUse = CURRENT_PROFILE_INDEX;
    rateProfileIndexToUse = CURRENT_PROFILE_INDEX;

    if (matchedCommands) {
        return;
    }

    cliPrintErrorLinef("Invalid name");
}

#ifdef USE_PEGASUS_UI
static const char * valueSectionMask[] = {
    "GLOBAL", "PROFILE", "RATE"
};

static const char * valueTypeMask[] = {
    "UINT8", "INT8", "UINT16", "INT16"
};

static const char * valueModeMask[] = {
    "DIRECT", "LOOKUP", "ARRAY", "BITMASK"
};

void cliPrintValueJson(int32_t i){
    const clivalue_t *var = &valueTable[i];
    cliPrintf("\"%s\":{\"scope\":\"%s\",\"type\":\"%s\",\"mode\":\"%s\",\"current\":\"",
                var->name,
                valueSectionMask[((var->type & VALUE_SECTION_MASK) >> VALUE_SECTION_OFFSET)],
                valueTypeMask[((var->type & VALUE_TYPE_MASK) >> VALUE_TYPE_OFFSET)],
                valueModeMask[((var->type & VALUE_MODE_MASK) >> VALUE_MODE_OFFSET)]);
    cliPrintVar(var, false);
    cliPrint("\",\"default\":\"");
    const pgRegistry_t *pg = pgFind(var->pgn);
    const int valueOffset = getValueOffset(var);
    printValuePointer(var, (uint8_t*)pg->address + valueOffset, false);
    cliPrint("\"");
    if ((var->type & VALUE_MODE_MASK) == MODE_LOOKUP)
    {
        const lookupTableEntry_t *tableEntry = &lookupTables[var->config.lookup.tableIndex];
        cliPrint(",\"values\":[");
        for (int32_t i = 0; i < tableEntry->valueCount ; i++) {
            if (i > 0)
            {
                cliPrintLine(",");
            }
            cliPrintf("\"%s\"", tableEntry->values[i]);
        }
        cliPrint("]");
    }
    if ((var->type & VALUE_MODE_MASK) == MODE_DIRECT) {
        cliPrintf(",\"min\":\"%d\",\"max\":\"%d\"", var->config.minmax.min, var->config.minmax.max);
    }
    // if ((var->type & VALUE_MODE_MASK) == MODE_ARRAY) {
    //     cliPrintf(",\"min\":\"%d\",\"max\":\"%d\"", var->config.minmax.min, var->config.minmax.max);
    // }
    cliPrint("}");
}

static void printFeatureJson(const featureConfig_t *configCopy)
{
    const uint32_t mask = configCopy->enabledFeatures;
    const uint32_t defaultMask = featureConfig()->enabledFeatures;
    cliPrintf(",\"features\":{\"scope\":\"GLOBAL\",\"type\":\"UINT8\",\"mode\":\"ARRAY\",\"current\":\"%d\",\"values\":[", mask);
    for (uint32_t i = 0; featureNames[i]; i++) { // disabled features first
        if (strcmp(featureNames[i], emptyString) != 0) { //Skip unused
            if (i > 0)
            {
                cliPrint(",");
            }
            if ((~defaultMask | mask) & (1 << i)) {
                cliPrintf("\"-%s\"", featureNames[i]);
            } else {
                cliPrintf("\"%s\"", featureNames[i]);
            }
        }
    }
    cliPrintf("]}");
}
static void printSerialJson(const serialConfig_t *serialConfig)
{
    cliPrint(",\"ports\":{\"scope\":\"GLOBAL\",\"type\":\"UINT16\",\"mode\":\"ARRAY\",\"values\":[");
    for (uint32_t i = 0; i < SERIAL_PORT_COUNT && serialIsPortAvailable(serialConfig->portConfigs[i].identifier); i++) {
        if (i > 0)
        {
            cliPrint(",");
        }
        cliPrintf("\"%d|%d|%ld|%ld|%ld|%ld\"",
            serialConfig->portConfigs[i].identifier,
            serialConfig->portConfigs[i].functionMask,
            baudRates[serialConfig->portConfigs[i].msp_baudrateIndex],
            baudRates[serialConfig->portConfigs[i].gps_baudrateIndex],
            baudRates[serialConfig->portConfigs[i].telemetry_baudrateIndex],
            baudRates[serialConfig->portConfigs[i].blackbox_baudrateIndex]);
    }
    cliPrintf("]}");
}

static void printAuxJson(const modeActivationCondition_t *modeActivationConditions)
{
    cliPrint(",\"modes\":{\"scope\":\"GLOBAL\",\"type\":\"UINT16\",\"mode\":\"ARRAY\",\"values\":[");
    for (uint32_t i = 0; i < MAX_MODE_ACTIVATION_CONDITION_COUNT; i++) {
        if (i > 0)
        {
            cliPrint(",");
        }
        const modeActivationCondition_t *mac = &modeActivationConditions[i];
        const box_t *box = findBoxByBoxId(mac->modeId);
        if (box) {
            cliPrintf("\"%u|%u|%u|%u|%u|%u\"",
                i,
                box->permanentId,
                mac->auxChannelIndex,
                MODE_STEP_TO_CHANNEL_VALUE(mac->range.startStep),
                MODE_STEP_TO_CHANNEL_VALUE(mac->range.endStep),
                mac->modeLogic
            );
        }
    }
    cliPrintf("]}");
}

static void printResourceJson() {
    cliPrint(",\"resources\":{\"scope\":\"GLOBAL\",\"type\":\"string\",\"mode\":\"ARRAY\",\"values\":[");
    for (int i = 0; i < DEFIO_IO_USED_COUNT; i++) {
        if (i > 0)
        {
            cliPrint(",");
        }
        const char* owner;
        owner = ownerNames[ioRecs[i].owner];

        cliPrintf("\"%c%02d|%s|", IO_GPIOPortIdx(ioRecs + i) + 'A', IO_GPIOPinIdx(ioRecs + i), owner);
        if (ioRecs[i].index > 0) {
            cliPrintf("%d", ioRecs[i].index);
        } else {
            cliPrintf("0");
        }
        cliPrintf("\"");
        //cliPrintLinefeed();
    }
    cliPrintf("]}");
}

#define PROFILE_JSON_STRING ",\"%s_profile\":{\"scope\":\"GLOBAL\",\"type\":\"UINT8\",\"mode\":\"LOOKUP\",\"current\":\"%d\",\"values\":[{"

static void dumpProfileValueJson(uint16_t valueSection)
{
    bool foundFirst = false;
    for (uint32_t i = 0; i < valueTableEntryCount; i++) {
        const clivalue_t *value = &valueTable[i];
        if ((value->type & VALUE_SECTION_MASK) == valueSection) {
            if (foundFirst)
            {
                cliPrint(",");
            }
            cliPrintValueJson(i);
            foundFirst = true;
        }
    }
}

static void cliPidProfilesJson()
{
    cliPrintf(PROFILE_JSON_STRING, "pid", getCurrentPidProfileIndex());
    const uint8_t saved = systemConfig_Copy.pidProfileIndex;
    for (uint32_t i = 0; i < PID_PROFILE_COUNT; i++) {
        changePidProfile(i);
        if (i > 0)
        {
            cliPrint("},{");
        }
        dumpProfileValueJson(PROFILE_VALUE);
    }
    changePidProfile(saved);
    cliPrint("}]}");
}

static void cliRateProfilesJson()
{
    cliPrintf(PROFILE_JSON_STRING, "rate", getCurrentControlRateProfileIndex());
    const uint8_t saved = systemConfig_Copy.activeRateProfile;
    for (uint32_t i = 0; i < CONTROL_RATE_PROFILE_COUNT; i++) {
        changeControlRateProfile(i);
        if (i > 0)
        {
            cliPrint("},{");
        }
        dumpProfileValueJson(PROFILE_RATE_VALUE);
    }
    changeControlRateProfile(saved);
    cliPrint("}]}");
}

void cliConfig(char *cmdline)
{

    UNUSED(cmdline);
    cliPrintLine("{");
    for (uint32_t i = 0; i < valueTableEntryCount; i++)
    {
        if (i > 0)
        {
            cliPrintLine(",");
        }
        cliPrintValueJson(i);
    }
    cliPidProfilesJson();
    cliRateProfilesJson();
    printFeatureJson(&featureConfig_Copy);
    printSerialJson(serialConfig());
    printAuxJson(modeActivationConditions(0));
    printResourceJson();
    cliPrintf(",\"name\":\"%s\"", pilotConfig()->name);
    cliPrintf(",\"version\":\"%s|%s|%s|%s\"",
        FC_FIRMWARE_NAME,
        targetName,
        systemConfig()->boardIdentifier,
        FC_VERSION_STRING
    );
#ifdef USE_GYRO_IMUF9001
    cliPrintf(",\"imuf\":\"%lu\"", imufCurrentVersion);
#endif
    cliPrintLine("}");
}
#endif

static uint8_t getWordLength(char *bufBegin, char *bufEnd)
{
    while (*(bufEnd - 1) == ' ') {
        bufEnd--;
    }

    return bufEnd - bufBegin;
}

STATIC_UNIT_TESTED void cliSet(char *cmdline)
{
    const uint32_t len = strlen(cmdline);
    char *eqptr;

    if (len == 0 || (len == 1 && cmdline[0] == '*')) {
        cliPrintLine("Current settings: ");

        for (uint32_t i = 0; i < valueTableEntryCount; i++) {
            const clivalue_t *val = &valueTable[i];
            cliPrintf("%s = ", valueTable[i].name);
            cliPrintVar(val, len); // when len is 1 (when * is passed as argument), it will print min/max values as well, for gui
            cliPrintLinefeed();
        }
    } else if ((eqptr = strstr(cmdline, "=")) != NULL) {
        // has equals

        uint8_t variableNameLength = getWordLength(cmdline, eqptr);

        // skip the '=' and any ' ' characters
        eqptr++;
        eqptr = skipSpace(eqptr);

        for (uint32_t i = 0; i < valueTableEntryCount; i++) {
            const clivalue_t *val = &valueTable[i];

            // ensure exact match when setting to prevent setting variables with shorter names
            if (strncasecmp(cmdline, val->name, strlen(val->name)) == 0 && variableNameLength == strlen(val->name)) {

                bool valueChanged = false;
                int16_t value  = 0;
                switch (val->type & VALUE_MODE_MASK) {
                case MODE_DIRECT: {
                        int16_t value = atoi(eqptr);

                        if (value >= val->config.minmax.min && value <= val->config.minmax.max) {
                            cliSetVar(val, value);
                            valueChanged = true;
                        }
                    }

                    break;
                case MODE_LOOKUP:
                case MODE_BITSET: {
                        int tableIndex;
                        if ((val->type & VALUE_MODE_MASK) == MODE_BITSET) {
                            tableIndex = TABLE_OFF_ON;
                        } else {
                            tableIndex = val->config.lookup.tableIndex;
                        }
                        const lookupTableEntry_t *tableEntry = &lookupTables[tableIndex];
                        bool matched = false;
                        for (uint32_t tableValueIndex = 0; tableValueIndex < tableEntry->valueCount && !matched; tableValueIndex++) {
                            matched = tableEntry->values[tableValueIndex] && strcasecmp(tableEntry->values[tableValueIndex], eqptr) == 0;

                            if (matched) {
                                value = tableValueIndex;

                                cliSetVar(val, value);
                                valueChanged = true;
                            }
                        }
                    }

                    break;

                case MODE_ARRAY: {
                        const uint8_t arrayLength = val->config.array.length;
                        char *valPtr = eqptr;

                        int i = 0;
                        while (i < arrayLength && valPtr != NULL) {
                            // skip spaces
                            valPtr = skipSpace(valPtr);

                            // process substring starting at valPtr
                            // note: no need to copy substrings for atoi()
                            //       it stops at the first character that cannot be converted...
                            switch (val->type & VALUE_TYPE_MASK) {
                            default:
                            case VAR_UINT8:
                                {
                                    // fetch data pointer
                                    uint8_t *data = (uint8_t *)cliGetValuePointer(val) + i;
                                    // store value
                                    *data = (uint8_t)atoi((const char*) valPtr);
                                }

                                break;
                            case VAR_INT8:
                                {
                                    // fetch data pointer
                                    int8_t *data = (int8_t *)cliGetValuePointer(val) + i;
                                    // store value
                                    *data = (int8_t)atoi((const char*) valPtr);
                                }

                                break;
                            case VAR_UINT16:
                                {
                                    // fetch data pointer
                                    uint16_t *data = (uint16_t *)cliGetValuePointer(val) + i;
                                    // store value
                                    *data = (uint16_t)atoi((const char*) valPtr);
                                }

                                break;
                            case VAR_INT16:
                                {
                                    // fetch data pointer
                                    int16_t *data = (int16_t *)cliGetValuePointer(val) + i;
                                    // store value
                                    *data = (int16_t)atoi((const char*) valPtr);
                                }

                                break;
                            }

                            // find next comma (or end of string)
                            valPtr = strchr(valPtr, ',') + 1;

                            i++;
                        }
                    }

                    // mark as changed
                    valueChanged = true;

                    break;

                }

                if (valueChanged) {
                    cliPrintf("%s set to ", val->name);
                    cliPrintVar(val, 0);
                } else {
                    cliPrintErrorLinef("Invalid value");
                    cliPrintVarRange(val);
                }

                return;
            }
        }
        cliPrintErrorLinef("Invalid name");
    } else {
        // no equals, check for matching variables.
        cliGet(cmdline);
    }
}

static void cliStatus(char *cmdline)
{
    UNUSED(cmdline);

    cliPrintLinef("System Uptime: %d seconds", millis() / 1000);

    #ifdef USE_RTC_TIME
    char buf[FORMATTED_DATE_TIME_BUFSIZE];
    dateTime_t dt;
    if (rtcGetDateTime(&dt)) {
        dateTimeFormatLocal(buf, &dt);
        cliPrintLinef("Current Time: %s", buf);
    }
    #endif

    cliPrintLinef("Voltage: %d * 0.1V (%dS battery - %s)", getBatteryVoltage(), getBatteryCellCount(), getBatteryStateString());

    cliPrintf("CPU Clock=%dMHz", (SystemCoreClock / 1000000));

#ifdef USE_ADC_INTERNAL
    uint16_t vrefintMv = getVrefMv();
    int16_t coretemp = getCoreTemperatureCelsius();
    cliPrintf(", Vref=%d.%2dV, Core temp=%ddegC", vrefintMv / 1000, (vrefintMv % 1000) / 10, coretemp);
#endif

#if defined(USE_SENSOR_NAMES) && !defined(USE_GYRO_IMUF9001)
    const uint32_t detectedSensorsMask = sensorsMask();
    for (uint32_t i = 0; ; i++) {
        if (sensorTypeNames[i] == NULL) {
            break;
        }
        const uint32_t mask = (1 << i);
        if ((detectedSensorsMask & mask) && (mask & SENSOR_NAMES_MASK)) {
            const uint8_t sensorHardwareIndex = detectedSensors[i];
            const char *sensorHardware = sensorHardwareNames[i][sensorHardwareIndex];
            cliPrintf(", %s=%s", sensorTypeNames[i], sensorHardware);
            if (mask == SENSOR_ACC && acc.dev.revisionCode) {
                cliPrintf(".%c", acc.dev.revisionCode);
            }
        }
    }
#else
    #if defined(USE_GYRO_IMUF9001)
    UNUSED(sensorHardwareNames);
    UNUSED(sensorTypeNames);
    cliPrintf(" | IMU-F Version: %lu", imufCurrentVersion);
    #endif
#endif /* USE_SENSOR_NAMES */
    cliPrintLinefeed();

#ifdef USE_SDCARD
    cliSdInfo(NULL);
#endif

#ifdef USE_I2C
    const uint16_t i2cErrorCounter = i2cGetErrorCounter();
#else
    const uint16_t i2cErrorCounter = 0;
#endif

#ifdef STACK_CHECK
    cliPrintf("Stack used: %d, ", stackUsedSize());
#endif
    cliPrintLinef("Stack size: %d, Stack address: 0x%x", stackTotalSize(), stackHighMem());
#ifdef EEPROM_IN_RAM
#define CONFIG_SIZE EEPROM_SIZE
#else
#define CONFIG_SIZE (&__config_end - &__config_start)
#endif
    cliPrintLinef("I2C Errors: %d, config size: %d, max available config: %d", i2cErrorCounter, getEEPROMConfigSize(), CONFIG_SIZE);

    const int gyroRate = getTaskDeltaTime(TASK_GYROPID) == 0 ? 0 : (int)(1000000.0f / ((float)getTaskDeltaTime(TASK_GYROPID)));
    const int rxRate = currentRxRefreshRate == 0 ? 0 : (int)(1000000.0f / ((float)currentRxRefreshRate));
    const int systemRate = getTaskDeltaTime(TASK_SYSTEM) == 0 ? 0 : (int)(1000000.0f / ((float)getTaskDeltaTime(TASK_SYSTEM)));
    cliPrintLinef("CPU:%d%%, cycle time: %d, GYRO rate: %d, RX rate: %d, System rate: %d",
            constrain(averageSystemLoadPercent, 0, 100), getTaskDeltaTime(TASK_GYROPID), gyroRate, rxRate, systemRate);
    cliPrint("Arming disable flags:");
    armingDisableFlags_e flags = getArmingDisableFlags();
    while (flags) {
        const int bitpos = ffs(flags) - 1;
        flags &= ~(1 << bitpos);
        cliPrintf(" %s", armingDisableFlagNames[bitpos]);
    }
    cliPrintLinefeed();
}

#ifndef SKIP_TASK_STATISTICS
static void cliTasks(char *cmdline)
{
    UNUSED(cmdline);
    int maxLoadSum = 0;
    int averageLoadSum = 0;

#ifndef MINIMAL_CLI
    if (systemConfig()->task_statistics) {
        cliPrintLine("Task list             rate/hz  max/us  avg/us maxload avgload     total/ms");
    } else {
        cliPrintLine("Task list");
    }
#endif
    for (cfTaskId_e taskId = 0; taskId < TASK_COUNT; taskId++) {
        cfTaskInfo_t taskInfo;
        getTaskInfo(taskId, &taskInfo);
        if (taskInfo.isEnabled) {
            int taskFrequency;
            int subTaskFrequency = 0;
            if (taskId == TASK_GYROPID) {
                subTaskFrequency = taskInfo.latestDeltaTime == 0 ? 0 : (int)(1000000.0f / ((float)taskInfo.latestDeltaTime));
                taskFrequency = subTaskFrequency / pidConfig()->pid_process_denom;
                if (pidConfig()->pid_process_denom > 1) {
                    cliPrintf("%02d - (%15s) ", taskId, taskInfo.taskName);
                } else {
                    taskFrequency = subTaskFrequency;
                    cliPrintf("%02d - (%11s/%3s) ", taskId, taskInfo.subTaskName, taskInfo.taskName);
                }
            } else {
                taskFrequency = taskInfo.latestDeltaTime == 0 ? 0 : (int)(1000000.0f / ((float)taskInfo.latestDeltaTime));
                cliPrintf("%02d - (%15s) ", taskId, taskInfo.taskName);
            }
            const int maxLoad = taskInfo.maxExecutionTime == 0 ? 0 :(taskInfo.maxExecutionTime * taskFrequency) / 1000;
            const int averageLoad = taskInfo.averageExecutionTime == 0 ? 0 : (taskInfo.averageExecutionTime * taskFrequency) / 1000;
            if (taskId != TASK_SERIAL) {
                maxLoadSum += maxLoad;
                averageLoadSum += averageLoad;
            }
            if (systemConfig()->task_statistics) {
                cliPrintLinef("%6d %7d %7d %4d.%1d%% %4d.%1d%% %9d",
                        taskFrequency, taskInfo.maxExecutionTime, taskInfo.averageExecutionTime,
                        maxLoad/10, maxLoad%10, averageLoad/10, averageLoad%10, taskInfo.totalExecutionTime / 1000);
            } else {
                cliPrintLinef("%6d", taskFrequency);
            }
            if (taskId == TASK_GYROPID && pidConfig()->pid_process_denom > 1) {
                cliPrintLinef("   - (%15s) %6d", taskInfo.subTaskName, subTaskFrequency);
            }

            schedulerResetTaskMaxExecutionTime(taskId);
        }
    }
    if (systemConfig()->task_statistics) {
        cfCheckFuncInfo_t checkFuncInfo;
        getCheckFuncInfo(&checkFuncInfo);
        cliPrintLinef("RX Check Function %19d %7d %25d", checkFuncInfo.maxExecutionTime, checkFuncInfo.averageExecutionTime, checkFuncInfo.totalExecutionTime / 1000);
        cliPrintLinef("Total (excluding SERIAL) %25d.%1d%% %4d.%1d%%", maxLoadSum/10, maxLoadSum%10, averageLoadSum/10, averageLoadSum%10);
    }
}
#endif

static void cliVersion(char *cmdline)
{
    UNUSED(cmdline);

    cliPrintLinef("# %s / %s (%s) %s %s / %s (%s) MSP API: %s",
        FC_FIRMWARE_NAME,
        targetName,
        systemConfig()->boardIdentifier,
        FC_VERSION_STRING,
        buildDate,
        buildTime,
        shortGitRevision,
        MSP_API_VERSION_STRING
    );
#ifdef USE_GYRO_IMUF9001
    cliPrintLinef("# IMU-F Version: %lu", imufCurrentVersion);
#endif
}

#ifdef USE_RC_SMOOTHING_FILTER
static void cliRcSmoothing(char *cmdline)
{
    UNUSED(cmdline);
    cliPrint("# RC Smoothing Type: ");
    if (rxConfig()->rc_smoothing_type == RC_SMOOTHING_TYPE_FILTER) {
        cliPrintLine("FILTER");
        uint16_t avgRxFrameMs = rcSmoothingGetValue(RC_SMOOTHING_VALUE_AVERAGE_FRAME);
        if (rcSmoothingAutoCalculate()) {
            cliPrint("# Detected RX frame rate: ");
            if (avgRxFrameMs == 0) {
                cliPrintLine("NO SIGNAL");
            } else {
                cliPrintLinef("%d.%dms", avgRxFrameMs / 1000, avgRxFrameMs % 1000);
            }
        }
        cliPrint("# Input filter type: ");
        cliPrintLinef(lookupTables[TABLE_RC_SMOOTHING_INPUT_TYPE].values[rxConfig()->rc_smoothing_input_type]);
        cliPrintf("# Active input cutoff: %dhz ", rcSmoothingGetValue(RC_SMOOTHING_VALUE_INPUT_ACTIVE));
        if (rxConfig()->rc_smoothing_input_cutoff == 0) {
            cliPrintLine("(auto)");
        } else {
            cliPrintLine("(manual)");
        }
        cliPrint("# Derivative filter type: ");
        cliPrintLinef(lookupTables[TABLE_RC_SMOOTHING_DERIVATIVE_TYPE].values[rxConfig()->rc_smoothing_derivative_type]);
        cliPrintf("# Active derivative cutoff: %dhz (", rcSmoothingGetValue(RC_SMOOTHING_VALUE_DERIVATIVE_ACTIVE));
        if (rxConfig()->rc_smoothing_derivative_type == RC_SMOOTHING_DERIVATIVE_OFF) {
            cliPrintLine("off)");
        } else {
            if (rxConfig()->rc_smoothing_derivative_cutoff == 0) {
                cliPrintLine("auto)");
            } else {
                cliPrintLine("manual)");
            }
        }
    } else {
        cliPrintLine("INTERPOLATION");
    }
}
#endif // USE_RC_SMOOTHING_FILTER

#if defined(USE_RESOURCE_MGMT)

#define MAX_RESOURCE_INDEX(x) ((x) == 0 ? 1 : (x))

typedef struct {
    const uint8_t owner;
    pgn_t pgn;
    uint8_t stride;
    uint8_t offset;
    const uint8_t maxIndex;
} cliResourceValue_t;

// Handy macros for keeping the table tidy.
// DEFS : Single entry
// DEFA : Array of uint8_t (stride = 1)
// DEFW : Wider stride case; array of structs.

#define DEFS(owner, pgn, type, member) \
    { owner, pgn, 0, offsetof(type, member), 0 }

#define DEFA(owner, pgn, type, member, max) \
    { owner, pgn, sizeof(ioTag_t), offsetof(type, member), max }

#define DEFW(owner, pgn, type, member, max) \
    { owner, pgn, sizeof(type), offsetof(type, member), max }

const cliResourceValue_t resourceTable[] = {
#ifdef USE_BEEPER
    DEFS( OWNER_BEEPER,        PG_BEEPER_DEV_CONFIG, beeperDevConfig_t, ioTag) ,
#endif
    DEFA( OWNER_MOTOR,         PG_MOTOR_CONFIG, motorConfig_t, dev.ioTags[0], MAX_SUPPORTED_MOTORS ),
#ifdef USE_SERVOS
    DEFA( OWNER_SERVO,         PG_SERVO_CONFIG, servoConfig_t, dev.ioTags[0], MAX_SUPPORTED_SERVOS ),
#endif
#if defined(USE_PPM)
    DEFS( OWNER_PPMINPUT,      PG_PPM_CONFIG, ppmConfig_t, ioTag ),
#endif
#if defined(USE_PWM)
    DEFA( OWNER_PWMINPUT,      PG_PWM_CONFIG, pwmConfig_t, ioTags[0], PWM_INPUT_PORT_COUNT ),
#endif
#ifdef USE_RANGEFINDER_HCSR04
    DEFS( OWNER_SONAR_TRIGGER, PG_SONAR_CONFIG, sonarConfig_t, triggerTag ),
    DEFS( OWNER_SONAR_ECHO,    PG_SONAR_CONFIG, sonarConfig_t, echoTag ),
#endif
#ifdef USE_LED_STRIP
    DEFS( OWNER_LED_STRIP,     PG_LED_STRIP_CONFIG, ledStripConfig_t, ioTag ),
#endif
    DEFA( OWNER_SERIAL_TX,     PG_SERIAL_PIN_CONFIG, serialPinConfig_t, ioTagTx[0], SERIAL_PORT_MAX_INDEX ),
    DEFA( OWNER_SERIAL_RX,     PG_SERIAL_PIN_CONFIG, serialPinConfig_t, ioTagRx[0], SERIAL_PORT_MAX_INDEX ),
#ifdef USE_INVERTER
    DEFA( OWNER_INVERTER,      PG_SERIAL_PIN_CONFIG, serialPinConfig_t, ioTagInverter[0], SERIAL_PORT_MAX_INDEX ),
#endif
#ifdef USE_I2C
    DEFW( OWNER_I2C_SCL,       PG_I2C_CONFIG, i2cConfig_t, ioTagScl, I2CDEV_COUNT ),
    DEFW( OWNER_I2C_SDA,       PG_I2C_CONFIG, i2cConfig_t, ioTagSda, I2CDEV_COUNT ),
#endif
    DEFA( OWNER_LED,           PG_STATUS_LED_CONFIG, statusLedConfig_t, ioTags[0], STATUS_LED_NUMBER ),
#ifdef USE_SPEKTRUM_BIND
    DEFS( OWNER_RX_BIND,       PG_RX_CONFIG, rxConfig_t, spektrum_bind_pin_override_ioTag ),
    DEFS( OWNER_RX_BIND_PLUG,  PG_RX_CONFIG, rxConfig_t, spektrum_bind_plug_ioTag ),
#endif
#ifdef USE_TRANSPONDER
    DEFS( OWNER_TRANSPONDER,   PG_TRANSPONDER_CONFIG, transponderConfig_t, ioTag ),
#endif
#ifdef USE_SPI
    DEFW( OWNER_SPI_SCK,       PG_SPI_PIN_CONFIG, spiPinConfig_t, ioTagSck, SPIDEV_COUNT ),
    DEFW( OWNER_SPI_MISO,      PG_SPI_PIN_CONFIG, spiPinConfig_t, ioTagMiso, SPIDEV_COUNT ),
    DEFW( OWNER_SPI_MOSI,      PG_SPI_PIN_CONFIG, spiPinConfig_t, ioTagMosi, SPIDEV_COUNT ),
#endif
#ifdef USE_ESCSERIAL
    DEFS( OWNER_ESCSERIAL,     PG_ESCSERIAL_CONFIG, escSerialConfig_t, ioTag ),
#endif
#ifdef USE_CAMERA_CONTROL
    DEFS( OWNER_CAMERA_CONTROL, PG_CAMERA_CONTROL_CONFIG, cameraControlConfig_t, ioTag ),
#endif
#ifdef USE_ADC
    DEFS( OWNER_ADC_BATT,      PG_ADC_CONFIG, adcConfig_t, vbat.ioTag ),
    DEFS( OWNER_ADC_RSSI,      PG_ADC_CONFIG, adcConfig_t, rssi.ioTag ),
    DEFS( OWNER_ADC_CURR,      PG_ADC_CONFIG, adcConfig_t, current.ioTag ),
    DEFS( OWNER_ADC_EXT,       PG_ADC_CONFIG, adcConfig_t, external1.ioTag ),
#endif
#ifdef USE_BARO
    DEFS( OWNER_BARO_CS,       PG_BAROMETER_CONFIG, barometerConfig_t, baro_spi_csn ),
#endif
#ifdef USE_MAG
    DEFS( OWNER_COMPASS_CS,    PG_COMPASS_CONFIG, compassConfig_t, mag_spi_csn ),
#ifdef USE_MAG_DATA_READY_SIGNAL
    DEFS( OWNER_COMPASS_EXTI,  PG_COMPASS_CONFIG, compassConfig_t, interruptTag ),
#endif
#endif
#ifdef USE_SDCARD
    DEFS( OWNER_SDCARD_CS,     PG_SDCARD_CONFIG, sdcardConfig_t, chipSelectTag ),
    DEFS( OWNER_SDCARD_DETECT, PG_SDCARD_CONFIG, sdcardConfig_t, cardDetectTag ),
#endif
#ifdef USE_PINIO
    DEFA( OWNER_PINIO,         PG_PINIO_CONFIG, pinioConfig_t, ioTag, PINIO_COUNT ),
#endif
#if defined(USE_USB_MSC)
    DEFS( OWNER_USB_MSC_PIN,   PG_USB_CONFIG, usbDev_t, mscButtonPin ),
#endif
#ifdef USE_FLASH
    DEFS( OWNER_FLASH_CS,      PG_FLASH_CONFIG, flashConfig_t, csTag ),
#endif
#ifdef USE_MAX7456
    DEFS( OWNER_OSD_CS,        PG_MAX7456_CONFIG, max7456Config_t, csTag ),
#endif
#ifdef USE_SPI
    DEFA( OWNER_SPI_PREINIT_IPU, PG_SPI_PREINIT_IPU_CONFIG, spiCs_t, csnTag, SPI_PREINIT_IPU_COUNT ),
    DEFA( OWNER_SPI_PREINIT_OPU, PG_SPI_PREINIT_OPU_CONFIG, spiCs_t, csnTag, SPI_PREINIT_OPU_COUNT ),
#endif
#ifdef USE_RX_SPI
    DEFS( OWNER_RX_SPI_CS,     PG_RX_SPI_CONFIG, rxSpiConfig_t, csnTag ),
#endif
};

#undef DEFS
#undef DEFA
#undef DEFW

static ioTag_t *getIoTag(const cliResourceValue_t value, uint8_t index)
{
    const pgRegistry_t* rec = pgFind(value.pgn);
    return CONST_CAST(ioTag_t *, rec->address + value.stride * index + value.offset);
}

static void printResource(uint8_t dumpMask)
{
    for (unsigned int i = 0; i < ARRAYLEN(resourceTable); i++) {
        const char* owner = ownerNames[resourceTable[i].owner];
        const pgRegistry_t* pg = pgFind(resourceTable[i].pgn);
        const void *currentConfig;
        const void *defaultConfig;
        if (configIsInCopy) {
            currentConfig = pg->copy;
            defaultConfig = pg->address;
        } else {
            currentConfig = pg->address;
            defaultConfig = NULL;
        }

        for (int index = 0; index < MAX_RESOURCE_INDEX(resourceTable[i].maxIndex); index++) {
            const ioTag_t ioTag = *(ioTag_t *)((const uint8_t *)currentConfig + resourceTable[i].stride * index + resourceTable[i].offset);
            ioTag_t ioTagDefault = NULL;
            if (defaultConfig) {
                ioTagDefault = *(ioTag_t *)((const uint8_t *)defaultConfig + resourceTable[i].stride * index + resourceTable[i].offset);
            }

            const bool equalsDefault = ioTag == ioTagDefault;
            const char *format = "resource %s %d %c%02d";
            const char *formatUnassigned = "resource %s %d NONE";
            if (ioTagDefault) {
                cliDefaultPrintLinef(dumpMask, equalsDefault, format, owner, RESOURCE_INDEX(index), IO_GPIOPortIdxByTag(ioTagDefault) + 'A', IO_GPIOPinIdxByTag(ioTagDefault));
            } else if (defaultConfig) {
                cliDefaultPrintLinef(dumpMask, equalsDefault, formatUnassigned, owner, RESOURCE_INDEX(index));
            }
            if (ioTag) {
                cliDumpPrintLinef(dumpMask, equalsDefault, format, owner, RESOURCE_INDEX(index), IO_GPIOPortIdxByTag(ioTag) + 'A', IO_GPIOPinIdxByTag(ioTag));
            } else if (!(dumpMask & HIDE_UNUSED)) {
                cliDumpPrintLinef(dumpMask, equalsDefault, formatUnassigned, owner, RESOURCE_INDEX(index));
            }
        }
    }
}

static void printResourceOwner(uint8_t owner, uint8_t index)
{
    cliPrintf("%s", ownerNames[resourceTable[owner].owner]);

    if (resourceTable[owner].maxIndex > 0) {
        cliPrintf(" %d", RESOURCE_INDEX(index));
    }
}

static void resourceCheck(uint8_t resourceIndex, uint8_t index, ioTag_t newTag)
{
    if (!newTag) {
        return;
    }

    const char * format = "\r\nNOTE: %c%02d already assigned to ";
    for (int r = 0; r < (int)ARRAYLEN(resourceTable); r++) {
        for (int i = 0; i < MAX_RESOURCE_INDEX(resourceTable[r].maxIndex); i++) {
            ioTag_t *tag = getIoTag(resourceTable[r], i);
            if (*tag == newTag) {
                bool cleared = false;
                if (r == resourceIndex) {
                    if (i == index) {
                        continue;
                    }
                    *tag = IO_TAG_NONE;
                    cleared = true;
                }

                cliPrintf(format, DEFIO_TAG_GPIOID(newTag) + 'A', DEFIO_TAG_PIN(newTag));

                printResourceOwner(r, i);

                if (cleared) {
                    cliPrintf(". ");
                    printResourceOwner(r, i);
                    cliPrintf(" disabled");
                }

                cliPrintLine(".");
            }
        }
    }
}

static bool strToPin(char *pch, ioTag_t *tag)
{
    if (strcasecmp(pch, "NONE") == 0) {
        *tag = IO_TAG_NONE;
        return true;
    } else {
        unsigned pin = 0;
        unsigned port = (*pch >= 'a') ? *pch - 'a' : *pch - 'A';

        if (port < 8) {
            pch++;
            pin = atoi(pch);
            if (pin < 16) {
                *tag = DEFIO_TAG_MAKE(port, pin);
                return true;
            }
        }
    }
    return false;
}

static void cliResource(char *cmdline)
{
    int len = strlen(cmdline);

    if (len == 0) {
        printResource(DUMP_MASTER | HIDE_UNUSED);

        return;
    } else if (strncasecmp(cmdline, "list", len) == 0) {
#ifdef MINIMAL_CLI
        cliPrintLine("IO");
#else
        cliPrintLine("Currently active IO resource assignments:\r\n(reboot to update)");
        cliRepeat('-', 20);
#endif
        for (int i = 0; i < DEFIO_IO_USED_COUNT; i++) {
            const char* owner;
            owner = ownerNames[ioRecs[i].owner];

            cliPrintf("%c%02d: %s", IO_GPIOPortIdx(ioRecs + i) + 'A', IO_GPIOPinIdx(ioRecs + i), owner);
            if (ioRecs[i].index > 0) {
                cliPrintf(" %d", ioRecs[i].index);
            }
            cliPrintLinefeed();
        }

#ifndef MINIMAL_CLI
        cliPrintLine("\r\nUse: 'resource' to see how to change resources.");
#endif

        return;
    }

    uint8_t resourceIndex = 0;
    int index = 0;
    char *pch = NULL;
    char *saveptr;

    pch = strtok_r(cmdline, " ", &saveptr);
    for (resourceIndex = 0; ; resourceIndex++) {
        if (resourceIndex >= ARRAYLEN(resourceTable)) {
            cliPrintErrorLinef("Invalid");
            return;
        }

        if (strncasecmp(pch, ownerNames[resourceTable[resourceIndex].owner], len) == 0) {
            break;
        }
    }

    pch = strtok_r(NULL, " ", &saveptr);
    index = atoi(pch);

    if (resourceTable[resourceIndex].maxIndex > 0 || index > 0) {
        if (index <= 0 || index > MAX_RESOURCE_INDEX(resourceTable[resourceIndex].maxIndex)) {
            cliShowArgumentRangeError("index", 1, MAX_RESOURCE_INDEX(resourceTable[resourceIndex].maxIndex));
            return;
        }
        index -= 1;

        pch = strtok_r(NULL, " ", &saveptr);
    }

    ioTag_t *tag = getIoTag(resourceTable[resourceIndex], index);

    if (strlen(pch) > 0) {
        if (strToPin(pch, tag)) {
            if (*tag == IO_TAG_NONE) {
#ifdef MINIMAL_CLI
                cliPrintLine("Freed");
#else
                cliPrintLine("Resource is freed");
#endif
                return;
            } else {
                ioRec_t *rec = IO_Rec(IOGetByTag(*tag));
                if (rec) {
                    resourceCheck(resourceIndex, index, *tag);
#ifdef MINIMAL_CLI
                    cliPrintLinef(" %c%02d set", IO_GPIOPortIdx(rec) + 'A', IO_GPIOPinIdx(rec));
#else
                    cliPrintLinef("\r\nResource is set to %c%02d", IO_GPIOPortIdx(rec) + 'A', IO_GPIOPinIdx(rec));
#endif
                } else {
                    cliShowParseError();
                }
                return;
            }
        }
    }

    cliShowParseError();
}

static void printDma(void)
{
    cliPrintLinefeed();

#ifdef MINIMAL_CLI
    cliPrintLine("DMA:");
#else
    cliPrintLine("Currently active DMA:");
    cliRepeat('-', 20);
#endif
    for (int i = 1; i <= DMA_LAST_HANDLER; i++) {
        const char* owner;
        owner = ownerNames[dmaGetOwner(i)];

        cliPrintf(DMA_OUTPUT_STRING, DMA_DEVICE_NO(i), DMA_DEVICE_INDEX(i));
        uint8_t resourceIndex = dmaGetResourceIndex(i);
        if (resourceIndex > 0) {
            cliPrintLinef(" %s %d", owner, resourceIndex);
        } else {
            cliPrintLinef(" %s", owner);
        }
    }
}

static void cliDma(char* cmdLine)
{
    UNUSED(cmdLine);
    printDma();
}
#endif /* USE_RESOURCE_MGMT */

#ifdef USE_TIMER_MGMT

static void printTimer(uint8_t dumpMask)
{
    cliPrintLine("# examples: ");
    const char *format = "timer %c%02d %d";
    cliPrint("#");
    cliPrintLinef(format, 'A', 1, 1);

    cliPrint("#");
    cliPrintLinef(format, 'A', 1, 0);

    for (unsigned int i = 0; i < MAX_TIMER_PINMAP_COUNT; i++) {

        const ioTag_t ioTag = timerIOConfig(i)->ioTag;
        const uint8_t timerIndex = timerIOConfig(i)->index;

        if (!ioTag) {
            continue;
        }

        if (timerIndex != 0 && !(dumpMask & HIDE_UNUSED)) {
            cliDumpPrintLinef(dumpMask, false, format,
                IO_GPIOPortIdxByTag(ioTag) + 'A',
                IO_GPIOPinIdxByTag(ioTag),
                timerIndex
                );
        }
    }
}

static void cliTimer(char *cmdline)
{
    int len = strlen(cmdline);

    if (len == 0) {
        printTimer(DUMP_MASTER | HIDE_UNUSED);
        return;
    } else if (strncasecmp(cmdline, "list", len) == 0) {
        printTimer(DUMP_MASTER);
        return;
    }

    char *pch = NULL;
    char *saveptr;
    int timerIOIndex = -1;

    ioTag_t ioTag = 0;
    pch = strtok_r(cmdline, " ", &saveptr);
    if (!pch || !(strToPin(pch, &ioTag) && IOGetByTag(ioTag))) {
        goto error;
    }

    /* find existing entry, or go for next available */
    for (unsigned i = 0; i < MAX_TIMER_PINMAP_COUNT; i++) {
        if (timerIOConfig(i)->ioTag == ioTag) {
            timerIOIndex = i;
            break;
        }

        /* first available empty slot */
        if (timerIOIndex < 0 && timerIOConfig(i)->ioTag == IO_TAG_NONE) {
            timerIOIndex = i;
        }
    }

    if (timerIOIndex < 0) {
        cliPrintErrorLinef("Index out of range.");
        return;
    }

    uint8_t timerIndex = 0;
    pch = strtok_r(NULL, " ", &saveptr);
    if (pch) {
        if (strcasecmp(pch, "list") == 0) {
            /* output the list of available options */
            uint8_t index = 1;
            for (unsigned i = 0; i < USABLE_TIMER_CHANNEL_COUNT; i++) {
                if (timerHardware[i].tag == ioTag) {
                    cliPrintLinef("# %d. TIM%d CH%d",
                        index,
                        timerGetTIMNumber(timerHardware[i].tim),
                        CC_INDEX_FROM_CHANNEL(timerHardware[i].channel)
                    );
                    index++;
                }
            }
            return;
        } else if (strcasecmp(pch, "none") == 0) {
            goto success;
        } else {
            timerIndex = atoi(pch);
        }
    } else {
        goto error;
    }

success:
    timerIOConfigMutable(timerIOIndex)->ioTag = timerIndex == 0 ? IO_TAG_NONE : ioTag;
    timerIOConfigMutable(timerIOIndex)->index = timerIndex;

    cliPrintLine("Success");
    return;

error:
    cliShowParseError();
}
#endif

static void printConfig(char *cmdline, bool doDiff)
{
    uint8_t dumpMask = DUMP_MASTER;
    char *options;
    if ((options = checkCommand(cmdline, "master"))) {
        dumpMask = DUMP_MASTER; // only
    } else if ((options = checkCommand(cmdline, "profile"))) {
        dumpMask = DUMP_PROFILE; // only
    } else if ((options = checkCommand(cmdline, "rates"))) {
        dumpMask = DUMP_RATES; // only
    } else if ((options = checkCommand(cmdline, "all"))) {
        dumpMask = DUMP_ALL;   // all profiles and rates
    } else {
        options = cmdline;
    }

    if (doDiff) {
        dumpMask = dumpMask | DO_DIFF;
    }

    backupAndResetConfigs();
    if (checkCommand(options, "defaults")) {
        dumpMask = dumpMask | SHOW_DEFAULTS;   // add default values as comments for changed values
    }

    if ((dumpMask & DUMP_MASTER) || (dumpMask & DUMP_ALL)) {
        cliPrintHashLine("version");
        cliVersion(NULL);
        cliPrintLinefeed();

#if defined(USE_BOARD_INFO)
        cliBoardName("");
        cliManufacturerId("");
#endif

        if (dumpMask & DUMP_ALL) {
            cliMcuId(NULL);
#if defined(USE_BOARD_INFO) && defined(USE_SIGNATURE)
        cliSignature("");
#endif
        }

        if ((dumpMask & (DUMP_ALL | DO_DIFF)) == (DUMP_ALL | DO_DIFF)) {
            cliPrintHashLine("reset configuration to default settings");
            cliPrint("defaults nosave");
            cliPrintLinefeed();
        }

        cliPrintHashLine("name");
        printName(dumpMask, &pilotConfig_Copy);

#ifdef USE_RESOURCE_MGMT
        cliPrintHashLine("resources");
        printResource(dumpMask);
#endif

#ifndef USE_QUAD_MIXER_ONLY
        cliPrintHashLine("mixer");
        const bool equalsDefault = mixerConfig_Copy.mixerMode == mixerConfig()->mixerMode;
        const char *formatMixer = "mixer %s";
        cliDefaultPrintLinef(dumpMask, equalsDefault, formatMixer, mixerNames[mixerConfig()->mixerMode - 1]);
        cliDumpPrintLinef(dumpMask, equalsDefault, formatMixer, mixerNames[mixerConfig_Copy.mixerMode - 1]);

        cliDumpPrintLinef(dumpMask, customMotorMixer(0)->throttle == 0.0f, "\r\nmmix reset\r\n");

        printMotorMix(dumpMask, customMotorMixer_CopyArray, customMotorMixer(0));

#ifdef USE_SERVOS
        cliPrintHashLine("servo");
        printServo(dumpMask, servoParams_CopyArray, servoParams(0));

        cliPrintHashLine("servo mix");
        // print custom servo mixer if exists
        cliDumpPrintLinef(dumpMask, customServoMixers(0)->rate == 0, "smix reset\r\n");
        printServoMix(dumpMask, customServoMixers_CopyArray, customServoMixers(0));
#endif
#endif

        cliPrintHashLine("feature");
        printFeature(dumpMask, &featureConfig_Copy, featureConfig());

#if defined(USE_BEEPER)
        cliPrintHashLine("beeper");
        printBeeper(dumpMask, beeperConfig_Copy.beeper_off_flags, beeperConfig()->beeper_off_flags, "beeper", BEEPER_ALLOWED_MODES);

#if defined(USE_DSHOT)
        cliPrintHashLine("beacon");
        printBeeper(dumpMask, beeperConfig_Copy.dshotBeaconOffFlags, beeperConfig()->dshotBeaconOffFlags, "beacon", DSHOT_BEACON_ALLOWED_MODES);
#endif
#endif // USE_BEEPER

        cliPrintHashLine("map");
        printMap(dumpMask, &rxConfig_Copy, rxConfig());

        cliPrintHashLine("serial");
        printSerial(dumpMask, &serialConfig_Copy, serialConfig());

#ifdef USE_LED_STRIP
        cliPrintHashLine("led");
        printLed(dumpMask, ledStripConfig_Copy.ledConfigs, ledStripConfig()->ledConfigs);

        cliPrintHashLine("color");
        printColor(dumpMask, ledStripConfig_Copy.colors, ledStripConfig()->colors);

        cliPrintHashLine("mode_color");
        printModeColor(dumpMask, &ledStripConfig_Copy, ledStripConfig());
#endif

        cliPrintHashLine("aux");
        printAux(dumpMask, modeActivationConditions_CopyArray, modeActivationConditions(0));

        cliPrintHashLine("adjrange");
        printAdjustmentRange(dumpMask, adjustmentRanges_CopyArray, adjustmentRanges(0));

        cliPrintHashLine("rxrange");
        printRxRange(dumpMask, rxChannelRangeConfigs_CopyArray, rxChannelRangeConfigs(0));

#ifdef USE_VTX_CONTROL
        cliPrintHashLine("vtx");
        printVtx(dumpMask, &vtxConfig_Copy, vtxConfig());
#endif

        cliPrintHashLine("rxfail");
        printRxFailsafe(dumpMask, rxFailsafeChannelConfigs_CopyArray, rxFailsafeChannelConfigs(0));

        cliPrintHashLine("master");
        dumpAllValues(MASTER_VALUE, dumpMask);

        if (dumpMask & DUMP_ALL) {
            for (uint32_t pidProfileIndex = 0; pidProfileIndex < PID_PROFILE_COUNT; pidProfileIndex++) {
                cliDumpPidProfile(pidProfileIndex, dumpMask);
            }
            cliPrintHashLine("restore original profile selection");

            pidProfileIndexToUse = systemConfig_Copy.pidProfileIndex;

            cliProfile("");

            pidProfileIndexToUse = CURRENT_PROFILE_INDEX;

            for (uint32_t rateIndex = 0; rateIndex < CONTROL_RATE_PROFILE_COUNT; rateIndex++) {
                cliDumpRateProfile(rateIndex, dumpMask);
            }
            cliPrintHashLine("restore original rateprofile selection");

            rateProfileIndexToUse = systemConfig_Copy.activeRateProfile;

            cliRateProfile("");

            rateProfileIndexToUse = CURRENT_PROFILE_INDEX;

            cliPrintHashLine("save configuration");
            cliPrint("save");
        } else {
            cliDumpPidProfile(systemConfig_Copy.pidProfileIndex, dumpMask);

            cliDumpRateProfile(systemConfig_Copy.activeRateProfile, dumpMask);
        }
    }

    if (dumpMask & DUMP_PROFILE) {
        cliDumpPidProfile(systemConfig_Copy.pidProfileIndex, dumpMask);
    }

    if (dumpMask & DUMP_RATES) {
        cliDumpRateProfile(systemConfig_Copy.activeRateProfile, dumpMask);
    }
    // restore configs from copies
    restoreConfigs();
}

static void cliDump(char *cmdline)
{
    printConfig(cmdline, false);
}

static void cliDiff(char *cmdline)
{
    printConfig(cmdline, true);
}

#if defined(USE_USB_MSC)
static void cliMsc(char *cmdline)
{
    UNUSED(cmdline);

    if (mscCheckFilesystemReady()) {
        cliPrintHashLine("Restarting in mass storage mode");
        cliPrint("\r\nRebooting");
        bufWriterFlush(cliWriter);
        waitForSerialPortToFinishTransmitting(cliPort);
        stopPwmAllMotors();

        systemResetToMsc();
    } else {
        cliPrintHashLine("Storage not present or failed to initialize!");
    }
}
#endif

typedef struct {
    const char *name;
#ifndef MINIMAL_CLI
    const char *description;
    const char *args;
#endif
    void (*func)(char *cmdline);
} clicmd_t;

#ifndef MINIMAL_CLI
#define CLI_COMMAND_DEF(name, description, args, method) \
{ \
    name , \
    description , \
    args , \
    method \
}
#else
#define CLI_COMMAND_DEF(name, description, args, method) \
{ \
    name, \
    method \
}
#endif

#ifdef USE_GYRO_IMUF9001
static void cliReportImufErrors(char *cmdline);
#endif

static void cliHelp(char *cmdline);

// should be sorted a..z for bsearch()
const clicmd_t cmdTable[] = {
    CLI_COMMAND_DEF("adjrange", "configure adjustment ranges", NULL, cliAdjustmentRange),
    CLI_COMMAND_DEF("aux", "configure modes", "<index> <mode> <aux> <start> <end> <logic>", cliAux),
#if defined(USE_BEEPER)
#if defined(USE_DSHOT)
    CLI_COMMAND_DEF("beacon", "enable/disable Dshot beacon for a condition", "list\r\n"
        "\t<->[name]", cliBeacon),
#endif
    CLI_COMMAND_DEF("beeper", "enable/disable beeper for a condition", "list\r\n"
        "\t<->[name]", cliBeeper),
#endif // USE_BEEPER
    CLI_COMMAND_DEF("bl", "reboot into bootloader", NULL, cliBootloader),
#if defined(USE_BOARD_INFO)
    CLI_COMMAND_DEF("board_name", "get / set the name of the board model", "[board name]", cliBoardName),
#endif
#ifdef USE_LED_STRIP
    CLI_COMMAND_DEF("color", "configure colors", NULL, cliColor),
#endif
    CLI_COMMAND_DEF("defaults", "reset to defaults and reboot", "[nosave]", cliDefaults),
    CLI_COMMAND_DEF("diff", "list configuration changes from default", "[master|profile|rates|all] {defaults}", cliDiff),
#ifdef USE_RESOURCE_MGMT
    CLI_COMMAND_DEF("dma", "list dma utilisation", NULL, cliDma),
#endif
#ifdef USE_DSHOT
    CLI_COMMAND_DEF("dshotprog", "program DShot ESC(s)", "<index> <command>+", cliDshotProg),
#endif
    CLI_COMMAND_DEF("dump", "dump configuration",
        "[master|profile|rates|all] {defaults}", cliDump),
#ifdef USE_ESCSERIAL
    CLI_COMMAND_DEF("escprog", "passthrough esc to serial", "<mode [sk/bl/ki/cc]> <index>", cliEscPassthrough),
#endif
#ifdef USE_GYRO_IMUF9001
    CLI_COMMAND_DEF("imufbootloader", NULL, NULL, cliImufBootloaderMode),
    CLI_COMMAND_DEF("imufloadbin", NULL, NULL, cliImufLoadBin),
    CLI_COMMAND_DEF("imufflashbin", NULL, NULL, cliImufFlashBin),
#endif
#ifdef MSP_OVER_CLI
    CLI_COMMAND_DEF("msp", NULL, NULL, cliMsp),
#endif
    CLI_COMMAND_DEF("exit", NULL, NULL, cliExit),
    CLI_COMMAND_DEF("feature", "configure features",
        "list\r\n"
        "\t<+|->[name]", cliFeature),
#ifdef USE_FLASHFS
    CLI_COMMAND_DEF("flash_erase", "erase flash chip", NULL, cliFlashErase),
    CLI_COMMAND_DEF("flash_info", "show flash chip info", NULL, cliFlashInfo),
#ifdef USE_FLASH_TOOLS
    CLI_COMMAND_DEF("flash_read", NULL, "<length> <address>", cliFlashRead),
    CLI_COMMAND_DEF("flash_write", NULL, "<address> <message>", cliFlashWrite),
#endif
#endif
#ifdef USE_RX_CC2500_BIND
    CLI_COMMAND_DEF("bind", "initiate binding for RX", NULL, cliRxBind),
#endif
    CLI_COMMAND_DEF("get", "get variable value", "[name]", cliGet),

#ifdef USE_PEGASUS_UI
    CLI_COMMAND_DEF("config", "get all configuration information", NULL, cliConfig),
#endif
#ifdef USE_GPS
    CLI_COMMAND_DEF("gpspassthrough", "passthrough gps to serial", NULL, cliGpsPassthrough),
#endif
#if defined(USE_GYRO_REGISTER_DUMP) && !defined(SIMULATOR_BUILD)
    CLI_COMMAND_DEF("gyroregisters", "dump gyro config registers contents", NULL, cliDumpGyroRegisters),
#endif
#ifdef USE_GYRO_IMUF9001
    CLI_COMMAND_DEF("reportimuferrors", "report imu-f comm errors", NULL, cliReportImufErrors),
#endif

    CLI_COMMAND_DEF("help", NULL, NULL, cliHelp),
#ifdef USE_LED_STRIP
    CLI_COMMAND_DEF("led", "configure leds", NULL, cliLed),
#endif
#if defined(USE_BOARD_INFO)
    CLI_COMMAND_DEF("manufacturer_id", "get / set the id of the board manufacturer", "[manufacturer id]", cliManufacturerId),
#endif
    CLI_COMMAND_DEF("map", "configure rc channel order", "[<map>]", cliMap),
    CLI_COMMAND_DEF("mcu_id", "id of the microcontroller", NULL, cliMcuId),
#ifndef USE_QUAD_MIXER_ONLY
    CLI_COMMAND_DEF("mixer", "configure mixer", "list\r\n\t<name>", cliMixer),
#endif
    CLI_COMMAND_DEF("mmix", "custom motor mixer", NULL, cliMotorMix),
#ifdef USE_LED_STRIP
    CLI_COMMAND_DEF("mode_color", "configure mode and special colors", NULL, cliModeColor),
#endif
    CLI_COMMAND_DEF("motor",  "get/set motor", "<index> [<value>]", cliMotor),
#ifdef USE_USB_MSC
    CLI_COMMAND_DEF("msc", "switch into msc mode", NULL, cliMsc),
#endif
    CLI_COMMAND_DEF("name", "name of craft", NULL, cliName),
#ifndef MINIMAL_CLI
    CLI_COMMAND_DEF("play_sound", NULL, "[<index>]", cliPlaySound),
#endif
    CLI_COMMAND_DEF("profile", "change profile", "[<index>]", cliProfile),
    CLI_COMMAND_DEF("rateprofile", "change rate profile", "[<index>]", cliRateProfile),
#ifdef USE_RC_SMOOTHING_FILTER
    CLI_COMMAND_DEF("rc_smoothing_info", "show rc_smoothing operational settings", NULL, cliRcSmoothing),
#endif // USE_RC_SMOOTHING_FILTER
#ifdef USE_RESOURCE_MGMT
    CLI_COMMAND_DEF("resource", "show/set resources", NULL, cliResource),
#endif
    CLI_COMMAND_DEF("rxfail", "show/set rx failsafe settings", NULL, cliRxFailsafe),
    CLI_COMMAND_DEF("rxrange", "configure rx channel ranges", NULL, cliRxRange),
    CLI_COMMAND_DEF("save", "save and reboot", NULL, cliSave),
#ifdef USE_SDCARD
    CLI_COMMAND_DEF("sd_info", "sdcard info", NULL, cliSdInfo),
#endif
    CLI_COMMAND_DEF("serial", "configure serial ports", NULL, cliSerial),
#ifndef SKIP_SERIAL_PASSTHROUGH
    CLI_COMMAND_DEF("serialpassthrough", "passthrough serial data to port", "<id> [baud] [mode] [DTR PINIO]: passthrough to serial", cliSerialPassthrough),
#endif
#ifdef USE_SERVOS
    CLI_COMMAND_DEF("servo", "configure servos", NULL, cliServo),
#endif
    CLI_COMMAND_DEF("set", "change setting", "[<name>=<value>]", cliSet),
#if defined(USE_BOARD_INFO) && defined(USE_SIGNATURE)
    CLI_COMMAND_DEF("signature", "get / set the board type signature", "[signature]", cliSignature),
#endif
#ifdef USE_SERVOS
    CLI_COMMAND_DEF("smix", "servo mixer", "<rule> <servo> <source> <rate> <speed> <min> <max> <box>\r\n"
        "\treset\r\n"
        "\tload <mixer>\r\n"
        "\treverse <servo> <source> r|n", cliServoMix),
#endif
    CLI_COMMAND_DEF("status", "show status", NULL, cliStatus),
#ifndef SKIP_TASK_STATISTICS
    CLI_COMMAND_DEF("tasks", "show task stats", NULL, cliTasks),
#endif
#ifdef USE_TIMER_MGMT
    CLI_COMMAND_DEF("timer", "show timer configuration", NULL, cliTimer),
#endif
    CLI_COMMAND_DEF("version", "show version", NULL, cliVersion),
#ifdef USE_VTX_CONTROL
    CLI_COMMAND_DEF("vtx", "vtx channels on switch", NULL, cliVtx),
#endif
};

#ifdef USE_GYRO_IMUF9001
static void cliReportImufErrors(char *cmdline)
{
    UNUSED(cmdline);
    cliPrintf("Current Comm Errors: %lu", crcErrorCount);
    cliPrintLinefeed();
}
#endif

static void cliHelp(char *cmdline)
{
    UNUSED(cmdline);

    for (uint32_t i = 0; i < ARRAYLEN(cmdTable); i++) {
        cliPrint(cmdTable[i].name);
#ifndef MINIMAL_CLI
        if (cmdTable[i].description) {
            cliPrintf(" - %s", cmdTable[i].description);
        }
        if (cmdTable[i].args) {
            cliPrintf("\r\n\t%s", cmdTable[i].args);
        }
#endif
        cliPrintLinefeed();
    }
}

void cliProcess(void)
{
    if (!cliWriter) {
        return;
    }

    // Be a little bit tricky.  Flush the last inputs buffer, if any.
    bufWriterFlush(cliWriter);

    while (serialRxBytesWaiting(cliPort)) {

        uint8_t c = serialRead(cliPort);

        if (c == '\t' || c == '?') {
            // do tab completion
            const clicmd_t *cmd, *pstart = NULL, *pend = NULL;
            uint32_t i = bufferIndex;
            for (cmd = cmdTable; cmd < cmdTable + ARRAYLEN(cmdTable); cmd++) {
                if (bufferIndex && (strncasecmp(cliBuffer, cmd->name, bufferIndex) != 0))
                    continue;
                if (!pstart)
                    pstart = cmd;
                pend = cmd;
            }
            if (pstart) {    /* Buffer matches one or more commands */
                for (; ; bufferIndex++) {
                    if (pstart->name[bufferIndex] != pend->name[bufferIndex])
                        break;
                    if (!pstart->name[bufferIndex] && bufferIndex < sizeof(cliBuffer) - 2) {
                        /* Unambiguous -- append a space */
                        cliBuffer[bufferIndex++] = ' ';
                        cliBuffer[bufferIndex] = '\0';
                        break;
                    }
                    cliBuffer[bufferIndex] = pstart->name[bufferIndex];
                }
            }
            if (!bufferIndex || pstart != pend) {
                /* Print list of ambiguous matches */
                cliPrint("\r\033[K");
                for (cmd = pstart; cmd <= pend; cmd++) {
                    cliPrint(cmd->name);
                    cliWrite('\t');
                }
                cliPrompt();
                i = 0;    /* Redraw prompt */
            }
            for (; i < bufferIndex; i++)
                cliWrite(cliBuffer[i]);
        } else if (!bufferIndex && c == 4) {   // CTRL-D
            cliExit(cliBuffer);
            return;
        } else if (c == 12) {                  // NewPage / CTRL-L
            // clear screen
            cliPrint("\033[2J\033[1;1H");
            cliPrompt();
        } else if (bufferIndex && (c == '\n' || c == '\r')) {
            // enter pressed
            cliPrintLinefeed();

            // Strip comment starting with # from line
            char *p = cliBuffer;
            p = strchr(p, '#');
            if (NULL != p) {
                bufferIndex = (uint32_t)(p - cliBuffer);
            }

            // Strip trailing whitespace
            while (bufferIndex > 0 && cliBuffer[bufferIndex - 1] == ' ') {
                bufferIndex--;
            }

            // Process non-empty lines
            if (bufferIndex > 0) {
                cliBuffer[bufferIndex] = 0; // null terminate

                const clicmd_t *cmd;
                char *options;
                for (cmd = cmdTable; cmd < cmdTable + ARRAYLEN(cmdTable); cmd++) {
                    if ((options = checkCommand(cliBuffer, cmd->name))) {
                        break;
                    }
                }
                if (cmd < cmdTable + ARRAYLEN(cmdTable))
                    cmd->func(options);
                else
                    cliPrint("Unknown command, try 'help'");
                bufferIndex = 0;
            }

            memset(cliBuffer, 0, sizeof(cliBuffer));

            // 'exit' will reset this flag, so we don't need to print prompt again
            if (!cliMode)
                return;

            cliPrompt();
        } else if (c == 127) {
            // backspace
            if (bufferIndex) {
                cliBuffer[--bufferIndex] = 0;
                cliPrint("\010 \010");
            }
        } else if (bufferIndex < sizeof(cliBuffer) && c >= 32 && c <= 126) {
            if (!bufferIndex && c == ' ')
                continue; // Ignore leading spaces
            cliBuffer[bufferIndex++] = c;
            cliWrite(c);
        }

    }
}

void cliEnter(serialPort_t *serialPort)
{
    cliMode = 1;
    cliPort = serialPort;
    setPrintfSerialPort(cliPort);
    cliWriter = bufWriterInit(cliWriteBuffer, sizeof(cliWriteBuffer), (bufWrite_t)serialWriteBufShim, serialPort);

    schedulerSetCalulateTaskStatistics(systemConfig()->task_statistics);

#ifndef MINIMAL_CLI
    cliPrintLine("\r\nEntering CLI Mode, type 'exit' to return, or 'help'");
#else
    cliPrintLine("\r\nCLI");
#endif
    cliPrompt();

    setArmingDisabled(ARMING_DISABLED_CLI);
}

void cliInit(const serialConfig_t *serialConfig)
{
    UNUSED(serialConfig);
}
#endif // USE_CLI
