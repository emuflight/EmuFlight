/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <limits.h>

extern "C" {
    #include <platform.h>

    #include "build/debug.h"

    #include "common/axis.h"
    #include "common/crc.h"
    #include "common/filter.h"
    #include "common/gps_conversion.h"
    #include "common/maths.h"
    #include "common/printf.h"
    #include "common/streambuf.h"
    #include "common/typeconversion.h"

    #include "pg/pg.h"
    #include "pg/pg_ids.h"
    #include "pg/rx.h"

    #include "drivers/serial.h"
    #include "drivers/system.h"

    #include "fc/config.h"
    #include "fc/runtime_config.h"

    #include "flight/pid.h"
    #include "flight/imu.h"

    #include "io/gps.h"
    #include "io/serial.h"

    #include "rx/rx.h"
    #include "rx/ghst.h"
    #include "rx/ghst_protocol.h"

    #include "sensors/battery.h"
    #include "sensors/sensors.h"
    #include "sensors/acceleration.h"

    #include "telemetry/ghst.h"
    #include "telemetry/telemetry.h"
    /* #include "telemetry/msp_shared.h" */


    uint8_t ghstScheduleCount;
    void ghstInitializeFrame(sbuf_t *dst);
    void processGhst(void);
    uint8_t *getGhstFrame(void);

    bool airMode;

    uint16_t testBatteryVoltage = 0;
    uint16_t testBatteryCellVoltage = 0;
    uint8_t testBatteryCellCount = 0;
    int32_t testAmperage = 0;
    int32_t testmAhDrawn = 0;
    uint32_t getEstimatedAltitude() { return 0; }

    serialPort_t *telemetrySharedPort;
    PG_REGISTER(batteryConfig_t, batteryConfig, PG_BATTERY_CONFIG, 0);
    PG_REGISTER(telemetryConfig_t, telemetryConfig, PG_TELEMETRY_CONFIG, 0);
    PG_REGISTER(systemConfig_t, systemConfig, PG_SYSTEM_CONFIG, 0);
    PG_REGISTER(rxConfig_t, rxConfig, PG_RX_CONFIG, 0);
    PG_REGISTER(accelerometerConfig_t, accelerometerConfig, PG_ACCELEROMETER_CONFIG, 0);
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

uint8_t crfsCrc(uint8_t *frame, int frameLen)
{
    uint8_t crc = 0;
    for (int ii = 2; ii < frameLen - 1; ++ii) {
        crc = crc8_dvb_s2(crc, frame[ii]);
    }
    return crc;
}

/*
int16_t     Voltage
int16_t     Current
uint16_t    mAh Drawn
*/
#define FRAME_HEADER_FOOTER_LEN 4


TEST(TelemetryGhstTest, TestBattery)
{
    uint8_t *frame = getGhstFrame();
    uint16_t voltage;
    uint16_t current;
    uint32_t usedMah;

    initGhstTelemetry();
    processGhst();

    testBatteryVoltage = 0; // 0.1V units

    EXPECT_EQ(GHST_ADDR_RX, frame[0]); // address
    EXPECT_EQ(12, frame[1]); // length
    EXPECT_EQ(0x23, frame[2]); // type
    voltage = frame[3] << 8 | frame[4]; // mV * 100
    EXPECT_EQ(0, voltage);
    current = frame[5] << 8 | frame[6]; // mA * 100
    EXPECT_EQ(0, current);
    usedMah = frame[7] << 16 | frame[8] << 8 | frame [9]; // mAh
    EXPECT_EQ(0, usedMah);

    testBatteryVoltage = 124; // 12.4V = 1240 mv
    testAmperage = 2960; // = 29.60A = 29600mA - amperage is in 0.01A steps
    testmAhDrawn = 1234;

    processGhst();

    voltage = frame[4] << 8 | frame[3]; // mV * 100
    EXPECT_EQ(testBatteryVoltage * 10, voltage);
    current = frame[6] << 8 | frame[5]; // mA * 100
    EXPECT_EQ(testAmperage, current);
    usedMah = frame[9] << 16 | frame[8] << 8 | frame [7]; // mAh
    EXPECT_EQ(testmAhDrawn/10, usedMah);

}


TEST(TelemetryGhstTest, TestBatteryCellVoltage)
{
    uint8_t *frame = getGhstFrame(); 
    /* memset(&frame, 0, sizeof(*frame)); */
    uint16_t voltage;
    uint16_t current;
    uint32_t usedMah;

    testBatteryVoltage = 124; // 12.4V = 1240 mv
    testBatteryCellVoltage = 413; // 12.4/3
    testBatteryCellCount = 3;
    testAmperage = 2960; // = 29.60A = 29600mA - amperage is in 0.01A steps
    testmAhDrawn = 1234;

    telemetryConfigMutable()->report_cell_voltage = true;

    processGhst();

    voltage = frame[4] << 8 | frame[3]; // mV * 100
    EXPECT_EQ(testBatteryCellVoltage, voltage);
    current = frame[6] << 8 | frame[5]; // mA * 100
    EXPECT_EQ(testAmperage, current);
    usedMah = frame[9] << 16 | frame[8] << 8 | frame [7]; // mAh
    EXPECT_EQ(testmAhDrawn/10, usedMah);
}

// STUBS

extern "C" {

int16_t debug[DEBUG16_VALUE_COUNT];

const uint32_t baudRates[] = {0, 9600, 19200, 38400, 57600, 115200, 230400, 250000, 400000}; // see baudRate_e

uint16_t batteryWarningVoltage;
uint8_t useHottAlarmSoundPeriod (void) { return 0; }

attitudeEulerAngles_t attitude = { { 0, 0, 0 } };     // absolute angle inclination in multiple of 0.1 degree    180 deg = 1800

uint16_t GPS_distanceToHome;        // distance to home point in meters
gpsSolutionData_t gpsSol;

void beeperConfirmationBeeps(uint8_t beepCount) {UNUSED(beepCount);}

uint32_t micros(void) {return 0;}

bool feature(uint32_t) {return true;}

uint32_t serialRxBytesWaiting(const serialPort_t *) {return 0;}
uint32_t serialTxBytesFree(const serialPort_t *) {return 0;}
uint8_t serialRead(serialPort_t *) {return 0;}
void serialWrite(serialPort_t *, uint8_t) {}
void serialWriteBuf(serialPort_t *, const uint8_t *, int) {}
void serialSetMode(serialPort_t *, portMode_e) {}
serialPort_t *openSerialPort(serialPortIdentifier_e, serialPortFunction_e, serialReceiveCallbackPtr, void *, uint32_t, portMode_e, portOptions_e) {return NULL;}
void closeSerialPort(serialPort_t *) {}
bool isSerialTransmitBufferEmpty(const serialPort_t *) { return true; }

serialPortConfig_t *findSerialPortConfig(serialPortFunction_e) {return NULL;}

bool telemetryDetermineEnabledState(portSharing_e) {return true;}
bool telemetryCheckRxPortShared(const serialPortConfig_t *) {return true;}

portSharing_e determinePortSharing(const serialPortConfig_t *, serialPortFunction_e) {return PORTSHARING_NOT_SHARED;}

bool isAirmodeActive(void) {return airMode;}

int32_t getAmperage(void) {
    return testAmperage;
}

uint16_t getBatteryVoltage(void) {
    return testBatteryVoltage;
}

uint16_t getBatteryAverageCellVoltage(void) {
    return testBatteryCellVoltage;
}

uint8_t getBatteryCellCount(void) {
    return testBatteryCellCount;
}

batteryState_e getBatteryState(void) {
    return BATTERY_OK;
}

uint8_t calculateBatteryPercentageRemaining(void) {
    return 67;
}

int32_t getMAhDrawn(void){
  return testmAhDrawn;
}

bool isBatteryVoltageConfigured(void) { return true; }
bool isAmperageConfigured(void) { return true; }

void setRssi(uint16_t, rssiSource_e){}
rssiSource_e rssiSource;


uint32_t microsISR(void) { return 0; };

bool checkGhstTelemetryState(void) {
    return true;
}

}
