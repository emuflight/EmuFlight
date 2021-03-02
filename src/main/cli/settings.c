/*
 * This file is part of Cleanflight and Betaflight and EmuFlight.
 *
 * Cleanflight and Betaflight and EmuFlight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight and EmuFlight are distributed in the hope that they
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

#include "build/debug.h"

#include "blackbox/blackbox.h"
#include "blackbox/blackbox_fielddefs.h"

#include "cms/cms.h"

#include "common/utils.h"
#include "common/time.h"

#include "drivers/adc.h"
#include "drivers/bus_i2c.h"
#include "drivers/bus_spi.h"
#include "drivers/dshot_command.h"
#include "drivers/camera_control.h"
#include "drivers/light_led.h"
#include "drivers/mco.h"
#include "drivers/pinio.h"
#include "drivers/sdio.h"
#include "drivers/vtx_common.h"
#include "drivers/vtx_table.h"

#include "config/config.h"
#include "fc/controlrate_profile.h"
#include "fc/core.h"
#include "fc/rc.h"
#include "fc/rc_adjustments.h"
#include "fc/rc_controls.h"

#include "flight/failsafe.h"
#include "flight/gps_rescue.h"
#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/position.h"
#include "flight/rpm_filter.h"
#include "flight/servos.h"

#include "io/beeper.h"
#include "io/dashboard.h"
#include "io/gimbal.h"
#include "io/gps.h"
#include "io/ledstrip.h"
#include "io/serial.h"
#include "io/vtx.h"
#include "io/vtx_control.h"
#include "io/vtx_rtc6705.h"

#include "osd/osd.h"

#include "pg/adc.h"
#include "pg/beeper.h"
#include "pg/beeper_dev.h"
#include "pg/bus_i2c.h"
#include "pg/dashboard.h"
#include "pg/displayport_profiles.h"
#include "pg/flash.h"
#include "pg/gyrodev.h"
#include "pg/max7456.h"
#include "pg/mco.h"
#include "pg/motor.h"
#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/pinio.h"
#include "pg/piniobox.h"
#include "pg/rx.h"
#include "pg/rx_pwm.h"
#include "pg/rx_spi.h"
#include "pg/rx_spi_cc2500.h"
#include "pg/sdcard.h"
#include "pg/vcd.h"
#include "pg/vtx_io.h"
#include "pg/usb.h"
#include "pg/sdio.h"
#include "pg/rcdevice.h"
#include "pg/stats.h"
#include "pg/board.h"

#include "rx/a7105_flysky.h"
#include "rx/cc2500_frsky_common.h"
#include "rx/cc2500_sfhss.h"
#include "rx/crsf.h"
#include "rx/cyrf6936_spektrum.h"
#include "rx/rx.h"
#include "rx/spektrum.h"

#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/battery.h"
#include "sensors/boardalignment.h"
#include "sensors/compass.h"
#include "sensors/esc_sensor.h"
#include "sensors/gyro.h"
#include "sensors/rangefinder.h"

#include "telemetry/frsky_hub.h"
#include "telemetry/ibus_shared.h"
#include "telemetry/telemetry.h"

#include "settings.h"


// Sensor names (used in lookup tables for *_hardware settings and in status command output)
// sync with accelerationSensor_e
const char * const lookupTableAccHardware[] = {
    "AUTO", "NONE", "ADXL345", "MPU6050", "MMA8452", "BMA280", "LSM303DLHC",
    "MPU6000", "MPU6500", "MPU9250", "ICM20601", "ICM20602", "ICM20608G", "ICM20649", "ICM20689", "ICM42605",
    "BMI160", "BMI270", "LSM6DSO", "FAKE"
};

// sync with gyroHardware_e
const char * const lookupTableGyroHardware[] = {
    "AUTO", "NONE", "MPU6050", "L3G4200D", "MPU3050", "L3GD20",
    "MPU6000", "MPU6500", "MPU9250", "ICM20601", "ICM20602", "ICM20608G", "ICM20649", "ICM20689", "ICM42605",
    "BMI160", "BMI270", "LSM6SDO", "FAKE"
};

#if defined(USE_SENSOR_NAMES) || defined(USE_BARO)
// sync with baroSensor_e
const char * const lookupTableBaroHardware[] = {
    "AUTO", "NONE", "BMP085", "MS5611", "BMP280", "LPS", "QMP6988", "BMP388", "DPS310"
};
#endif
#if defined(USE_SENSOR_NAMES) || defined(USE_MAG)
// sync with magSensor_e
const char * const lookupTableMagHardware[] = {
    "AUTO", "NONE", "HMC5883", "AK8975", "AK8963", "QMC5883", "LIS3MDL", "MAG_MPU925X_AK8963"
};
#endif
#if defined(USE_SENSOR_NAMES) || defined(USE_RANGEFINDER)
const char * const lookupTableRangefinderHardware[] = {
    "NONE", "HCSR04", "TFMINI", "TF02"
};
#endif

static const char * const lookupTableOffOn[] = {
    "OFF", "ON"
};

static const char * const lookupTableCrashRecovery[] = {
    "OFF", "DISARM"
};

static const char * const lookupTableUnit[] = {
    "IMPERIAL", "METRIC", "BRITISH"
};

static const char * const lookupTableAlignment[] = {
    "DEFAULT",
    "CW0",
    "CW90",
    "CW180",
    "CW270",
    "CW0FLIP",
    "CW90FLIP",
    "CW180FLIP",
    "CW270FLIP",
    "CUSTOM",
};

#ifdef USE_MULTI_GYRO
static const char * const lookupTableGyro[] = {
    "FIRST", "SECOND", "BOTH"
};
#endif

#ifdef USE_GPS
static const char * const lookupTableGPSProvider[] = {
    "NMEA", "UBLOX", "MSP"
};

static const char * const lookupTableGPSSBASMode[] = {
    "AUTO", "EGNOS", "WAAS", "MSAS", "GAGAN", "NONE"
};

static const char * const lookupTableGPSUBLOXMode[] = {
    "AIRBORNE", "PEDESTRIAN", "DYNAMIC"
};
#endif

#ifdef USE_SERVOS
static const char * const lookupTableGimbalMode[] = {
    "NORMAL", "MIXTILT"
};
#endif

#ifdef USE_BLACKBOX
static const char * const lookupTableBlackboxDevice[] = {
    "NONE", "SPIFLASH", "SDCARD", "SERIAL"
};

static const char * const lookupTableBlackboxMode[] = {
    "NORMAL", "MOTOR_TEST", "ALWAYS"
};

static const char * const lookupTableBlackboxSampleRate[] = {
    "1/1", "1/2", "1/4", "1/8", "1/16"
};
#endif

#ifdef USE_SERIAL_RX
static const char * const lookupTableSerialRX[] = {
    "SPEK1024",
    "SPEK2048",
    "SBUS",
    "SUMD",
    "SUMH",
    "XB-B",
    "XB-B-RJ01",
    "IBUS",
    "JETIEXBUS",
    "CRSF",
    "SRXL",
    "CUSTOM",
    "FPORT",
    "SRXL2",
    "GHST",
};
#endif

#ifdef USE_RX_SPI
// sync with rx_spi_protocol_e
static const char * const lookupTableRxSpi[] = {
    "V202_250K",
    "V202_1M",
    "SYMA_X",
    "SYMA_X5C",
    "CX10",
    "CX10A",
    "H8_3D",
    "INAV",
    "FRSKY_D",
    "FRSKY_X",
    "FLYSKY",
    "FLYSKY_2A",
    "KN",
    "SFHSS",
    "SPEKTRUM",
    "FRSKY_X_LBT",
    "REDPINE",
    "FRSKY_X_V2",
    "FRSKY_X_LBT_V2",
};
#endif

static const char * const lookupTableGyroHardwareLpf[] = {
    "NORMAL",
#ifdef USE_GYRO_DLPF_EXPERIMENTAL
    "EXPERIMENTAL"
#endif
};

#ifdef USE_CAMERA_CONTROL
static const char * const lookupTableCameraControlMode[] = {
    "HARDWARE_PWM",
    "SOFTWARE_PWM",
    "DAC"
};
#endif

static const char * const lookupTablePwmProtocol[] = {
    "PWM", "MULTISHOT", "BRUSHED",
    "DSHOT150", "DSHOT300", "DSHOT600", "DSHOT1200", "DSHOT2400", "DSHOT4800", "PROSHOT1000",
    "DISABLED"
};

static const char * const lookupTableRcInterpolation[] = {
    "OFF", "PRESET", "AUTO", "MANUAL"
};

static const char * const lookupTableRcInterpolationChannels[] = {
    "RP", "RPY", "RPYT", "T", "RPT",
};

static const char * const lookupTableLowpassType[] = {
    "PT1",
    "BIQUAD",
    "PT3",
    "PT4",
};

static const char * const lookupTableAntiGravityMode[] = {
    "SMOOTH",
    "STEP",
};

static const char * const lookupTableFailsafe[] = {
    "AUTO-LAND", "DROP", "GPS-RESCUE"
};

static const char * const lookupTableFailsafeSwitchMode[] = {
    "STAGE1", "KILL", "STAGE2"
};

static const char * const lookupTableBusType[] = {
    "NONE", "I2C", "SPI", "SLAVE",
#if defined(USE_SPI_GYRO) && defined(USE_I2C_GYRO)
    "GYROAUTO"
#endif
};

#ifdef USE_MAX7456
static const char * const lookupTableMax7456Clock[] = {
    "HALF", "DEFAULT", "FULL"
};
#endif

#ifdef USE_RX_FRSKY_SPI
static const char * const lookupTableFrskySpiA1Source[] = {
    "VBAT", "EXTADC", "CONST"
};
#endif

#ifdef USE_GYRO_OVERFLOW_CHECK
static const char * const lookupTableGyroOverflowCheck[] = {
    "OFF", "YAW", "ALL"
};
#endif

static const char * const lookupTableRatesType[] = {
    "BETAFLIGHT", "RACEFLIGHT", "KISS", "ACTUAL"
};

#ifdef USE_OVERCLOCK
static const char * const lookupOverclock[] = {
    "OFF",
#if defined(STM32F40_41xxx) || defined(STM32G4)
    "192MHZ", "216MHZ", "240MHZ"
#elif defined(STM32F411xE)
    "108MHZ", "120MHZ"
#elif defined(STM32F7)
    "240MHZ"
#endif
};
#endif

#ifdef USE_LED_STRIP
    static const char * const lookupLedStripFormatRGB[] = {
        "GRB", "RGB"
    };
#endif

static const char * const lookupTableThrottleLimitType[] = {
    "OFF", "SCALE", "CLIP"
};


#ifdef USE_GPS_RESCUE
static const char * const lookupTableRescueSanityType[] = {
    "RESCUE_SANITY_OFF", "RESCUE_SANITY_ON", "RESCUE_SANITY_FS_ONLY"
};
const char * const lookupTableRescueAltitudeMode[] = {
    "MAX_ALT", "FIXED_ALT", "CURRENT_ALT"
};
#endif

#if defined(USE_MAX7456) || defined(USE_FRSKYOSD)
static const char * const lookupTableVideoSystem[] = {
    "AUTO", "PAL", "NTSC"
};
#endif

#ifdef USE_RC_SMOOTHING_FILTER
static const char * const lookupTableRcSmoothingType[] = {
    "INTERPOLATION", "FILTER"
};
static const char * const lookupTableRcSmoothingDebug[] = {
    "ROLL", "PITCH", "YAW", "THROTTLE"
};
static const char * const lookupTableRcSmoothingInputType[] = {
    "PT1", "BIQUAD"
};
static const char * const lookupTableRcSmoothingDerivativeType[] = {
    "OFF", "PT1", "BIQUAD", "AUTO"
};
#endif // USE_RC_SMOOTHING_FILTER

#ifdef USE_VTX_COMMON
static const char * const lookupTableVtxLowPowerDisarm[] = {
    "OFF", "ON", "UNTIL_FIRST_ARM"
};
#endif

#ifdef USE_SDCARD
static const char * const lookupTableSdcardMode[] = {
    "OFF", "SPI", "SDIO"
};
#endif

#ifdef USE_LAUNCH_CONTROL
static const char * const lookupTableLaunchControlMode[] = {
    "NORMAL", "PITCHONLY", "FULL"
};
#endif

#ifdef USE_LED_STRIP
#ifdef USE_LED_STRIP_STATUS_MODE
static const char * const lookupTableLEDProfile[] = {
    "RACE", "BEACON", "STATUS"
};
#else
static const char * const lookupTableLEDProfile[] = {
    "RACE", "BEACON"
};
#endif
#endif

const char * const lookupTableLedstripColors[COLOR_COUNT] = {
    "BLACK",
    "WHITE",
    "RED",
    "ORANGE",
    "YELLOW",
    "LIME_GREEN",
    "GREEN",
    "MINT_GREEN",
    "CYAN",
    "LIGHT_BLUE",
    "BLUE",
    "DARK_VIOLET",
    "MAGENTA",
    "DEEP_PINK"
};

static const char * const lookupTableGyroFilterDebug[] = {
    "ROLL", "PITCH", "YAW"
};

static const char * const lookupTablePositionAltSource[] = {
    "DEFAULT", "BARO_ONLY", "GPS_ONLY"
};

static const char * const lookupTableOffOnAuto[] = {
    "OFF", "ON", "AUTO"
};

const char* const lookupTableInterpolatedSetpoint[] = {
    "OFF", "ON", "AVERAGED_2", "AVERAGED_3", "AVERAGED_4"
};

static const char* const lookupTableDshotBitbangedTimer[] = {
    "AUTO", "TIM1", "TIM8"
};

const char * const lookupTableOsdDisplayPortDevice[] = {
    "NONE", "AUTO", "MAX7456", "MSP", "FRSKYOSD"
};

#ifdef USE_OSD
static const char* const lookupTableOsdLogoOnArming[] = {
    "OFF", "ON", "FIRST_ARMING",
};
#endif

#if defined(GYRO_USES_SPI) && defined(USE_32K_CAPABLE_GYRO)
static const char* const lookupTable32k[] = {
  "OFF", "32K", "16K",
};
#endif

#ifdef USE_OSD
const char * const lookupTableCMSMenuBackgroundType[] = {
    "TRANSPARENT", "BLACK", "GRAY", "LIGHT_GRAY"
};
#endif

#define LOOKUP_TABLE_ENTRY(name) { name, ARRAYLEN(name) }

const lookupTableEntry_t lookupTables[] = {
    LOOKUP_TABLE_ENTRY(lookupTableOffOn),
    LOOKUP_TABLE_ENTRY(lookupTableUnit),
    LOOKUP_TABLE_ENTRY(lookupTableAlignment),
#ifdef USE_GPS
    LOOKUP_TABLE_ENTRY(lookupTableGPSProvider),
    LOOKUP_TABLE_ENTRY(lookupTableGPSSBASMode),
    LOOKUP_TABLE_ENTRY(lookupTableGPSUBLOXMode),
#ifdef USE_GPS_RESCUE
    LOOKUP_TABLE_ENTRY(lookupTableRescueSanityType),
    LOOKUP_TABLE_ENTRY(lookupTableRescueAltitudeMode),
#endif
#endif
#ifdef USE_BLACKBOX
    LOOKUP_TABLE_ENTRY(lookupTableBlackboxDevice),
    LOOKUP_TABLE_ENTRY(lookupTableBlackboxMode),
    LOOKUP_TABLE_ENTRY(lookupTableBlackboxSampleRate),
#endif
    LOOKUP_TABLE_ENTRY(currentMeterSourceNames),
    LOOKUP_TABLE_ENTRY(voltageMeterSourceNames),
#ifdef USE_SERVOS
    LOOKUP_TABLE_ENTRY(lookupTableGimbalMode),
#endif
#ifdef USE_SERIAL_RX
    LOOKUP_TABLE_ENTRY(lookupTableSerialRX),
#endif
#ifdef USE_RX_SPI
    LOOKUP_TABLE_ENTRY(lookupTableRxSpi),
#endif
    LOOKUP_TABLE_ENTRY(lookupTableGyroHardwareLpf),
    LOOKUP_TABLE_ENTRY(lookupTableAccHardware),
#ifdef USE_BARO
    LOOKUP_TABLE_ENTRY(lookupTableBaroHardware),
#endif
#ifdef USE_MAG
    LOOKUP_TABLE_ENTRY(lookupTableMagHardware),
#endif
    LOOKUP_TABLE_ENTRY(debugModeNames),
    LOOKUP_TABLE_ENTRY(lookupTablePwmProtocol),
    LOOKUP_TABLE_ENTRY(lookupTableRcInterpolation),
    LOOKUP_TABLE_ENTRY(lookupTableRcInterpolationChannels),
    LOOKUP_TABLE_ENTRY(lookupTableLowpassType),
    LOOKUP_TABLE_ENTRY(lookupTableAntiGravityMode),
    LOOKUP_TABLE_ENTRY(lookupTableFailsafe),
    LOOKUP_TABLE_ENTRY(lookupTableFailsafeSwitchMode),
    LOOKUP_TABLE_ENTRY(lookupTableCrashRecovery),
#ifdef USE_CAMERA_CONTROL
    LOOKUP_TABLE_ENTRY(lookupTableCameraControlMode),
#endif
    LOOKUP_TABLE_ENTRY(lookupTableBusType),
#ifdef USE_MAX7456
    LOOKUP_TABLE_ENTRY(lookupTableMax7456Clock),
#endif
#ifdef USE_RX_FRSKY_SPI
    LOOKUP_TABLE_ENTRY(lookupTableFrskySpiA1Source),
#endif
#ifdef USE_RANGEFINDER
    LOOKUP_TABLE_ENTRY(lookupTableRangefinderHardware),
#endif
#ifdef USE_GYRO_OVERFLOW_CHECK
    LOOKUP_TABLE_ENTRY(lookupTableGyroOverflowCheck),
#endif
    LOOKUP_TABLE_ENTRY(lookupTableRatesType),
#ifdef USE_OVERCLOCK
    LOOKUP_TABLE_ENTRY(lookupOverclock),
#endif
#ifdef USE_LED_STRIP
    LOOKUP_TABLE_ENTRY(lookupLedStripFormatRGB),
#endif
#ifdef USE_MULTI_GYRO
    LOOKUP_TABLE_ENTRY(lookupTableGyro),
#endif
    LOOKUP_TABLE_ENTRY(lookupTableThrottleLimitType),
#if defined(USE_MAX7456) || defined(USE_FRSKYOSD)
    LOOKUP_TABLE_ENTRY(lookupTableVideoSystem),
#endif
#ifdef USE_RC_SMOOTHING_FILTER
    LOOKUP_TABLE_ENTRY(lookupTableRcSmoothingType),
    LOOKUP_TABLE_ENTRY(lookupTableRcSmoothingDebug),
    LOOKUP_TABLE_ENTRY(lookupTableRcSmoothingInputType),
    LOOKUP_TABLE_ENTRY(lookupTableRcSmoothingDerivativeType),
#endif // USE_RC_SMOOTHING_FILTER
#ifdef USE_VTX_COMMON
    LOOKUP_TABLE_ENTRY(lookupTableVtxLowPowerDisarm),
#endif
    LOOKUP_TABLE_ENTRY(lookupTableGyroHardware),
#ifdef USE_SDCARD
    LOOKUP_TABLE_ENTRY(lookupTableSdcardMode),
#endif
#ifdef USE_LAUNCH_CONTROL
    LOOKUP_TABLE_ENTRY(lookupTableLaunchControlMode),
#endif
#ifdef USE_LED_STRIP
    LOOKUP_TABLE_ENTRY(lookupTableLEDProfile),
    LOOKUP_TABLE_ENTRY(lookupTableLedstripColors),
#endif

    LOOKUP_TABLE_ENTRY(lookupTableGyroFilterDebug),

    LOOKUP_TABLE_ENTRY(lookupTablePositionAltSource),
    LOOKUP_TABLE_ENTRY(lookupTableOffOnAuto),
    LOOKUP_TABLE_ENTRY(lookupTableInterpolatedSetpoint),
    LOOKUP_TABLE_ENTRY(lookupTableDshotBitbangedTimer),
    LOOKUP_TABLE_ENTRY(lookupTableOsdDisplayPortDevice),

#ifdef USE_OSD
    LOOKUP_TABLE_ENTRY(lookupTableOsdLogoOnArming),
#endif

#if defined(GYRO_USES_SPI) && defined(USE_32K_CAPABLE_GYRO)
    LOOKUP_TABLE_ENTRY(lookupTable32k),
#endif

#ifdef USE_OSD
    LOOKUP_TABLE_ENTRY(lookupTableCMSMenuBackgroundType),
#endif
};

#undef LOOKUP_TABLE_ENTRY

const clivalue_t valueTable[] = {
// PG_GYRO_CONFIG
    { "00",          VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_GYRO_HARDWARE_LPF }, PG_GYRO_CONFIG, offsetof(gyroConfig_t, gyro_hardware_lpf) }, // gyro_hardware_lpf
#if defined(USE_GYRO_SPI_ICM20649)
    { "01",            VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_GYRO_CONFIG, offsetof(gyroConfig_t, gyro_high_fsr) }, // gyro_high_range
#endif

    { "02",          VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_LOWPASS_TYPE }, PG_GYRO_CONFIG, offsetof(gyroConfig_t, gyro_lowpass_type) }, // gyro_lowpass_type

    { "03",             VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 0, FILTER_FREQUENCY_MAX }, PG_GYRO_CONFIG, offsetof(gyroConfig_t, gyro_soft_notch_hz_1) }, // gyro_notch1_hz
    { "04",         VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 0, FILTER_FREQUENCY_MAX }, PG_GYRO_CONFIG, offsetof(gyroConfig_t, gyro_soft_notch_cutoff_1) }, // gyro_notch1_cutoff

    { "05",             VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 0, 1000 }, PG_GYRO_CONFIG, offsetof(gyroConfig_t, alpha) }, // gyro_abg_alpha
    { "06",             VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 0, 2000 }, PG_GYRO_CONFIG, offsetof(gyroConfig_t, abg_boost) }, // gyro_abg_boost
    { "07",         VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 0, 1000 }, PG_GYRO_CONFIG, offsetof(gyroConfig_t, abg_half_life) }, // gyro_abg_half_life

    { "08",        VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 50,  3000 }, PG_GYRO_CONFIG, offsetof(gyroConfig_t, gyroCalibrationDuration) }, // gyro_calib_duration
    { "09",     VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 0,  200 }, PG_GYRO_CONFIG, offsetof(gyroConfig_t, gyroMovementCalibrationThreshold) }, // gyro_calib_noise_limit
    { "0a",            VAR_INT16  | MASTER_VALUE, .config.minmax = { -1000, 1000 }, PG_GYRO_CONFIG, offsetof(gyroConfig_t, gyro_offset_yaw) }, // gyro_offset_yaw
#ifdef USE_GYRO_IMUF9001
    { "0b",                  VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_IMUF_RATE }, PG_GYRO_CONFIG, offsetof(gyroConfig_t, imuf_rate) }, // imuf_rate
    { "0c",                VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 0, 16000 }, PG_GYRO_CONFIG, offsetof(gyroConfig_t, imuf_roll_q) }, // imuf_roll_q
    { "0d",               VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 0, 16000 }, PG_GYRO_CONFIG, offsetof(gyroConfig_t, imuf_pitch_q) }, // imuf_pitch_q
    { "0e",                 VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 0, 16000 }, PG_GYRO_CONFIG, offsetof(gyroConfig_t, imuf_yaw_q) }, // imuf_yaw_q
    { "0f",                     VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 3, 500  }, PG_GYRO_CONFIG, offsetof(gyroConfig_t, imuf_w) }, // imuf_w
    { "0g",   VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 0, 450   }, PG_GYRO_CONFIG, offsetof(gyroConfig_t, imuf_pitch_lpf_cutoff_hz) }, // imuf_pitch_lpf_cutoff_hz
    { "0h",    VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 0, 450   }, PG_GYRO_CONFIG, offsetof(gyroConfig_t, imuf_roll_lpf_cutoff_hz) }, // imuf_roll_lpf_cutoff_hz
    { "0i",     VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 0, 450   }, PG_GYRO_CONFIG, offsetof(gyroConfig_t, imuf_yaw_lpf_cutoff_hz) }, // imuf_yaw_lpf_cutoff_hz
    { "0j",     VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 30, 180  }, PG_GYRO_CONFIG, offsetof(gyroConfig_t, imuf_acc_lpf_cutoff_hz) }, // imuf_acc_lpf_cutoff_hz
#else
    { "0c",                VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 0, 16000 }, PG_GYRO_CONFIG, offsetof(gyroConfig_t, imuf_roll_q) }, // imuf_roll_q
    { "0d",               VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 0, 16000 }, PG_GYRO_CONFIG, offsetof(gyroConfig_t, imuf_pitch_q) }, // imuf_pitch_q
    { "0e",                 VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 0, 16000 }, PG_GYRO_CONFIG, offsetof(gyroConfig_t, imuf_yaw_q) }, // imuf_yaw_q
    { "0f",                     VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 0, 300   }, PG_GYRO_CONFIG, offsetof(gyroConfig_t, imuf_w) }, // imuf_w
#endif
#ifdef USE_GYRO_OVERFLOW_CHECK
    { "0k",       VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_GYRO_OVERFLOW_CHECK }, PG_GYRO_CONFIG, offsetof(gyroConfig_t, checkOverflow) }, // gyro_overflow_detect
#endif
#ifdef USE_YAW_SPIN_RECOVERY
    { "0l",          VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON_AUTO }, PG_GYRO_CONFIG, offsetof(gyroConfig_t, yaw_spin_recovery) }, // yaw_spin_recovery
    { "0m",         VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { YAW_SPIN_RECOVERY_THRESHOLD_MIN,  YAW_SPIN_RECOVERY_THRESHOLD_MAX }, PG_GYRO_CONFIG, offsetof(gyroConfig_t, yaw_spin_threshold) }, // yaw_spin_threshold
#endif
#if defined(GYRO_USES_SPI) && defined(USE_32K_CAPABLE_GYRO)
    { "0n",             VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_32K }, PG_GYRO_CONFIG, offsetof(gyroConfig_t, gyro_use_32khz) }, // gyro_use_32khz
#endif
#ifdef USE_MULTI_GYRO
    { "0o",                VAR_UINT8  | HARDWARE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_GYRO }, PG_GYRO_CONFIG, offsetof(gyroConfig_t, gyro_to_use) }, // gyro_to_use
#endif
#if defined(USE_GYRO_DATA_ANALYSE)
    { "0p",                VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 1, 1000 }, PG_GYRO_CONFIG, offsetof(gyroConfig_t, dyn_notch_q) }, // dyn_matrix_q
    { "0q",           VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 60, 250 }, PG_GYRO_CONFIG, offsetof(gyroConfig_t, dyn_notch_min_hz) }, // dyn_matrix_min_hz
    { "0r",           VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 200, 1000 }, PG_GYRO_CONFIG, offsetof(gyroConfig_t, dyn_notch_max_hz) }, // dyn_matrix_max_hz
#endif
#ifdef USE_DYN_LPF
    { "0s",        VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 0, 1000 }, PG_GYRO_CONFIG, offsetof(gyroConfig_t, dyn_lpf_gyro_min_hz) }, // dyn_lpf_gyro_min_hz
    { "0t",         VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 0, 255 }, PG_GYRO_CONFIG, offsetof(gyroConfig_t, dyn_lpf_gyro_width) }, // dyn_lpf_gyro_width
    { "0u",    VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 0, 10 }, PG_GYRO_CONFIG, offsetof(gyroConfig_t, dyn_lpf_curve_expo) }, // dyn_lpf_gyro_curve_expo
    { "0v",              VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 0,  200 },    PG_GYRO_CONFIG, offsetof(gyroConfig_t, dyn_lpf_gyro_gain) }, // dyn_lpf2_gain
#endif
    { "0w",     VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_GYRO_FILTER_DEBUG }, PG_GYRO_CONFIG, offsetof(gyroConfig_t, gyro_filter_debug_axis) }, // gyro_filter_debug_axis

// PG_ACCELEROMETER_CONFIG
#if defined(USE_ACC)
    { "0x",               VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_ACC_HARDWARE }, PG_ACCELEROMETER_CONFIG, offsetof(accelerometerConfig_t, acc_hardware) }, // acc_hardware
#if defined(USE_GYRO_SPI_ICM20649)
    { "0y",             VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_ACCELEROMETER_CONFIG, offsetof(accelerometerConfig_t, acc_high_fsr) }, // acc_high_range
#endif
    { "0z",                 VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 0, 400 }, PG_ACCELEROMETER_CONFIG, offsetof(accelerometerConfig_t, acc_lpf_hz) }, // acc_lpf_hz
    { "0A",             VAR_INT16  | MASTER_VALUE, .config.minmax = { -300, 300 }, PG_ACCELEROMETER_CONFIG, offsetof(accelerometerConfig_t, accelerometerTrims.values.pitch) }, // acc_trim_pitch
    { "0B",              VAR_INT16  | MASTER_VALUE, .config.minmax = { -300, 300 }, PG_ACCELEROMETER_CONFIG, offsetof(accelerometerConfig_t, accelerometerTrims.values.roll) }, // acc_trim_roll

    // 4 elements are output for the ACC calibration - The 3 axis values and the 4th representing whether calibration has been performed
    { "0C",            VAR_INT16  | MASTER_VALUE | MODE_ARRAY, .config.array.length = 4, PG_ACCELEROMETER_CONFIG, offsetof(accelerometerConfig_t, accZero.raw) }, // acc_calibration
#endif

// PG_COMPASS_CONFIG
#ifdef USE_MAG
    { "0D",                  VAR_UINT8  | HARDWARE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_ALIGNMENT }, PG_COMPASS_CONFIG, offsetof(compassConfig_t, mag_alignment) }, // align_mag
    { "0E",             VAR_INT16  | HARDWARE_VALUE, .config.minmax = { -3600, 3600 }, PG_COMPASS_CONFIG, offsetof(compassConfig_t, mag_customAlignment.roll) }, // mag_align_roll
    { "0F",            VAR_INT16  | HARDWARE_VALUE, .config.minmax = { -3600, 3600 }, PG_COMPASS_CONFIG, offsetof(compassConfig_t, mag_customAlignment.pitch) }, // mag_align_pitch
    { "0G",              VAR_INT16  | HARDWARE_VALUE, .config.minmax = { -3600, 3600 }, PG_COMPASS_CONFIG, offsetof(compassConfig_t, mag_customAlignment.yaw) }, // mag_align_yaw
    { "0H",                VAR_UINT8  | HARDWARE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_BUS_TYPE }, PG_COMPASS_CONFIG, offsetof(compassConfig_t, mag_bustype) }, // mag_bustype
    { "0I",             VAR_UINT8  | HARDWARE_VALUE, .config.minmaxUnsigned = { 0, I2CDEV_COUNT }, PG_COMPASS_CONFIG, offsetof(compassConfig_t, mag_i2c_device) }, // mag_i2c_device
    { "0J",            VAR_UINT8  | HARDWARE_VALUE, .config.minmaxUnsigned = { 0, I2C_ADDR7_MAX }, PG_COMPASS_CONFIG, offsetof(compassConfig_t, mag_i2c_address) }, // mag_i2c_address
    { "0K",             VAR_UINT8  | HARDWARE_VALUE, .config.minmaxUnsigned = { 0, SPIDEV_COUNT }, PG_COMPASS_CONFIG, offsetof(compassConfig_t, mag_spi_device) }, // mag_spi_device
    { "0L",               VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_MAG_HARDWARE }, PG_COMPASS_CONFIG, offsetof(compassConfig_t, mag_hardware) }, // mag_hardware
    { "0M",            VAR_INT16  | MASTER_VALUE | MODE_ARRAY, .config.array.length = XYZ_AXIS_COUNT, PG_COMPASS_CONFIG, offsetof(compassConfig_t, magZero.raw) }, // mag_calibration
#endif

// PG_BAROMETER_CONFIG
#ifdef USE_BARO
    { "0N",               VAR_UINT8  | HARDWARE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_BUS_TYPE }, PG_BAROMETER_CONFIG, offsetof(barometerConfig_t, baro_bustype) }, // baro_bustype
    { "0O",            VAR_UINT8  | HARDWARE_VALUE, .config.minmaxUnsigned = { 0, 5 }, PG_BAROMETER_CONFIG, offsetof(barometerConfig_t, baro_spi_device) }, // baro_spi_device
    { "0P",            VAR_UINT8  | HARDWARE_VALUE, .config.minmaxUnsigned = { 0, 5 }, PG_BAROMETER_CONFIG, offsetof(barometerConfig_t, baro_i2c_device) }, // baro_i2c_device
    { "0Q",           VAR_UINT8  | HARDWARE_VALUE, .config.minmaxUnsigned = { 0, I2C_ADDR7_MAX }, PG_BAROMETER_CONFIG, offsetof(barometerConfig_t, baro_i2c_address) }, // baro_i2c_address
    { "0R",              VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_BARO_HARDWARE }, PG_BAROMETER_CONFIG, offsetof(barometerConfig_t, baro_hardware) }, // baro_hardware
    { "0S",              VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 2, BARO_SAMPLE_COUNT_MAX }, PG_BAROMETER_CONFIG, offsetof(barometerConfig_t, baro_sample_count) }, // baro_tab_size
    { "0T",             VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, 1000 }, PG_BAROMETER_CONFIG, offsetof(barometerConfig_t, baro_noise_lpf) }, // baro_noise_lpf
    { "0U",                VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, 1000 }, PG_BAROMETER_CONFIG, offsetof(barometerConfig_t, baro_cf_vel) }, // baro_cf_vel
#endif

// PG_RX_CONFIG
    { "0V",                     VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 1200, 1700 }, PG_RX_CONFIG, offsetof(rxConfig_t, midrc) }, // mid_rc
    { "0W",                  VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { PWM_PULSE_MIN, PWM_PULSE_MAX }, PG_RX_CONFIG, offsetof(rxConfig_t, mincheck) }, // min_check
    { "0X",                  VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { PWM_PULSE_MIN, PWM_PULSE_MAX }, PG_RX_CONFIG, offsetof(rxConfig_t, maxcheck) }, // max_check
    { "0Y",               VAR_INT8   | MASTER_VALUE, .config.minmaxUnsigned = { 0, MAX_SUPPORTED_RC_CHANNEL_COUNT }, PG_RX_CONFIG, offsetof(rxConfig_t, rssi_channel) }, // rssi_channel
    { "0Z",      VAR_INT8   | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_RX_CONFIG, offsetof(rxConfig_t, rssi_src_frame_errors) }, // rssi_src_frame_errors
    { "10",                 VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { RSSI_SCALE_MIN, RSSI_SCALE_MAX }, PG_RX_CONFIG, offsetof(rxConfig_t, rssi_scale) }, // rssi_scale
    { "11",                VAR_INT8   | MASTER_VALUE, .config.minmax = { -100, 100 }, PG_RX_CONFIG, offsetof(rxConfig_t, rssi_offset) }, // rssi_offset
    { "12",                VAR_INT8   | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_RX_CONFIG, offsetof(rxConfig_t, rssi_invert) }, // rssi_invert
    { "13",  VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 0, UINT8_MAX }, PG_RX_CONFIG, offsetof(rxConfig_t, rssi_src_frame_lpf_period) }, // rssi_src_frame_lpf_period
    { "14",                  VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_RC_INTERPOLATION }, PG_RX_CONFIG, offsetof(rxConfig_t, rcInterpolation) }, // rc_interp
    { "15",               VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_RC_INTERPOLATION_CHANNELS }, PG_RX_CONFIG, offsetof(rxConfig_t, rcInterpolationChannels) }, // rc_interp_ch
    { "16",              VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 1, 50 }, PG_RX_CONFIG, offsetof(rxConfig_t, rcInterpolationInterval) }, // rc_interp_int

#ifdef USE_RC_SMOOTHING_FILTER
    { "17",          VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_RC_SMOOTHING_TYPE }, PG_RX_CONFIG, offsetof(rxConfig_t, rc_smoothing_type) }, // rc_smoothing_type
    { "18",      VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 0, UINT8_MAX }, PG_RX_CONFIG, offsetof(rxConfig_t, rc_smoothing_input_cutoff) }, // rc_smoothing_input_hz
    { "19", VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 0, UINT8_MAX }, PG_RX_CONFIG, offsetof(rxConfig_t, rc_smoothing_derivative_cutoff) }, // rc_smoothing_derivative_hz
    { "1a",    VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_RC_SMOOTHING_DEBUG }, PG_RX_CONFIG, offsetof(rxConfig_t, rc_smoothing_debug_axis) }, // rc_smoothing_debug_axis
    { "1b",    VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_RC_SMOOTHING_INPUT_TYPE }, PG_RX_CONFIG, offsetof(rxConfig_t, rc_smoothing_input_type) }, // rc_smoothing_input_type
    { "1c",VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_RC_SMOOTHING_DERIVATIVE_TYPE }, PG_RX_CONFIG, offsetof(rxConfig_t, rc_smoothing_derivative_type) }, // rc_smoothing_derivative_type
    { "1d",VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { RC_SMOOTHING_AUTO_FACTOR_MIN, RC_SMOOTHING_AUTO_FACTOR_MAX }, PG_RX_CONFIG, offsetof(rxConfig_t, rc_smoothing_auto_factor) }, // rc_smoothing_auto_smoothness
#endif // USE_RC_SMOOTHING_FILTER
    { "1e",       VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 0, 100 }, PG_RX_CONFIG, offsetof(rxConfig_t, predictive_rc_percent) }, // rc_predictor_percent
    { "1f",   VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 0, UINT8_MAX }, PG_RX_CONFIG, offsetof(rxConfig_t, predictive_rc_cutoff) }, // rc_predictor_velocity_hz
    { "1g",          VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 10, UINT8_MAX }, PG_RX_CONFIG, offsetof(rxConfig_t, predictive_rc_time) }, // rc_predictor_time

    { "1h",            VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 0, 90 }, PG_RX_CONFIG, offsetof(rxConfig_t, fpvCamAngleDegrees) }, // fpv_mix_degrees
    { "1i",         VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_RX_CONFIG, offsetof(rxConfig_t, yawAroundGravity) }, // yaw_around_gravity
    { "1j",           VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 0, MAX_AUX_CHANNEL_COUNT }, PG_RX_CONFIG, offsetof(rxConfig_t, max_aux_channel) }, // max_aux_channels
#ifdef USE_SERIAL_RX
    { "1k",          VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_SERIAL_RX }, PG_RX_CONFIG, offsetof(rxConfig_t, serialrx_provider) }, // serialrx_provider
    { "1l",          VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_RX_CONFIG, offsetof(rxConfig_t, serialrx_inverted) }, // serialrx_inverted
#endif
#ifdef USE_SPEKTRUM_BIND
    { "1m",          VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { SPEKTRUM_SAT_BIND_DISABLED, SPEKTRUM_SAT_BIND_MAX}, PG_RX_CONFIG, offsetof(rxConfig_t, spektrum_sat_bind) }, // spektrum_sat_bind
    { "1n",          VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_RX_CONFIG, offsetof(rxConfig_t, spektrum_sat_bind_autoreset) }, // spektrum_sat_bind_autoreset
#endif
#ifdef USE_SERIALRX_SRXL2
    { "1o",             VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 0, 0xf }, PG_RX_CONFIG, offsetof(rxConfig_t, srxl2_unit_id) }, // srxl2_unit_id
    { "1p",           VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_RX_CONFIG, offsetof(rxConfig_t, srxl2_baud_fast) }, // srxl2_baud_fast
#endif
#if defined(USE_SERIALRX_SBUS)
    { "1q",           VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_RX_CONFIG, offsetof(rxConfig_t, sbus_baud_fast) }, // sbus_baud_fast
#endif
#if defined(USE_SERIALRX_CRSF)
    { "1r",          VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_RX_CONFIG, offsetof(rxConfig_t, crsf_use_rx_snr) }, // crsf_use_rx_snr
#endif
    { "1s",     VAR_UINT8 | MASTER_VALUE, .config.minmaxUnsigned = { 0, 100 }, PG_RX_CONFIG, offsetof(rxConfig_t, airModeActivateThreshold) }, // airmode_start_throttle_percent
    { "1t",                VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { PWM_PULSE_MIN, PWM_PULSE_MAX }, PG_RX_CONFIG, offsetof(rxConfig_t, rx_min_usec) }, // rx_min_usec
    { "1u",                VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { PWM_PULSE_MIN, PWM_PULSE_MAX }, PG_RX_CONFIG, offsetof(rxConfig_t, rx_max_usec) }, // rx_max_usec
    { "1v",        VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_RX_CONFIG, offsetof(rxConfig_t, halfDuplex) }, // serialrx_halfduplex
#if defined(USE_RX_MSP_OVERRIDE)
    { "1w",      VAR_UINT32 | MASTER_VALUE, .config.u32Max = (1 << MAX_SUPPORTED_RC_CHANNEL_COUNT) - 1, PG_RX_CONFIG, offsetof(rxConfig_t, msp_override_channels_mask)}, // msp_override_channels_mask
#endif
#ifdef USE_RX_SPI
    { "1x",            VAR_UINT8  | HARDWARE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_RX_SPI }, PG_RX_SPI_CONFIG, offsetof(rxSpiConfig_t, rx_spi_protocol) }, // rx_spi_protocol
    { "1y",                 VAR_UINT8  | HARDWARE_VALUE, .config.minmaxUnsigned = { 0, SPIDEV_COUNT }, PG_RX_SPI_CONFIG, offsetof(rxSpiConfig_t, spibus) }, // rx_spi_bus
    { "1z",       VAR_UINT8  | HARDWARE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_RX_SPI_CONFIG, offsetof(rxSpiConfig_t, ledInversion) }, // rx_spi_led_inversion
#endif
    { "1A",            VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_RX_CONFIG, offsetof(rxConfig_t, showAlteredRc) }, // show_altered_rc

// PG_ADC_CONFIG
#if defined(USE_ADC)
    { "1B",                 VAR_INT8 | HARDWARE_VALUE, .config.minmaxUnsigned = { 0, ADCDEV_COUNT }, PG_ADC_CONFIG, offsetof(adcConfig_t, device) }, // adc_device
    { "1C",    VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 0, 2000 }, PG_ADC_CONFIG, offsetof(adcConfig_t, vrefIntCalibration) }, // adc_vrefint_calibration
    { "1D", VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 0, 2000 }, PG_ADC_CONFIG, offsetof(adcConfig_t, tempSensorCalibration1) }, // adc_tempsensor_calibration30
    { "1E", VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 0, 2000 }, PG_ADC_CONFIG, offsetof(adcConfig_t, tempSensorCalibration2) }, // adc_tempsensor_calibration110
#endif

// PG_PWM_CONFIG
#if defined(USE_PWM)
    { "1F",       VAR_INT8   | MASTER_VALUE | MODE_LOOKUP,  .config.lookup = { TABLE_OFF_ON }, PG_PWM_CONFIG, offsetof(pwmConfig_t, inputFilteringMode) }, // input_filtering_mode
#endif

// PG_BLACKBOX_CONFIG
#ifdef USE_BLACKBOX
    { "1G",       VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_BLACKBOX_SAMPLE_RATE }, PG_BLACKBOX_CONFIG, offsetof(blackboxConfig_t, sample_rate) }, // blackbox_sample_rate
    { "1H",            VAR_UINT8  | HARDWARE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_BLACKBOX_DEVICE }, PG_BLACKBOX_CONFIG, offsetof(blackboxConfig_t, device) }, // blackbox_device
    { "1I",      VAR_UINT32 | MASTER_VALUE | MODE_BITSET, .config.bitpos = FLIGHT_LOG_FIELD_SELECT_PID,   PG_BLACKBOX_CONFIG, offsetof(blackboxConfig_t, fields_disabled_mask) }, // blackbox_disable_pids
    { "1J",        VAR_UINT32 | MASTER_VALUE | MODE_BITSET, .config.bitpos = FLIGHT_LOG_FIELD_SELECT_RC_COMMANDS,   PG_BLACKBOX_CONFIG, offsetof(blackboxConfig_t, fields_disabled_mask) }, // blackbox_disable_rc
    { "1K",  VAR_UINT32 | MASTER_VALUE | MODE_BITSET, .config.bitpos = FLIGHT_LOG_FIELD_SELECT_SETPOINT,   PG_BLACKBOX_CONFIG, offsetof(blackboxConfig_t, fields_disabled_mask) }, // blackbox_disable_setpoint
    { "1L",       VAR_UINT32 | MASTER_VALUE | MODE_BITSET, .config.bitpos = FLIGHT_LOG_FIELD_SELECT_BATTERY,   PG_BLACKBOX_CONFIG, offsetof(blackboxConfig_t, fields_disabled_mask) }, // blackbox_disable_bat
#ifdef USE_MAG
    { "1M",       VAR_UINT32 | MASTER_VALUE | MODE_BITSET, .config.bitpos = FLIGHT_LOG_FIELD_SELECT_MAG,   PG_BLACKBOX_CONFIG, offsetof(blackboxConfig_t, fields_disabled_mask) }, // blackbox_disable_mag
#endif
#if defined(USE_BARO) || defined(USE_RANGEFINDER)
    { "1N",       VAR_UINT32 | MASTER_VALUE | MODE_BITSET, .config.bitpos = FLIGHT_LOG_FIELD_SELECT_ALTITUDE,   PG_BLACKBOX_CONFIG, offsetof(blackboxConfig_t, fields_disabled_mask) }, // blackbox_disable_alt
#endif
    { "1O",      VAR_UINT32 | MASTER_VALUE | MODE_BITSET, .config.bitpos = FLIGHT_LOG_FIELD_SELECT_RSSI,   PG_BLACKBOX_CONFIG, offsetof(blackboxConfig_t, fields_disabled_mask) }, // blackbox_disable_rssi
    { "1P",      VAR_UINT32 | MASTER_VALUE | MODE_BITSET, .config.bitpos = FLIGHT_LOG_FIELD_SELECT_GYRO,   PG_BLACKBOX_CONFIG, offsetof(blackboxConfig_t, fields_disabled_mask) }, // blackbox_disable_gyro
#if defined(USE_ACC)
    { "1Q",       VAR_UINT32 | MASTER_VALUE | MODE_BITSET, .config.bitpos = FLIGHT_LOG_FIELD_SELECT_ACC,   PG_BLACKBOX_CONFIG, offsetof(blackboxConfig_t, fields_disabled_mask) }, // blackbox_disable_acc
#endif
    { "1R",     VAR_UINT32 | MASTER_VALUE | MODE_BITSET, .config.bitpos = FLIGHT_LOG_FIELD_SELECT_DEBUG_LOG,   PG_BLACKBOX_CONFIG, offsetof(blackboxConfig_t, fields_disabled_mask) }, // blackbox_disable_debug
    { "1S",    VAR_UINT32 | MASTER_VALUE | MODE_BITSET, .config.bitpos = FLIGHT_LOG_FIELD_SELECT_MOTOR,   PG_BLACKBOX_CONFIG, offsetof(blackboxConfig_t, fields_disabled_mask) }, // blackbox_disable_motors
#ifdef USE_GPS
    { "1T",       VAR_UINT32 | MASTER_VALUE | MODE_BITSET, .config.bitpos = FLIGHT_LOG_FIELD_SELECT_GPS,   PG_BLACKBOX_CONFIG, offsetof(blackboxConfig_t, fields_disabled_mask) }, // blackbox_disable_gps
#endif
    { "1U",              VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_BLACKBOX_MODE }, PG_BLACKBOX_CONFIG, offsetof(blackboxConfig_t, mode) }, // blackbox_mode
#endif

// PG_MOTOR_CONFIG
    { "1V",               VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { PWM_PULSE_MIN, PWM_PULSE_MAX }, PG_MOTOR_CONFIG, offsetof(motorConfig_t, minthrottle) }, // min_throttle
    { "1Q",               VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { PWM_PULSE_MIN, PWM_PULSE_MAX }, PG_MOTOR_CONFIG, offsetof(motorConfig_t, maxthrottle) }, // max_throttle
    { "1R",                VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { PWM_PULSE_MIN, PWM_PULSE_MAX }, PG_MOTOR_CONFIG, offsetof(motorConfig_t, mincommand) }, // min_command
#ifdef USE_DSHOT
    { "1S",           VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, 2000 }, PG_MOTOR_CONFIG, offsetof(motorConfig_t, digitalIdleOffsetValue) }, // dshot_idle_value
#ifdef USE_DSHOT_DMAR
    { "1T",                VAR_UINT8  | HARDWARE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON_AUTO }, PG_MOTOR_CONFIG, offsetof(motorConfig_t, dev.useBurstDshot) }, // dshot_burst
#endif
#ifdef USE_DSHOT_TELEMETRY
    { "1U",            VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_MOTOR_CONFIG, offsetof(motorConfig_t, dev.useDshotTelemetry) }, // dshot_bidir
#endif
#ifdef USE_DSHOT_BITBANG
    { "1V",               VAR_UINT8  | HARDWARE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON_AUTO }, PG_MOTOR_CONFIG, offsetof(motorConfig_t, dev.useDshotBitbang) }, // dshot_bitbang
    { "1W",         VAR_UINT8  | HARDWARE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_DSHOT_BITBANGED_TIMER }, PG_MOTOR_CONFIG, offsetof(motorConfig_t, dev.useDshotBitbangedTimer) }, // dshot_bitbang_timer
#endif
#endif
    { "1X",           VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_MOTOR_CONFIG, offsetof(motorConfig_t, dev.useUnsyncedPwm) }, // use_unsynced_pwm
    { "1Y",         VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_MOTOR_PWM_PROTOCOL }, PG_MOTOR_CONFIG, offsetof(motorConfig_t, dev.motorPwmProtocol) }, // motor_pwm_protocol
    { "1Z",             VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 200, 32000 }, PG_MOTOR_CONFIG, offsetof(motorConfig_t, dev.motorPwmRate) }, // motor_pwm_rate
    { "20",        VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_MOTOR_CONFIG, offsetof(motorConfig_t, dev.motorPwmInversion) }, // motor_pwm_inversion
    { "21",                VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 4, UINT8_MAX }, PG_MOTOR_CONFIG, offsetof(motorConfig_t, motorPoleCount) }, // motor_poles
    { "22",    VAR_UINT8  | MASTER_VALUE | MODE_ARRAY, .config.array.length = MAX_SUPPORTED_MOTORS, PG_MOTOR_CONFIG, offsetof(motorConfig_t, dev.motorOutputReordering)}, // motor_output_reordering

// PG_THROTTLE_CORRECTION_CONFIG
    { "23",             VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 0,  150 }, PG_THROTTLE_CORRECTION_CONFIG, offsetof(throttleCorrectionConfig_t, throttle_correction_value) }, // thr_corr_value
    { "24",             VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 1,  900 }, PG_THROTTLE_CORRECTION_CONFIG, offsetof(throttleCorrectionConfig_t, throttle_correction_angle) }, // thr_corr_angle

// PG_FAILSAFE_CONFIG
    { "25",             VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 0, 200 }, PG_FAILSAFE_CONFIG, offsetof(failsafeConfig_t, failsafe_delay) }, // failsafe_delay
    { "26",         VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 0, 200 }, PG_FAILSAFE_CONFIG, offsetof(failsafeConfig_t, failsafe_off_delay) }, // failsafe_off_delay
    { "27",          VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { PWM_PULSE_MIN, PWM_PULSE_MAX }, PG_FAILSAFE_CONFIG, offsetof(failsafeConfig_t, failsafe_throttle) }, // failsafe_throttle
    { "28",       VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_FAILSAFE_SWITCH_MODE }, PG_FAILSAFE_CONFIG, offsetof(failsafeConfig_t, failsafe_switch_mode) }, // failsafe_switch_mode
    { "29",       VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 0, 300 }, PG_FAILSAFE_CONFIG, offsetof(failsafeConfig_t, failsafe_throttle_low_delay) }, // failsafe_throttle_low_delay
    { "2a",         VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_FAILSAFE }, PG_FAILSAFE_CONFIG, offsetof(failsafeConfig_t, failsafe_procedure) }, // failsafe_procedure
    { "2b",    VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 0, 200 }, PG_FAILSAFE_CONFIG, offsetof(failsafeConfig_t, failsafe_recovery_delay) }, // failsafe_recovery_delay
    { "2c",   VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 0, 50 }, PG_FAILSAFE_CONFIG, offsetof(failsafeConfig_t, failsafe_stick_threshold) }, // failsafe_stick_threshold

// PG_BOARDALIGNMENT_CONFIG
    { "2d",           VAR_INT16  | MASTER_VALUE, .config.minmax = { -180, 360 }, PG_BOARD_ALIGNMENT, offsetof(boardAlignment_t, rollDegrees) }, // align_board_roll
    { "2e",          VAR_INT16  | MASTER_VALUE, .config.minmax = { -180, 360 }, PG_BOARD_ALIGNMENT, offsetof(boardAlignment_t, pitchDegrees) }, // align_board_pitch
    { "2f",            VAR_INT16  | MASTER_VALUE, .config.minmax = { -180, 360 }, PG_BOARD_ALIGNMENT, offsetof(boardAlignment_t, yawDegrees) }, // align_board_yaw

// PG_GIMBAL_CONFIG
#ifdef USE_SERVOS
    { "2g",                VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_GIMBAL_MODE }, PG_GIMBAL_CONFIG, offsetof(gimbalConfig_t, mode) }, // gimbal_mode
#endif

// PG_BATTERY_CONFIG
    { "2h",               VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 0, 20000 }, PG_BATTERY_CONFIG, offsetof(batteryConfig_t, batteryCapacity) }, // bat_capacity
    { "2i",      VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { VBAT_CELL_VOTAGE_RANGE_MIN, VBAT_CELL_VOTAGE_RANGE_MAX }, PG_BATTERY_CONFIG, offsetof(batteryConfig_t, vbatmaxcellvoltage) }, // vbat_max_cell_voltage
    { "2j",     VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { VBAT_CELL_VOTAGE_RANGE_MIN, VBAT_CELL_VOTAGE_RANGE_MAX }, PG_BATTERY_CONFIG, offsetof(batteryConfig_t, vbatfullcellvoltage) }, // vbat_full_cell_voltage
    { "2k",      VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { VBAT_CELL_VOTAGE_RANGE_MIN, VBAT_CELL_VOTAGE_RANGE_MAX }, PG_BATTERY_CONFIG, offsetof(batteryConfig_t, vbatmincellvoltage) }, // vbat_min_cell_voltage
    { "2l",  VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { VBAT_CELL_VOTAGE_RANGE_MIN, VBAT_CELL_VOTAGE_RANGE_MAX }, PG_BATTERY_CONFIG, offsetof(batteryConfig_t, vbatwarningcellvoltage) }, // vbat_warning_cell_voltage
    { "2m",            VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 0, 250 }, PG_BATTERY_CONFIG, offsetof(batteryConfig_t, vbathysteresis) }, // vbat_hysteresis
    { "2n",              VAR_UINT8  | HARDWARE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_CURRENT_METER }, PG_BATTERY_CONFIG, offsetof(batteryConfig_t, currentMeterSource) }, // current_meter
    { "2o",              VAR_UINT8  | HARDWARE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_VOLTAGE_METER }, PG_BATTERY_CONFIG, offsetof(batteryConfig_t, voltageMeterSource) }, // battery_meter
    { "2p",   VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, 2000 }, PG_BATTERY_CONFIG, offsetof(batteryConfig_t, vbatnotpresentcellvoltage) }, // vbat_detect_cell_voltage
    { "2q",            VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_BATTERY_CONFIG, offsetof(batteryConfig_t, useVBatAlerts) }, // use_vbat_alerts
    { "2r",            VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_BATTERY_CONFIG, offsetof(batteryConfig_t, useConsumptionAlerts) }, // use_cbat_alerts
    { "2s",         VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 0, 100 }, PG_BATTERY_CONFIG, offsetof(batteryConfig_t, consumptionWarningPercentage) }, // cbat_alert_percent
    { "2t",        VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 0, 100 }, PG_BATTERY_CONFIG, offsetof(batteryConfig_t, lvcPercentage) }, // vbat_cutoff_percent
    { "2u",   VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 0, 24 }, PG_BATTERY_CONFIG, offsetof(batteryConfig_t, forceBatteryCellCount) }, // force_battery_cell_count
    { "2v",    VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 1, UINT8_MAX }, PG_BATTERY_CONFIG, offsetof(batteryConfig_t, vbatDisplayLpfPeriod) }, // vbat_display_lpf_period
#if defined(USE_BATTERY_VOLTAGE_SAG_COMPENSATION)
    { "2w",        VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 1, UINT8_MAX }, PG_BATTERY_CONFIG, offsetof(batteryConfig_t, vbatSagLpfPeriod) }, // vbat_sag_lpf_period
#endif
    { "2x",            VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 0, UINT8_MAX }, PG_BATTERY_CONFIG, offsetof(batteryConfig_t, ibatLpfPeriod) }, // ibat_lpf_period
    { "2y",  VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 0, 150 }, PG_BATTERY_CONFIG, offsetof(batteryConfig_t, vbatDurationForWarning) }, // vbat_duration_for_warning
    { "2z", VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 0, 150 }, PG_BATTERY_CONFIG, offsetof(batteryConfig_t, vbatDurationForCritical) }, // vbat_duration_for_critical

//  PG_VOLTAGE_SENSOR_ADC_CONFIG
    { "2A",                 VAR_UINT8  | HARDWARE_VALUE, .config.minmaxUnsigned = { VBAT_SCALE_MIN, VBAT_SCALE_MAX }, PG_VOLTAGE_SENSOR_ADC_CONFIG, offsetof(voltageSensorADCConfig_t, vbatscale) }, // vbat_scale
    { "2B",               VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { VBAT_DIVIDER_MIN, VBAT_DIVIDER_MAX }, PG_VOLTAGE_SENSOR_ADC_CONFIG, offsetof(voltageSensorADCConfig_t, vbatresdivval) }, // vbat_divider
    { "2C",            VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { VBAT_MULTIPLIER_MIN, VBAT_MULTIPLIER_MAX }, PG_VOLTAGE_SENSOR_ADC_CONFIG, offsetof(voltageSensorADCConfig_t, vbatresdivmultiplier) }, // vbat_multiplier

// PG_CURRENT_SENSOR_ADC_CONFIG
    { "2D",                VAR_INT16  | HARDWARE_VALUE, .config.minmax = { -16000, 16000 }, PG_CURRENT_SENSOR_ADC_CONFIG, offsetof(currentSensorADCConfig_t, scale) }, // ibata_scale
    { "2E",               VAR_INT16  | MASTER_VALUE, .config.minmax = { -32000, 32000 }, PG_CURRENT_SENSOR_ADC_CONFIG, offsetof(currentSensorADCConfig_t, offset) }, // ibata_offset
// PG_CURRENT_SENSOR_ADC_CONFIG
#ifdef USE_VIRTUAL_CURRENT_METER
    { "2F",                VAR_INT16  | MASTER_VALUE, .config.minmax = { -16000, 16000 }, PG_CURRENT_SENSOR_VIRTUAL_CONFIG, offsetof(currentSensorVirtualConfig_t, scale) }, // ibatv_scale
    { "2G",               VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 0, 16000 }, PG_CURRENT_SENSOR_VIRTUAL_CONFIG, offsetof(currentSensorVirtualConfig_t, offset) }, // ibatv_offset
#endif

#ifdef USE_BEEPER
// PG_BEEPER_DEV_CONFIG
    { "2H",           VAR_UINT8  | HARDWARE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_BEEPER_DEV_CONFIG, offsetof(beeperDevConfig_t, isInverted) }, // beeper_inversion
    { "2I",                  VAR_UINT8  | HARDWARE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_BEEPER_DEV_CONFIG, offsetof(beeperDevConfig_t, isOpenDrain) }, // beeper_od
    { "2J",           VAR_INT16  | HARDWARE_VALUE, .config.minmaxUnsigned = { 0, 16000 }, PG_BEEPER_DEV_CONFIG, offsetof(beeperDevConfig_t, frequency) }, // beeper_frequency

// PG_BEEPER_CONFIG
#ifdef USE_DSHOT
    { "2K",   VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = {1, DSHOT_CMD_BEACON5 }, PG_BEEPER_CONFIG, offsetof(beeperConfig_t, dshotBeaconTone) }, // beeper_dshot_beacon_tone
#endif
#endif // USE_BEEPER

// PG_MIXER_CONFIG
    { "2L",         VAR_INT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_MIXER_CONFIG, offsetof(mixerConfig_t, yaw_motors_reversed) }, // yaw_motors_reversed
    { "2M",     VAR_UINT8 |  MASTER_VALUE,  .config.minmaxUnsigned = { 0, 100 }, PG_MIXER_CONFIG, offsetof(mixerConfig_t, crashflip_motor_percent) }, // crashflip_motor_percent
    { "2N",              VAR_UINT8 |  MASTER_VALUE,  .config.minmaxUnsigned = { 0, 100 }, PG_MIXER_CONFIG, offsetof(mixerConfig_t, crashflip_expo) }, // crashflip_expo

// PG_MOTOR_3D_CONFIG
    { "2O",            VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { PWM_PULSE_MIN, PWM_RANGE_MIDDLE }, PG_MOTOR_3D_CONFIG, offsetof(flight3DConfig_t, deadband3d_low) }, // crashflip_expo
    { "2P",           VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { PWM_RANGE_MIDDLE, PWM_PULSE_MAX }, PG_MOTOR_3D_CONFIG, offsetof(flight3DConfig_t, deadband3d_high) }, // 3d_deadband_high
    { "2Q",                 VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { PWM_PULSE_MIN, PWM_PULSE_MAX }, PG_MOTOR_3D_CONFIG, offsetof(flight3DConfig_t, neutral3d) }, // 3d_neutral
    { "2R",       VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 1, 100 }, PG_MOTOR_3D_CONFIG, offsetof(flight3DConfig_t, deadband3d_throttle) }, // 3d_deadband_throttle
    { "2S",               VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { PWM_PULSE_MIN, PWM_RANGE_MIDDLE }, PG_MOTOR_3D_CONFIG, offsetof(flight3DConfig_t, limit3d_low) }, //3d_limit_low
    { "2T",              VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { PWM_RANGE_MIDDLE, PWM_PULSE_MAX }, PG_MOTOR_3D_CONFIG, offsetof(flight3DConfig_t, limit3d_high) }, // 3d_limit_high
    { "2U",           VAR_UINT8 | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_MOTOR_3D_CONFIG, offsetof(flight3DConfig_t, switched_mode3d) }, // 3d_switched_mode
    { "2V",            VAR_UINT8 | MASTER_VALUE, .config.minmaxUnsigned = { 0, 30 }, PG_MOTOR_3D_CONFIG, offsetof(flight3DConfig_t, reverse3dKick) }, // 3d_reverse_kick
    { "2W",       VAR_UINT8 | MASTER_VALUE, .config.minmaxUnsigned = { 0, 255 }, PG_MOTOR_3D_CONFIG, offsetof(flight3DConfig_t, reverse3dKickTime) }, // 3d_reverse_kick_time

// PG_SERVO_CONFIG
#ifdef USE_SERVOS
    { "2X",         VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { PWM_PULSE_MIN, PWM_PULSE_MAX }, PG_SERVO_CONFIG, offsetof(servoConfig_t, dev.servoCenterPulse) }, //servo_center_pulse
    { "2Y",             VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 50, 498 }, PG_SERVO_CONFIG, offsetof(servoConfig_t, dev.servoPwmRate) }, // servo_pwm_rate
    { "2Z",           VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 0, 400}, PG_SERVO_CONFIG, offsetof(servoConfig_t, servo_lowpass_freq) }, // servo_lowpass_hz
    { "30",          VAR_INT8   | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_SERVO_CONFIG, offsetof(servoConfig_t, tri_unarmed_servo) }, // tri_unarmed_servo
    { "31",   VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { AUX1, MAX_SUPPORTED_RC_CHANNEL_COUNT }, PG_SERVO_CONFIG, offsetof(servoConfig_t, channelForwardingStartChannel) }, // channel_forwarding_start
#endif

// PG_CONTROLRATE_PROFILES
#ifdef USE_PROFILE_NAMES
    { "32",           VAR_UINT8  | PROFILE_RATE_VALUE | MODE_STRING, .config.string = { 1, MAX_RATE_PROFILE_NAME_LENGTH, STRING_FLAGS_NONE }, PG_CONTROL_RATE_PROFILES, offsetof(controlRateConfig_t, profileName) }, // rateprofile_name
#endif
    { "33",                    VAR_UINT8  | PROFILE_RATE_VALUE, .config.minmaxUnsigned = { 0, 100 }, PG_CONTROL_RATE_PROFILES, offsetof(controlRateConfig_t, thrMid8) }, // thr_mid
    { "34",                   VAR_UINT8  | PROFILE_RATE_VALUE, .config.minmaxUnsigned = { 0, 100 }, PG_CONTROL_RATE_PROFILES, offsetof(controlRateConfig_t, thrExpo8) }, // thr_expo
    { "35",                 VAR_UINT8  | PROFILE_RATE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_RATES_TYPE }, PG_CONTROL_RATE_PROFILES, offsetof(controlRateConfig_t, rates_type) }, // rates_type
    { "36",               VAR_UINT8  | PROFILE_RATE_VALUE, .config.minmaxUnsigned = { 1, CONTROL_RATE_CONFIG_RC_RATES_MAX }, PG_CONTROL_RATE_PROFILES, offsetof(controlRateConfig_t, rcRates[FD_ROLL]) }, // roll_rc_rate
    { "37",              VAR_UINT8  | PROFILE_RATE_VALUE, .config.minmaxUnsigned = { 1, CONTROL_RATE_CONFIG_RC_RATES_MAX }, PG_CONTROL_RATE_PROFILES, offsetof(controlRateConfig_t, rcRates[FD_PITCH]) }, // pitch_rc_rate
    { "38",                VAR_UINT8  | PROFILE_RATE_VALUE, .config.minmaxUnsigned = { 1, CONTROL_RATE_CONFIG_RC_RATES_MAX }, PG_CONTROL_RATE_PROFILES, offsetof(controlRateConfig_t, rcRates[FD_YAW]) }, // yaw_rc_rate
    { "39",                  VAR_UINT8  | PROFILE_RATE_VALUE, .config.minmaxUnsigned = { 0, CONTROL_RATE_CONFIG_RC_EXPO_MAX }, PG_CONTROL_RATE_PROFILES, offsetof(controlRateConfig_t, rcExpo[FD_ROLL]) }, // roll_expo
    { "3a",                 VAR_UINT8  | PROFILE_RATE_VALUE, .config.minmaxUnsigned = { 0, CONTROL_RATE_CONFIG_RC_EXPO_MAX }, PG_CONTROL_RATE_PROFILES, offsetof(controlRateConfig_t, rcExpo[FD_PITCH]) }, // pitch_expo
    { "3b",                   VAR_UINT8  | PROFILE_RATE_VALUE, .config.minmaxUnsigned = { 0, CONTROL_RATE_CONFIG_RC_EXPO_MAX }, PG_CONTROL_RATE_PROFILES, offsetof(controlRateConfig_t, rcExpo[FD_YAW]) }, // yaw_expo
    { "3c",                 VAR_UINT8  | PROFILE_RATE_VALUE, .config.minmaxUnsigned = { 0, CONTROL_RATE_CONFIG_RATE_MAX }, PG_CONTROL_RATE_PROFILES, offsetof(controlRateConfig_t, rates[FD_ROLL]) }, // roll_srate
    { "3d",                VAR_UINT8  | PROFILE_RATE_VALUE, .config.minmaxUnsigned = { 0, CONTROL_RATE_CONFIG_RATE_MAX }, PG_CONTROL_RATE_PROFILES, offsetof(controlRateConfig_t, rates[FD_PITCH]) }, // pitch_srate
    { "3e",                  VAR_UINT8  | PROFILE_RATE_VALUE, .config.minmaxUnsigned = { 0, CONTROL_RATE_CONFIG_RATE_MAX }, PG_CONTROL_RATE_PROFILES, offsetof(controlRateConfig_t, rates[FD_YAW]) }, // yaw_srate
    { "3f",            VAR_UINT8  | PROFILE_RATE_VALUE, .config.minmaxUnsigned = { 0, 100 }, PG_CONTROL_RATE_PROFILES, offsetof(controlRateConfig_t, addRollToYawRc) }, // add_roll_to_yaw
    { "3g",            VAR_UINT8  | PROFILE_RATE_VALUE, .config.minmaxUnsigned = { 0, 100 }, PG_CONTROL_RATE_PROFILES, offsetof(controlRateConfig_t, addYawToRollRc) }, // add_yaw_to_roll
    { "3h",        VAR_UINT8  | PROFILE_RATE_VALUE, .config.minmaxUnsigned = { 0, 250 }, PG_CONTROL_RATE_PROFILES, offsetof(controlRateConfig_t, rollPitchMagExpo) }, // roll_pitch_mag_expo
    { "3i",                 VAR_UINT8  | PROFILE_VALUE, .config.minmaxUnsigned = { 0, CONTROL_RATE_CONFIG_TPA_MAX}, PG_PID_PROFILE, offsetof(pidProfile_t, dynThr[0]) }, // tpa_rate_p
    { "3j",                 VAR_UINT8  | PROFILE_VALUE, .config.minmaxUnsigned = { 0, CONTROL_RATE_CONFIG_TPA_MAX}, PG_PID_PROFILE, offsetof(pidProfile_t, dynThr[1]) }, // tpa_rate_i
    { "3k",                 VAR_UINT8  | PROFILE_VALUE, .config.minmaxUnsigned = { 0, CONTROL_RATE_CONFIG_TPA_MAX}, PG_PID_PROFILE, offsetof(pidProfile_t, dynThr[2]) }, // tpa_rate_d
    { "3l",             VAR_UINT16 | PROFILE_VALUE, .config.minmaxUnsigned = { PWM_PULSE_MIN, PWM_PULSE_MAX }, PG_PID_PROFILE, offsetof(pidProfile_t, tpa_breakpoint) }, // tpa_breakpoint
// rateDynamics
    { "3m",   VAR_UINT8  | PROFILE_RATE_VALUE, .config.minmaxUnsigned = { 25, 175 }, PG_CONTROL_RATE_PROFILES, offsetof(controlRateConfig_t, rateDynamics.rateSensCenter) }, // rate_center_sensitivity
    { "3n",      VAR_UINT8  | PROFILE_RATE_VALUE, .config.minmaxUnsigned = { 25, 175 }, PG_CONTROL_RATE_PROFILES, offsetof(controlRateConfig_t, rateDynamics.rateSensEnd) }, // rate_end_sensitivity
    { "3o",    VAR_UINT8  | PROFILE_RATE_VALUE, .config.minmaxUnsigned = { 0, 95 }, PG_CONTROL_RATE_PROFILES, offsetof(controlRateConfig_t, rateDynamics.rateCorrectionCenter) }, // rate_center_correction
    { "3p",       VAR_UINT8  | PROFILE_RATE_VALUE, .config.minmaxUnsigned = { 0, 95 }, PG_CONTROL_RATE_PROFILES, offsetof(controlRateConfig_t, rateDynamics.rateCorrectionEnd) }, // rate_end_correction
    { "3q",        VAR_UINT8  | PROFILE_RATE_VALUE, .config.minmaxUnsigned = { 0, 95 }, PG_CONTROL_RATE_PROFILES, offsetof(controlRateConfig_t, rateDynamics.rateWeightCenter) }, // rate_center_weight
    { "3r",           VAR_UINT8  | PROFILE_RATE_VALUE, .config.minmaxUnsigned = { 0, 95 }, PG_CONTROL_RATE_PROFILES, offsetof(controlRateConfig_t, rateDynamics.rateWeightEnd) }, // rate_end_weight

    { "3s",        VAR_UINT8  | PROFILE_RATE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_THROTTLE_LIMIT_TYPE }, PG_CONTROL_RATE_PROFILES, offsetof(controlRateConfig_t, throttle_limit_type) }, // throttle_limit_type
    { "3t",     VAR_UINT8  | PROFILE_RATE_VALUE, .config.minmaxUnsigned = { 25, 100 }, PG_CONTROL_RATE_PROFILES, offsetof(controlRateConfig_t, throttle_limit_percent) }, // throttle_limit_percent

// PG_SERIAL_CONFIG
    { "3u",           VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 48, 126 }, PG_SERIAL_CONFIG, offsetof(serialConfig_t, reboot_character) }, // reboot_character
    { "3v",      VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 100, 2000 }, PG_SERIAL_CONFIG, offsetof(serialConfig_t, serial_update_rate_hz) }, // serial_update_rate_hz

// PG_IMU_CONFIG
    { "3w",                 VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 0, 32000 }, PG_IMU_CONFIG, offsetof(imuConfig_t, dcm_kp) }, // imu_dcm_kp
    { "3x",                 VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 0, 32000 }, PG_IMU_CONFIG, offsetof(imuConfig_t, dcm_ki) }, // imu_dcm_ki
    { "3y",                VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 0, 180 }, PG_IMU_CONFIG, offsetof(imuConfig_t, small_angle) }, // small_angle
    { "3z",             VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_IMU_CONFIG, offsetof(imuConfig_t, level_recovery) }, // level_recovery
    { "3A",        VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 0, 10000 }, PG_IMU_CONFIG, offsetof(imuConfig_t, level_recovery_time) }, // level_recovery_time
    { "3B",        VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 0, 20 }, PG_IMU_CONFIG, offsetof(imuConfig_t, level_recovery_coef) }, // level_recovery_coef
    { "3C",   VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 0, 2000 }, PG_IMU_CONFIG, offsetof(imuConfig_t, level_recovery_threshold) }, // level_recovery_threshold

// PG_ARMING_CONFIG
    { "3D",          VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 0, 60 }, PG_ARMING_CONFIG, offsetof(armingConfig_t, auto_disarm_delay) }, // auto_disarm_delay
    { "3E",      VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_ARMING_CONFIG, offsetof(armingConfig_t, gyro_cal_on_first_arm) }, // gyro_cal_on_first_arm


// PG_GPS_CONFIG
#ifdef USE_GPS
    { "3F",               VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_GPS_PROVIDER }, PG_GPS_CONFIG, offsetof(gpsConfig_t, provider) }, // gps_provider
    { "3G",              VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_GPS_SBAS_MODE }, PG_GPS_CONFIG, offsetof(gpsConfig_t, sbasMode) }, // gps_sbas_mode
    { "3H",         VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_GPS_CONFIG, offsetof(gpsConfig_t, sbas_integrity) }, // gps_sbas_integrity
    { "3I",            VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_GPS_CONFIG, offsetof(gpsConfig_t, autoConfig) }, // gps_auto_config
    { "3J",              VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_GPS_CONFIG, offsetof(gpsConfig_t, autoBaud) }, // gps_auto_baud
    { "3K",      VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_GPS_CONFIG, offsetof(gpsConfig_t, gps_ublox_use_galileo) }, // gps_ublox_use_galileo
    { "3L",             VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_GPS_UBLOX_MODE }, PG_GPS_CONFIG, offsetof(gpsConfig_t, gps_ublox_mode) }, // gps_ublox_mode
    { "3M",    VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_GPS_CONFIG, offsetof(gpsConfig_t, gps_set_home_point_once) }, // gps_set_home_point_once
    { "3N",           VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_GPS_CONFIG, offsetof(gpsConfig_t, gps_use_3d_speed) }, // gps_use_3d_speed

#ifdef USE_GPS_RESCUE
    // PG_GPS_RESCUE
    { "3O",           VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 0, 200 }, PG_GPS_RESCUE, offsetof(gpsRescueConfig_t, angle) }, // gps_rescue_angle
    { "3P",      VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 0, 100 }, PG_GPS_RESCUE, offsetof(gpsRescueConfig_t, rescueAltitudeBufferM) }, // gps_rescue_alt_buffer
    { "3Q",     VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 20, 100 }, PG_GPS_RESCUE, offsetof(gpsRescueConfig_t, initialAltitudeM) }, // gps_rescue_initial_alt
    { "3R",    VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 30, 500 }, PG_GPS_RESCUE, offsetof(gpsRescueConfig_t, descentDistanceM) }, // gps_rescue_descent_dist
    { "3S",     VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 3, 10 }, PG_GPS_RESCUE, offsetof(gpsRescueConfig_t, targetLandingAltitudeM) }, // gps_rescue_landing_alt
    { "3T",    VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 5, 15 }, PG_GPS_RESCUE, offsetof(gpsRescueConfig_t, targetLandingDistanceM) }, // gps_rescue_landing_dist
    { "3U",    VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 30, 3000 }, PG_GPS_RESCUE, offsetof(gpsRescueConfig_t, rescueGroundspeed) }, // gps_rescue_ground_speed
    { "3V",      VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 0, 500 }, PG_GPS_RESCUE, offsetof(gpsRescueConfig_t, throttleP) }, // gps_rescue_throttle_p
    { "3W",      VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 0, 500 }, PG_GPS_RESCUE, offsetof(gpsRescueConfig_t, throttleI) }, // gps_rescue_throttle_i
    { "3X",      VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 0, 500 }, PG_GPS_RESCUE, offsetof(gpsRescueConfig_t, throttleD) }, // gps_rescue_throttle_d
    { "3Y",      VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 0, 500 }, PG_GPS_RESCUE, offsetof(gpsRescueConfig_t, velP) }, // gps_rescue_velocity_p
    { "3Z",      VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 0, 500 }, PG_GPS_RESCUE, offsetof(gpsRescueConfig_t, velI) }, // gps_rescue_velocity_i
    { "40",      VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 0, 500 }, PG_GPS_RESCUE, offsetof(gpsRescueConfig_t, velD) }, // gps_rescue_velocity_d
    { "41",           VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 0, 500 }, PG_GPS_RESCUE, offsetof(gpsRescueConfig_t, yawP) }, // gps_rescue_yaw_p

    { "42",    VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 1000, 2000 }, PG_GPS_RESCUE, offsetof(gpsRescueConfig_t, throttleMin) }, // gps_rescue_throttle_min
    { "43",    VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 1000, 2000 }, PG_GPS_RESCUE, offsetof(gpsRescueConfig_t, throttleMax) }, // gps_rescue_throttle_max
    { "44",     VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 100, 2500 }, PG_GPS_RESCUE, offsetof(gpsRescueConfig_t, ascendRate) }, // gps_rescue_ascend_rate
    { "45",    VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 100, 500 }, PG_GPS_RESCUE, offsetof(gpsRescueConfig_t, descendRate) }, // gps_rescue_descend_rate
    { "46",  VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 1000, 2000 }, PG_GPS_RESCUE, offsetof(gpsRescueConfig_t, throttleHover) }, // gps_rescue_throttle_hover
    { "47",   VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_GPS_RESCUE_SANITY_CHECK }, PG_GPS_RESCUE, offsetof(gpsRescueConfig_t, sanityChecks) }, // gps_rescue_sanity_checks
    { "48",        VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 5, 50 }, PG_GPS_RESCUE, offsetof(gpsRescueConfig_t, minSats) }, // gps_rescue_min_sats
    { "49",         VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 50, 1000 }, PG_GPS_RESCUE, offsetof(gpsRescueConfig_t, minRescueDth) }, // gps_rescue_min_dth
    { "4a", VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_GPS_RESCUE, offsetof(gpsRescueConfig_t, allowArmingWithoutFix) }, // gps_rescue_allow_arming_without_fix
    { "4b",        VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_GPS_RESCUE_ALT_MODE }, PG_GPS_RESCUE, offsetof(gpsRescueConfig_t, altitudeMode) }, // gps_rescue_alt_mode
#ifdef USE_MAG
    { "4c",         VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_GPS_RESCUE, offsetof(gpsRescueConfig_t, useMag) }, // gps_rescue_use_mag
#endif
#endif
#endif

    { "4d",                   VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 0, 32 }, PG_RC_CONTROLS_CONFIG, offsetof(rcControlsConfig_t, deadband) }, // deadband
    { "4e",               VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 0, 100 }, PG_RC_CONTROLS_CONFIG, offsetof(rcControlsConfig_t, yaw_deadband) }, // yaw_deadband
    { "4f",       VAR_INT8   | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_RC_CONTROLS_CONFIG, offsetof(rcControlsConfig_t, yaw_control_reversed) }, // yaw_control_reversed

// PG_PID_CONFIG
    { "4g",          VAR_UINT8  | MASTER_VALUE,  .config.minmaxUnsigned = { 1, MAX_PID_PROCESS_DENOM }, PG_PID_CONFIG, offsetof(pidConfig_t, pid_process_denom) }, // pid_process_denom
#ifdef USE_RUNAWAY_TAKEOFF
    { "4h", VAR_UINT8  | MODE_LOOKUP,  .config.lookup = { TABLE_OFF_ON }, PG_PID_CONFIG, offsetof(pidConfig_t, runaway_takeoff_prevention) }, // runaway_takeoff_prevention
    { "4i",  VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 100, 1000 }, PG_PID_CONFIG, offsetof(pidConfig_t, runaway_takeoff_deactivate_delay) }, // runaway_takeoff_deactivate_delay
    { "4j",  VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 0, 100 }, PG_PID_CONFIG, offsetof(pidConfig_t, runaway_takeoff_deactivate_throttle) }, // runaway_takeoff_deactivate_throttle_percent
#endif

// PG_PID_PROFILE
#ifdef USE_PROFILE_NAMES
    { "4k",               VAR_UINT8  | PROFILE_VALUE | MODE_STRING, .config.string = { 1, MAX_PROFILE_NAME_LENGTH, STRING_FLAGS_NONE }, PG_PID_PROFILE, offsetof(pidProfile_t, profileName) }, // profile_name
#endif
#ifdef USE_DYN_LPF
    { "4l",       VAR_UINT16 | PROFILE_VALUE, .config.minmaxUnsigned = { 0, 1000 }, PG_PID_PROFILE, offsetof(pidProfile_t, dyn_lpf_dterm_min_hz) }, // dyn_lpf_dterm_min_hz
    { "4m",        VAR_UINT8  | PROFILE_VALUE, .config.minmaxUnsigned = { 0, 255 }, PG_PID_PROFILE, offsetof(pidProfile_t, dyn_lpf_dterm_width) }, // dyn_lpf_dterm_width
    { "4n",   VAR_UINT8  | PROFILE_VALUE, .config.minmaxUnsigned = { 0, 10 }, PG_PID_PROFILE, offsetof(pidProfile_t, dyn_lpf_curve_expo) }, // dyn_lpf_dterm_curve_expo
    { "4o",        VAR_UINT8  | MASTER_VALUE,  .config.minmaxUnsigned = { 0,  200 },    PG_PID_PROFILE, offsetof(pidProfile_t, dyn_lpf_dterm_gain) }, // dyn_lpf2_dterm_gain
#endif
    { "4p",         VAR_UINT8  | PROFILE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_LOWPASS_TYPE }, PG_PID_PROFILE, offsetof(pidProfile_t, dterm_filter_type) }, // dterm_lowpass_type
    { "4q",        VAR_UINT8  | PROFILE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_LOWPASS_TYPE }, PG_PID_PROFILE, offsetof(pidProfile_t, dterm_filter2_type) }, // dterm_lowpass2_type
    { "4r",          VAR_UINT16 | PROFILE_VALUE, .config.minmaxUnsigned = { 0, FILTER_FREQUENCY_MAX }, PG_PID_PROFILE, offsetof(pidProfile_t, dterm_lowpass2_hz) }, // dterm_lowpass2_hz
    { "4s",             VAR_UINT16 | PROFILE_VALUE, .config.minmaxUnsigned = { 0, FILTER_FREQUENCY_MAX }, PG_PID_PROFILE, offsetof(pidProfile_t, dterm_notch_hz) }, // dterm_notch_hz
    { "4t",         VAR_UINT16 | PROFILE_VALUE, .config.minmaxUnsigned = { 0, FILTER_FREQUENCY_MAX }, PG_PID_PROFILE, offsetof(pidProfile_t, dterm_notch_cutoff) }, // dterm_notch_cutoff
    { "4u",            VAR_UINT16 | PROFILE_VALUE, .config.minmaxUnsigned = { 0, 1000 }, PG_PID_PROFILE, offsetof(pidProfile_t, dtermAlpha) }, // dterm_abg_alpha
    { "4v",            VAR_UINT16 | PROFILE_VALUE, .config.minmaxUnsigned = { 0, 2000 }, PG_PID_PROFILE, offsetof(pidProfile_t, dterm_abg_boost) }, // dterm_abg_boost
    { "4w",        VAR_UINT16 | PROFILE_VALUE, .config.minmaxUnsigned = { 0, 1000 }, PG_PID_PROFILE, offsetof(pidProfile_t, dterm_abg_half_life) }, // dterm_abg_half_life
#if defined(USE_BATTERY_VOLTAGE_SAG_COMPENSATION)
    { "4x",      VAR_UINT8  | PROFILE_VALUE, .config.minmaxUnsigned = { 0, 150 }, PG_PID_PROFILE, offsetof(pidProfile_t, vbat_sag_compensation) }, // vbat_sag_compensation
#endif
    { "4y",        VAR_UINT8  | PROFILE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_PID_PROFILE, offsetof(pidProfile_t, pidAtMinThrottle) }, // pid_at_min_throttle
    { "4z",          VAR_UINT8  | PROFILE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_ANTI_GRAVITY_MODE }, PG_PID_PROFILE, offsetof(pidProfile_t, antiGravityMode) }, // anti_gravity_mode
    { "4A",     VAR_UINT16 | PROFILE_VALUE, .config.minmaxUnsigned = { 20, 1000 }, PG_PID_PROFILE, offsetof(pidProfile_t, itermThrottleThreshold) }, // anti_gravity_threshold
    { "4B",          VAR_UINT16 | PROFILE_VALUE, .config.minmaxUnsigned = { ITERM_ACCELERATOR_GAIN_OFF, ITERM_ACCELERATOR_GAIN_MAX }, PG_PID_PROFILE, offsetof(pidProfile_t, itermAcceleratorGain) }, // anti_gravity_gain
    { "4C",     VAR_UINT8  | PROFILE_VALUE, .config.minmaxUnsigned = { 0, 100 }, PG_PID_PROFILE, offsetof(pidProfile_t, feedForwardTransition) }, // feedforward_transition
    { "4D",           VAR_UINT16 | PROFILE_VALUE, .config.minmaxUnsigned = { 10, 2000 }, PG_PID_PROFILE, offsetof(pidProfile_t, crash_dthreshold) }, // crash_dthreshold
    { "4E",           VAR_UINT16 | PROFILE_VALUE, .config.minmaxUnsigned = { 100, 2000 }, PG_PID_PROFILE, offsetof(pidProfile_t, crash_gthreshold) }, // crash_gthreshold
    { "4F",   VAR_UINT16 | PROFILE_VALUE, .config.minmaxUnsigned = { 50, 2000 }, PG_PID_PROFILE, offsetof(pidProfile_t, crash_setpoint_threshold) }, // crash_setpoint_threshold
    { "4G",             VAR_UINT8  | PROFILE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_CRASH_RECOVERY }, PG_PID_PROFILE, offsetof(pidProfile_t, crash_recovery) }, // crash_recovery
    { "4H",             VAR_UINT8  | PROFILE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_PID_PROFILE, offsetof(pidProfile_t, iterm_rotation) }, // iterm_rotation
#if defined(USE_ITERM_RELAX)
    { "4I",         VAR_UINT8  | PROFILE_VALUE, .config.minmaxUnsigned = { 0, 50 }, PG_PID_PROFILE, offsetof(pidProfile_t, iterm_relax_cutoff) }, // iterm_relax_cutoff
    { "4J",     VAR_UINT8  | PROFILE_VALUE, .config.minmaxUnsigned = { 0, 50 }, PG_PID_PROFILE, offsetof(pidProfile_t, iterm_relax_cutoff_yaw) }, // iterm_relax_cutoff_yaw
    { "4K",      VAR_UINT8  | PROFILE_VALUE, .config.minmaxUnsigned = { 10, 100 }, PG_PID_PROFILE, offsetof(pidProfile_t, iterm_relax_threshold) }, // iterm_relax_threshold
    { "4L",  VAR_UINT8  | PROFILE_VALUE, .config.minmaxUnsigned = { 10, 100 }, PG_PID_PROFILE, offsetof(pidProfile_t, iterm_relax_threshold_yaw) }, // iterm_relax_threshold_yaw
#endif
    { "4M",               VAR_UINT8  | PROFILE_VALUE, .config.minmaxUnsigned = { 0, 100 }, PG_PID_PROFILE, offsetof(pidProfile_t, itermWindupPointPercent) }, // iterm_windup
    { "4N",                VAR_UINT16 | PROFILE_VALUE, .config.minmaxUnsigned = { 0, 500 }, PG_PID_PROFILE, offsetof(pidProfile_t, itermLimit) }, // iterm_limit
    { "4O",               VAR_UINT16 | PROFILE_VALUE, .config.minmaxUnsigned = { PIDSUM_LIMIT_MIN, PIDSUM_LIMIT_MAX }, PG_PID_PROFILE, offsetof(pidProfile_t, pidSumLimit) }, // pidsum_limit
    { "4P",           VAR_UINT16 | PROFILE_VALUE, .config.minmaxUnsigned = { PIDSUM_LIMIT_MIN, PIDSUM_LIMIT_MAX }, PG_PID_PROFILE, offsetof(pidProfile_t, pidSumLimitYaw) }, // pidsum_limit_yaw
    { "4Q",             VAR_UINT16 | PROFILE_VALUE, .config.minmaxUnsigned = { 0, 500 }, PG_PID_PROFILE, offsetof(pidProfile_t, yaw_lowpass_hz) }, // yaw_lowpass_hz

#if defined(USE_THROTTLE_BOOST)
    { "4R",             VAR_UINT8 | PROFILE_VALUE,  .config.minmaxUnsigned = { 0, 100 }, PG_PID_PROFILE, offsetof(pidProfile_t, throttle_boost) }, // throttle_boost
    { "4S",      VAR_UINT8 | PROFILE_VALUE,  .config.minmaxUnsigned = { 5, 50 }, PG_PID_PROFILE, offsetof(pidProfile_t, throttle_boost_cutoff) }, // throttle_boost_cutoff
#endif

    { "4T",       VAR_UINT8  | PROFILE_VALUE, .config.minmaxUnsigned = { 0, 100 }, PG_PID_PROFILE, offsetof(pidProfile_t, dtermMeasurementSlider) }, // d_measurement_slider

    { "4U",                   VAR_UINT16 | PROFILE_VALUE, .config.minmaxUnsigned = { 0, 1000 },PG_PID_PROFILE, offsetof(pidProfile_t, emuBoostPR) }, // emuboost
    { "4V",               VAR_UINT16 | PROFILE_VALUE, .config.minmaxUnsigned = { 0, 1000 },PG_PID_PROFILE, offsetof(pidProfile_t, emuBoostY) }, // emuboost_yaw
    { "4W",                VAR_UINT16 | PROFILE_VALUE, .config.minmaxUnsigned = { 0, 1000 },PG_PID_PROFILE, offsetof(pidProfile_t, dtermBoost) }, // dterm_boost

    { "4X",                    VAR_UINT8  | PROFILE_VALUE, .config.minmaxUnsigned = { 1, 10 }, PG_PID_PROFILE, offsetof(pidProfile_t, i_decay) }, // i_decay
    { "4Y",             VAR_UINT8  | PROFILE_VALUE, .config.minmaxUnsigned = { 1, 250 }, PG_PID_PROFILE, offsetof(pidProfile_t, i_decay_cutoff) }, // i_decay_cutoff

    { "4Z",       VAR_UINT8 | PROFILE_VALUE, .config.minmaxUnsigned = { 0, 50 }, PG_PID_PROFILE, offsetof(pidProfile_t, axis_lock_multiplier) }, // axis_lock_multiplier
    { "50",               VAR_UINT8 | PROFILE_VALUE, .config.minmaxUnsigned = { 1, 50 }, PG_PID_PROFILE, offsetof(pidProfile_t, axis_lock_hz) }, // axis_lock_hz

    { "51",                 VAR_UINT8  | PROFILE_VALUE, .config.minmaxUnsigned = { 0, 250}, PG_PID_PROFILE, offsetof(pidProfile_t, stickTransition[0][ROLL]) }, // spa_roll_p
    { "52",                 VAR_UINT8  | PROFILE_VALUE, .config.minmaxUnsigned = { 0, 250}, PG_PID_PROFILE, offsetof(pidProfile_t, stickTransition[1][ROLL]) }, // spa_roll_i
    { "53",                 VAR_UINT8  | PROFILE_VALUE, .config.minmaxUnsigned = { 0, 250}, PG_PID_PROFILE, offsetof(pidProfile_t, stickTransition[2][ROLL]) }, // spa_roll_d
    { "54",                VAR_UINT8  | PROFILE_VALUE, .config.minmaxUnsigned = { 0, 250}, PG_PID_PROFILE, offsetof(pidProfile_t, stickTransition[0][PITCH]) }, // spa_pitch_p
    { "55",                VAR_UINT8  | PROFILE_VALUE, .config.minmaxUnsigned = { 0, 250}, PG_PID_PROFILE, offsetof(pidProfile_t, stickTransition[1][PITCH]) }, // spa_pitch_i
    { "56",                VAR_UINT8  | PROFILE_VALUE, .config.minmaxUnsigned = { 0, 250}, PG_PID_PROFILE, offsetof(pidProfile_t, stickTransition[2][PITCH]) }, // spa_pitch_d
    { "57",                  VAR_UINT8  | PROFILE_VALUE, .config.minmaxUnsigned = { 0, 250}, PG_PID_PROFILE, offsetof(pidProfile_t, stickTransition[0][YAW]) }, // spa_yaw_p
    { "58",                  VAR_UINT8  | PROFILE_VALUE, .config.minmaxUnsigned = { 0, 250}, PG_PID_PROFILE, offsetof(pidProfile_t, stickTransition[1][YAW]) }, // spa_yaw_i
    { "59",                  VAR_UINT8  | PROFILE_VALUE, .config.minmaxUnsigned = { 0, 250}, PG_PID_PROFILE, offsetof(pidProfile_t, stickTransition[2][YAW]) }, // spa_yaw_d

    { "5a",                    VAR_UINT8  | PROFILE_VALUE, .config.minmaxUnsigned = { 0, 255 }, PG_PID_PROFILE, offsetof(pidProfile_t, pid[PID_PITCH].P) }, // p_pitch
    { "5b",                    VAR_UINT8  | PROFILE_VALUE, .config.minmaxUnsigned = { 0, 255 }, PG_PID_PROFILE, offsetof(pidProfile_t, pid[PID_PITCH].I) }, // i_pitch
    { "5c",                    VAR_UINT8  | PROFILE_VALUE, .config.minmaxUnsigned = { 0, 255 }, PG_PID_PROFILE, offsetof(pidProfile_t, pid[PID_PITCH].D) }, // d_pitch
    { "5d",                    VAR_UINT16 | PROFILE_VALUE, .config.minmaxUnsigned = { 0, 2000 },PG_PID_PROFILE, offsetof(pidProfile_t, pid[PID_PITCH].F) }, // f_pitch
    { "5e",                     VAR_UINT8  | PROFILE_VALUE, .config.minmaxUnsigned = { 0, 255 }, PG_PID_PROFILE, offsetof(pidProfile_t, pid[PID_ROLL].P) }, // p_roll
    { "5f",                     VAR_UINT8  | PROFILE_VALUE, .config.minmaxUnsigned = { 0, 255 }, PG_PID_PROFILE, offsetof(pidProfile_t, pid[PID_ROLL].I) }, // i_roll
    { "5g",                     VAR_UINT8  | PROFILE_VALUE, .config.minmaxUnsigned = { 0, 255 }, PG_PID_PROFILE, offsetof(pidProfile_t, pid[PID_ROLL].D) }, // d_roll
    { "5h",                     VAR_UINT16 | PROFILE_VALUE, .config.minmaxUnsigned = { 0, 2000 },PG_PID_PROFILE, offsetof(pidProfile_t, pid[PID_ROLL].F) }, // f_roll
    { "5i",                      VAR_UINT8  | PROFILE_VALUE, .config.minmaxUnsigned = { 0, 255 }, PG_PID_PROFILE, offsetof(pidProfile_t, pid[PID_YAW].P) }, // p_yaw
    { "5j",                      VAR_UINT8  | PROFILE_VALUE, .config.minmaxUnsigned = { 0, 255 }, PG_PID_PROFILE, offsetof(pidProfile_t, pid[PID_YAW].I) }, // i_yaw
    { "5k",                      VAR_UINT8  | PROFILE_VALUE, .config.minmaxUnsigned = { 0, 255 }, PG_PID_PROFILE, offsetof(pidProfile_t, pid[PID_YAW].D) }, // d_yaw
    { "5l",                      VAR_UINT16 | PROFILE_VALUE, .config.minmaxUnsigned = { 0, 2000 },PG_PID_PROFILE, offsetof(pidProfile_t, pid[PID_YAW].F) }, // f_yaw
    { "5m",               VAR_UINT8  | PROFILE_VALUE, .config.minmaxUnsigned = { 0, 255 }, PG_PID_PROFILE, offsetof(pidProfile_t, pid[PID_YAW].DF) }, // direct_f_yaw

    { "5n",                VAR_UINT8  | PROFILE_VALUE, .config.minmaxUnsigned = { 0, 200 }, PG_PID_PROFILE, offsetof(pidProfile_t, pid[PID_LEVEL_LOW].P) }, // p_angle_low
    { "5o",                VAR_UINT8  | PROFILE_VALUE, .config.minmaxUnsigned = { 0, 200 }, PG_PID_PROFILE, offsetof(pidProfile_t, pid[PID_LEVEL_LOW].D) }, // d_angle_low
    { "5p",               VAR_UINT8  | PROFILE_VALUE, .config.minmaxUnsigned = { 0, 200 }, PG_PID_PROFILE, offsetof(pidProfile_t, pid[PID_LEVEL_LOW].DF) }, // df_angle_low
    { "5q",               VAR_UINT8  | PROFILE_VALUE, .config.minmaxUnsigned = { 0, 200 }, PG_PID_PROFILE, offsetof(pidProfile_t, pid[PID_LEVEL_HIGH].P) }, // p_angle_high
    { "5r",               VAR_UINT8  | PROFILE_VALUE, .config.minmaxUnsigned = { 0, 200 }, PG_PID_PROFILE, offsetof(pidProfile_t, pid[PID_LEVEL_HIGH].D) }, // d_angle_high
    { "5s",              VAR_UINT8  | PROFILE_VALUE, .config.minmaxUnsigned = { 0, 200 }, PG_PID_PROFILE, offsetof(pidProfile_t, pid[PID_LEVEL_HIGH].DF) }, // df_angle_high
    { "5t",                    VAR_UINT16 | PROFILE_VALUE, .config.minmaxUnsigned = { 0, 2000 }, PG_PID_PROFILE, offsetof(pidProfile_t, pid[PID_LEVEL_LOW].F) }, // f_angle

    { "5u",                VAR_UINT8  | PROFILE_VALUE, .config.minmaxUnsigned = { 10, 90 }, PG_PID_PROFILE, offsetof(pidProfile_t, levelAngleLimit) }, // angle_limit
    { "5v",                 VAR_UINT8  | PROFILE_VALUE, .config.minmaxUnsigned = { 0, 100 }, PG_PID_PROFILE, offsetof(pidProfile_t, angleExpo) }, // angle_expo

    { "5w",         VAR_UINT8  | PROFILE_VALUE, .config.minmaxUnsigned = { 0,  250 }, PG_PID_PROFILE, offsetof(pidProfile_t, horizonTransition) }, // horizon_transition
    { "5x",        VAR_UINT8  | PROFILE_VALUE, .config.minmaxUnsigned = { 0,  180 }, PG_PID_PROFILE, offsetof(pidProfile_t, horizon_tilt_effect) }, // horizon_tilt_effect
    { "5y",               VAR_UINT8  | PROFILE_VALUE, .config.minmaxUnsigned = { 0,  250 }, PG_PID_PROFILE, offsetof(pidProfile_t, horizon_strength) }, // horizon_strength

    { "5z",         VAR_UINT8 | PROFILE_VALUE,  .config.minmaxUnsigned = { MOTOR_OUTPUT_LIMIT_PERCENT_MIN, MOTOR_OUTPUT_LIMIT_PERCENT_MAX }, PG_PID_PROFILE, offsetof(pidProfile_t, motor_output_limit) }, // motor_output_limit
    { "5A",    VAR_INT8 | PROFILE_VALUE,  .config.minmax = { AUTO_PROFILE_CELL_COUNT_CHANGE, MAX_AUTO_DETECT_CELL_COUNT }, PG_PID_PROFILE, offsetof(pidProfile_t, auto_profile_cell_count) }, // auto_profile_cell_count

#ifdef USE_LAUNCH_CONTROL
    { "5B",        VAR_UINT8  | PROFILE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_LAUNCH_CONTROL_MODE }, PG_PID_PROFILE, offsetof(pidProfile_t, launchControlMode) }, // launch_control_mode
    { "5C", VAR_UINT8  | PROFILE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_PID_PROFILE, offsetof(pidProfile_t, launchControlAllowTriggerReset) }, // launch_trigger_allow_reset
    { "5D", VAR_UINT8 | PROFILE_VALUE,  .config.minmaxUnsigned = { 0, LAUNCH_CONTROL_THROTTLE_TRIGGER_MAX }, PG_PID_PROFILE, offsetof(pidProfile_t, launchControlThrottlePercent) }, // launch_trigger_throttle_percent
    { "5E",         VAR_UINT8 | PROFILE_VALUE,  .config.minmaxUnsigned = { 0, 80 }, PG_PID_PROFILE, offsetof(pidProfile_t, launchControlAngleLimit) }, // launch_angle_limit
    { "5F",        VAR_UINT8 | PROFILE_VALUE,  .config.minmaxUnsigned = { 0, 200 }, PG_PID_PROFILE, offsetof(pidProfile_t, launchControlGain) }, // launch_control_gain
#endif

#ifdef USE_THRUST_LINEARIZATION
    { "5G",              VAR_UINT8 | MASTER_VALUE, .config.minmaxUnsigned = { 0, 150 }, PG_PID_PROFILE, offsetof(pidProfile_t, thrustLinearization) }, // thrust_linear
#endif
#ifdef USE_INTERPOLATED_SP
    { "5H",          VAR_UINT8 | PROFILE_VALUE | MODE_LOOKUP, .config.lookup = {TABLE_INTERPOLATED_SP}, PG_PID_PROFILE, offsetof(pidProfile_t, ff_interpolate_sp) }, // ff_interpolate_sp
    { "5I",          VAR_UINT8 | PROFILE_VALUE, .config.minmaxUnsigned = {0, 150}, PG_PID_PROFILE, offsetof(pidProfile_t, ff_max_rate_limit) }, // ff_max_rate_limit
    { "5J",           VAR_UINT8 | PROFILE_VALUE, .config.minmaxUnsigned = {0, 75}, PG_PID_PROFILE, offsetof(pidProfile_t, ff_smooth_factor) }, // ff_smooth_factor
#endif
    { "5K",                   VAR_UINT8 | PROFILE_VALUE,  .config.minmaxUnsigned = { 0, 50 }, PG_PID_PROFILE, offsetof(pidProfile_t, ff_boost) }, // ff_boost

#ifdef USE_DYN_IDLE
    { "5L",           VAR_UINT8 | PROFILE_VALUE, .config.minmaxUnsigned = { 0, 100 }, PG_PID_PROFILE, offsetof(pidProfile_t, dyn_idle_min_rpm) }, // dyn_idle_min_rpm
    { "5M",            VAR_UINT8 | PROFILE_VALUE, .config.minmaxUnsigned = { 1, 250 }, PG_PID_PROFILE, offsetof(pidProfile_t, dyn_idle_p_gain) }, // dyn_idle_p_gain
    { "5N",            VAR_UINT8 | PROFILE_VALUE, .config.minmaxUnsigned = { 1, 250 }, PG_PID_PROFILE, offsetof(pidProfile_t, dyn_idle_i_gain) }, // dyn_idle_i_gain
    { "5O",            VAR_UINT8 | PROFILE_VALUE, .config.minmaxUnsigned = { 0, 250 }, PG_PID_PROFILE, offsetof(pidProfile_t, dyn_idle_d_gain) }, // dyn_idle_d_gain
    { "5P",      VAR_UINT8 | PROFILE_VALUE, .config.minmaxUnsigned = { 10, 255 }, PG_PID_PROFILE, offsetof(pidProfile_t, dyn_idle_max_increase) }, // dyn_idle_max_increase
#endif

// PG_TELEMETRY_CONFIG
#ifdef USE_TELEMETRY
    { "5Q",               VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_TELEMETRY_CONFIG, offsetof(telemetryConfig_t, telemetry_inverted) }, // tlm_inverted
    { "5R",             VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_TELEMETRY_CONFIG, offsetof(telemetryConfig_t, halfDuplex) }, // tlm_halfduplex
#if defined(USE_TELEMETRY_FRSKY_HUB)
#if defined(USE_GPS)
    { "5S",          VAR_INT16  | MASTER_VALUE, .config.minmax = { -9000, 9000 }, PG_TELEMETRY_CONFIG, offsetof(telemetryConfig_t, gpsNoFixLatitude) }, // frsky_default_lat
    { "5T",         VAR_INT16  | MASTER_VALUE, .config.minmax = { -18000, 18000 }, PG_TELEMETRY_CONFIG, offsetof(telemetryConfig_t, gpsNoFixLongitude) }, // frsky_default_long
    { "5U",           VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 0, FRSKY_FORMAT_NMEA }, PG_TELEMETRY_CONFIG, offsetof(telemetryConfig_t, frsky_coordinate_format) }, // frsky_gps_format
    { "5V",                 VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_UNIT }, PG_TELEMETRY_CONFIG, offsetof(telemetryConfig_t, frsky_unit) }, // frsky_unit
#endif
    { "5W",       VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { FRSKY_VFAS_PRECISION_LOW,  FRSKY_VFAS_PRECISION_HIGH }, PG_TELEMETRY_CONFIG, offsetof(telemetryConfig_t, frsky_vfas_precision) }, // frsky_vfas_precision
#endif // USE_TELEMETRY_FRSKY_HUB
    { "5X",             VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 0, 120 }, PG_TELEMETRY_CONFIG, offsetof(telemetryConfig_t, hottAlarmSoundInterval) }, // hott_alarm_int
    { "5Y",                 VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = {TABLE_OFF_ON }, PG_TELEMETRY_CONFIG, offsetof(telemetryConfig_t, pidValuesAsTelemetry) }, // pid_in_tlm
    { "5Z",        VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_TELEMETRY_CONFIG, offsetof(telemetryConfig_t, report_cell_voltage) }, // report_cell_voltage
#if defined(USE_TELEMETRY_IBUS)
    { "60",                VAR_UINT8  | MASTER_VALUE | MODE_ARRAY, .config.array.length = IBUS_SENSOR_COUNT, PG_TELEMETRY_CONFIG, offsetof(telemetryConfig_t, flysky_sensors)}, // ibus_sensor
#endif
#ifdef USE_TELEMETRY_MAVLINK
    // Support for misusing the heading field in MAVlink to indicate mAh drawn for Connex Prosight OSD
    // Set to 10 to show a tenth of your capacity drawn.
    // Set to $size_of_battery to get a percentage of battery used.
    { "61", VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 0, 30000 }, PG_TELEMETRY_CONFIG, offsetof(telemetryConfig_t, mavlink_mah_as_heading_divisor) }, // mavlink_mah_as_heading_divisor
#endif
#ifdef USE_TELEMETRY_SENSORS_DISABLED_DETAILS
    { "62",         VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = LOG2(SENSOR_VOLTAGE),         PG_TELEMETRY_CONFIG, offsetof(telemetryConfig_t, disabledSensors)}, // telemetry_disabled_voltage
    { "63",         VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = LOG2(SENSOR_CURRENT),         PG_TELEMETRY_CONFIG, offsetof(telemetryConfig_t, disabledSensors)}, // telemetry_disabled_current
    { "64",            VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = LOG2(SENSOR_FUEL),            PG_TELEMETRY_CONFIG, offsetof(telemetryConfig_t, disabledSensors)}, // telemetry_disabled_fuel
    { "65",            VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = LOG2(SENSOR_MODE),            PG_TELEMETRY_CONFIG, offsetof(telemetryConfig_t, disabledSensors)}, // telemetry_disabled_mode
    { "66",           VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = LOG2(SENSOR_ACC_X),           PG_TELEMETRY_CONFIG, offsetof(telemetryConfig_t, disabledSensors)}, // telemetry_disabled_acc_x
    { "67",           VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = LOG2(SENSOR_ACC_Y),           PG_TELEMETRY_CONFIG, offsetof(telemetryConfig_t, disabledSensors)}, // telemetry_disabled_acc_y
    { "68",           VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = LOG2(SENSOR_ACC_Z),           PG_TELEMETRY_CONFIG, offsetof(telemetryConfig_t, disabledSensors)}, // telemetry_disabled_acc_z
    { "69",           VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = LOG2(SENSOR_PITCH),           PG_TELEMETRY_CONFIG, offsetof(telemetryConfig_t, disabledSensors)}, // telemetry_disabled_pitch
    { "7a",            VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = LOG2(SENSOR_ROLL),            PG_TELEMETRY_CONFIG, offsetof(telemetryConfig_t, disabledSensors)}, // telemetry_disabled_roll
    { "7b",         VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = LOG2(SENSOR_HEADING),         PG_TELEMETRY_CONFIG, offsetof(telemetryConfig_t, disabledSensors)}, // telemetry_disabled_heading
    { "7c",        VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = LOG2(SENSOR_ALTITUDE),        PG_TELEMETRY_CONFIG, offsetof(telemetryConfig_t, disabledSensors)}, // telemetry_disabled_altitude
    { "7d",           VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = LOG2(SENSOR_VARIO),           PG_TELEMETRY_CONFIG, offsetof(telemetryConfig_t, disabledSensors)}, // telemetry_disabled_vario
    { "7e",        VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = LOG2(SENSOR_LAT_LONG),        PG_TELEMETRY_CONFIG, offsetof(telemetryConfig_t, disabledSensors)}, // telemetry_disabled_lat_long
    { "7f",    VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = LOG2(SENSOR_GROUND_SPEED),    PG_TELEMETRY_CONFIG, offsetof(telemetryConfig_t, disabledSensors)}, // telemetry_disabled_ground_speed
    { "7g",        VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = LOG2(SENSOR_DISTANCE),        PG_TELEMETRY_CONFIG, offsetof(telemetryConfig_t, disabledSensors)}, // telemetry_disabled_distance
    { "7h",     VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = LOG2(ESC_SENSOR_CURRENT),     PG_TELEMETRY_CONFIG, offsetof(telemetryConfig_t, disabledSensors)}, // telemetry_disabled_esc_current
    { "7i",     VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = LOG2(ESC_SENSOR_VOLTAGE),     PG_TELEMETRY_CONFIG, offsetof(telemetryConfig_t, disabledSensors)}, // telemetry_disabled_esc_voltage
    { "7j",         VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = LOG2(ESC_SENSOR_RPM),         PG_TELEMETRY_CONFIG, offsetof(telemetryConfig_t, disabledSensors)}, // telemetry_disabled_esc_rpm
    { "7k", VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = LOG2(ESC_SENSOR_TEMPERATURE), PG_TELEMETRY_CONFIG, offsetof(telemetryConfig_t, disabledSensors)}, // telemetry_disabled_esc_temperature
    { "7l",     VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = LOG2(SENSOR_TEMPERATURE),     PG_TELEMETRY_CONFIG, offsetof(telemetryConfig_t, disabledSensors)}, // telemetry_disabled_temperature
    { "7m",        VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = LOG2(SENSOR_CAP_USED),        PG_TELEMETRY_CONFIG, offsetof(telemetryConfig_t, disabledSensors)}, // telemetry_disabled_cap_used
#else
    { "7n", VAR_UINT32 | MASTER_VALUE, .config.u32Max = SENSOR_ALL, PG_TELEMETRY_CONFIG, offsetof(telemetryConfig_t, disabledSensors)}, // telemetry_disabled_sensors
#endif
#endif // USE_TELEMETRY

// PG_LED_STRIP_CONFIG
#ifdef USE_LED_STRIP
    { "7o",     VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_LED_STRIP_CONFIG, offsetof(ledStripConfig_t, ledstrip_visual_beeper) }, // ledstrip_visual_beeper
    { "7p",VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_LEDSTRIP_COLOR }, PG_LED_STRIP_CONFIG, offsetof(ledStripConfig_t, ledstrip_visual_beeper_color) }, // ledstrip_visual_beeper_color
    { "7q",           VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_RGB_GRB }, PG_LED_STRIP_CONFIG, offsetof(ledStripConfig_t, ledstrip_grb_rgb) }, // ledstrip_grb_rgb
    { "7r",           VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_LED_PROFILE }, PG_LED_STRIP_CONFIG, offsetof(ledStripConfig_t, ledstrip_profile) }, // ledstrip_profile
    { "7s",        VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_LEDSTRIP_COLOR }, PG_LED_STRIP_CONFIG, offsetof(ledStripConfig_t, ledstrip_race_color) }, // ledstrip_race_color
    { "7t",      VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_LEDSTRIP_COLOR }, PG_LED_STRIP_CONFIG, offsetof(ledStripConfig_t, ledstrip_beacon_color) }, // ledstrip_beacon_color
    { "7u",  VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 50, 10000 }, PG_LED_STRIP_CONFIG, offsetof(ledStripConfig_t, ledstrip_beacon_period_ms) }, // ledstrip_beacon_period_ms
    { "7v",    VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 0, 100 }, PG_LED_STRIP_CONFIG, offsetof(ledStripConfig_t, ledstrip_beacon_percent) }, // ledstrip_beacon_percent
    { "7w", VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_LED_STRIP_CONFIG, offsetof(ledStripConfig_t, ledstrip_beacon_armed_only) }, // ledstrip_beacon_armed_only
#endif

// PG_SDCARD_CONFIG
#ifdef USE_SDCARD
    { "7x",     VAR_UINT8  | HARDWARE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_SDCARD_CONFIG, offsetof(sdcardConfig_t, cardDetectInverted) }, // sdcard_detect_inverted
    { "7y",                VAR_UINT8  | HARDWARE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_SDCARD_MODE }, PG_SDCARD_CONFIG, offsetof(sdcardConfig_t, mode) }, // sdcard_mode
#endif
#ifdef USE_SDCARD_SPI
    { "7z",                 VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_SDCARD_CONFIG, offsetof(sdcardConfig_t, useDma) }, // sdcard_dma
    { "7A",             VAR_UINT8  | HARDWARE_VALUE, .config.minmaxUnsigned = { 0, SPIDEV_COUNT }, PG_SDCARD_CONFIG, offsetof(sdcardConfig_t, device) }, // sdcard_spi_bus
#endif
#ifdef USE_SDCARD_SDIO
    { "7B",            VAR_UINT8  | HARDWARE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_SDIO_CONFIG, offsetof(sdioConfig_t, clockBypass) }, // sdio_clk_bypass
    { "7C",             VAR_UINT8  | HARDWARE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_SDIO_CONFIG, offsetof(sdioConfig_t, useCache) }, // sdio_use_cache
    { "7D",        VAR_UINT8  | HARDWARE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_SDIO_CONFIG, offsetof(sdioConfig_t, use4BitWidth) }, // sdio_use_4bit_width
#ifdef STM32H7
    { "7E",                VAR_UINT8  | HARDWARE_VALUE, .config.minmaxUnsigned = { 0, SDIODEV_COUNT }, PG_SDIO_CONFIG, offsetof(sdioConfig_t, device) }, // sdio_device
#endif
#endif

// PG_OSD_CONFIG
#ifdef USE_OSD
    { "7F",                  VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_UNIT }, PG_OSD_CONFIG, offsetof(osdConfig_t, units) }, // osd_units

// Please try to keep the OSD warnings in the same order as presented in the Configurator.
// This makes it easier for the user to relate the CLI output as warnings are in the same relative
// position as displayed in the GUI.
    { "7G",    VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = OSD_WARNING_ARMING_DISABLE,   PG_OSD_CONFIG, offsetof(osdConfig_t, enabledWarnings)}, // osd_warn_arming_disable
    { "7H",     VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = OSD_WARNING_BATTERY_NOT_FULL, PG_OSD_CONFIG, offsetof(osdConfig_t, enabledWarnings)}, // osd_warn_batt_not_full
    { "7I",      VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = OSD_WARNING_BATTERY_WARNING,  PG_OSD_CONFIG, offsetof(osdConfig_t, enabledWarnings)}, // osd_warn_batt_warning
    { "7J",     VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = OSD_WARNING_BATTERY_CRITICAL, PG_OSD_CONFIG, offsetof(osdConfig_t, enabledWarnings)}, // osd_warn_batt_critical
    { "7K",     VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = OSD_WARNING_VISUAL_BEEPER,    PG_OSD_CONFIG, offsetof(osdConfig_t, enabledWarnings)}, // osd_warn_visual_beeper
    { "7L",        VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = OSD_WARNING_CRASH_FLIP,       PG_OSD_CONFIG, offsetof(osdConfig_t, enabledWarnings)}, // osd_warn_crash_flip
    { "7M",          VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = OSD_WARNING_ESC_FAIL,         PG_OSD_CONFIG, offsetof(osdConfig_t, enabledWarnings)}, // osd_warn_esc_fail
    { "7N",    VAR_UINT16  | MASTER_VALUE | MODE_BITSET, .config.bitpos = OSD_WARNING_LEVEL_RECOVERY,   PG_OSD_CONFIG, offsetof(osdConfig_t, enabledWarnings)}, // osd_warn_level_recovery
#ifdef USE_ADC_INTERNAL
    { "7O",         VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = OSD_WARNING_CORE_TEMPERATURE, PG_OSD_CONFIG, offsetof(osdConfig_t, enabledWarnings)}, // osd_warn_core_temp
#endif
#ifdef USE_RC_SMOOTHING_FILTER
    { "7P",      VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = OSD_WARNING_RC_SMOOTHING,     PG_OSD_CONFIG, offsetof(osdConfig_t, enabledWarnings)}, // osd_warn_rc_smoothing
#endif
    { "7Q",         VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = OSD_WARNING_FAIL_SAFE,   PG_OSD_CONFIG, offsetof(osdConfig_t, enabledWarnings)}, // osd_warn_fail_safe
#ifdef USE_LAUNCH_CONTROL
    { "7R",    VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = OSD_WARNING_LAUNCH_CONTROL,   PG_OSD_CONFIG, offsetof(osdConfig_t, enabledWarnings)}, // osd_warn_launch_control
#endif
    { "7S",     VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = OSD_WARNING_GPS_RESCUE_UNAVAILABLE, PG_OSD_CONFIG, offsetof(osdConfig_t, enabledWarnings)}, // osd_warn_no_gps_rescue
    { "7T", VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = OSD_WARNING_GPS_RESCUE_DISABLED, PG_OSD_CONFIG, offsetof(osdConfig_t, enabledWarnings)}, // osd_warn_gps_rescue_disabled
    { "7U",              VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = OSD_WARNING_RSSI,             PG_OSD_CONFIG, offsetof(osdConfig_t, enabledWarnings)}, // osd_warn_rssi
#ifdef USE_RX_LINK_QUALITY_INFO
    { "7V",      VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = OSD_WARNING_LINK_QUALITY,     PG_OSD_CONFIG, offsetof(osdConfig_t, enabledWarnings)}, // osd_warn_link_quality
#endif
#if defined(USE_RX_RSSI_DBM)
    { "7W",          VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = OSD_WARNING_RSSI_DBM,         PG_OSD_CONFIG, offsetof(osdConfig_t, enabledWarnings)}, // osd_warn_rssi_dbm
#endif
    { "7X",          VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = OSD_WARNING_OVER_CAP,         PG_OSD_CONFIG, offsetof(osdConfig_t, enabledWarnings)}, // osd_warn_over_cap

    { "7Y",             VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 0, 100 }, PG_OSD_CONFIG, offsetof(osdConfig_t, rssi_alarm) }, // osd_rssi_alarm
#ifdef USE_RX_LINK_QUALITY_INFO
    { "7Z",     VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, 100 }, PG_OSD_CONFIG, offsetof(osdConfig_t, link_quality_alarm) }, // osd_link_quality_alarm
#endif
#ifdef USE_RX_RSSI_DBM
    { "80",         VAR_INT16   | MASTER_VALUE, .config.minmaxUnsigned = { CRSF_RSSI_MIN, CRSF_SNR_MAX }, PG_OSD_CONFIG, offsetof(osdConfig_t, rssi_dbm_alarm) }, // osd_rssi_dbm_alarm
#endif
    { "81",              VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 0, 20000 }, PG_OSD_CONFIG, offsetof(osdConfig_t, cap_alarm) }, // osd_cap_alarm
    { "82",              VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 0, 10000 }, PG_OSD_CONFIG, offsetof(osdConfig_t, alt_alarm) }, // osd_alt_alarm
    { "83",         VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 0, UINT16_MAX }, PG_OSD_CONFIG, offsetof(osdConfig_t, distance_alarm) }, // osd_distance_alarm
    { "84",         VAR_INT8   | MASTER_VALUE, .config.minmax = { INT8_MIN, INT8_MAX }, PG_OSD_CONFIG, offsetof(osdConfig_t, esc_temp_alarm) }, // osd_esc_temp_alarm
    { "85",          VAR_INT16  | MASTER_VALUE, .config.minmax = { ESC_RPM_ALARM_OFF, INT16_MAX }, PG_OSD_CONFIG, offsetof(osdConfig_t, esc_rpm_alarm) }, // osd_esc_rpm_alarm
    { "86",      VAR_INT16  | MASTER_VALUE, .config.minmax = { ESC_CURRENT_ALARM_OFF, INT16_MAX }, PG_OSD_CONFIG, offsetof(osdConfig_t, esc_current_alarm) }, // osd_esc_current_alarm
#ifdef USE_ADC_INTERNAL
    { "87",        VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 0, UINT8_MAX }, PG_OSD_CONFIG, offsetof(osdConfig_t, core_temp_alarm) }, // osd_core_temp_alarm
#endif

    { "88",             VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 0, 90 }, PG_OSD_CONFIG, offsetof(osdConfig_t, ahMaxPitch) }, // osd_ah_max_pit
    { "89",             VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 0, 90 }, PG_OSD_CONFIG, offsetof(osdConfig_t, ahMaxRoll) }, // osd_ah_max_rol
    { "8a",              VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_OSD_CONFIG, offsetof(osdConfig_t, ahInvert) }, // osd_ah_invert
    { "8b",         VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OSD_LOGO_ON_ARMING }, PG_OSD_CONFIG, offsetof(osdConfig_t, logo_on_arming) }, // osd_logo_on_arming
    { "8c",VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 5, 50 }, PG_OSD_CONFIG, offsetof(osdConfig_t, logo_on_arming_duration) }, // osd_logo_on_arming_duration

    { "8d",                   VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 0, INT16_MAX }, PG_OSD_CONFIG, offsetof(osdConfig_t, timers[OSD_TIMER_1]) }, // osd_tim1
    { "8e",                   VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 0, INT16_MAX }, PG_OSD_CONFIG, offsetof(osdConfig_t, timers[OSD_TIMER_2]) }, // osd_tim2

    { "8f",               VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_MAIN_BATT_VOLTAGE]) }, // osd_vbat_pos
    { "8g",               VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_RSSI_VALUE]) }, // osd_rssi_pos
#ifdef USE_RX_LINK_QUALITY_INFO
    { "8h",       VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_LINK_QUALITY]) }, // osd_link_quality_pos
#endif
#ifdef USE_RX_RSSI_DBM
    { "8i",           VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_RSSI_DBM_VALUE]) }, // osd_rssi_dbm_pos
#endif
    { "8j",              VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_ITEM_TIMER_1]) }, // osd_tim_1_pos
    { "8k",              VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_ITEM_TIMER_2]) }, // osd_tim_2_pos
    { "8l",        VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_REMAINING_TIME_ESTIMATE]) }, // osd_remaining_time_estimate_pos
    { "8m",            VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_FLYMODE]) }, // osd_flymode_pos
    { "8n",       VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_ANTI_GRAVITY]) }, // osd_anti_gravity_pos
    { "8o",            VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_G_FORCE]) }, // osd_g_force_pos
    { "8p",           VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_THROTTLE_POS]) }, // osd_throttle_pos
    { "8q",        VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_VTX_CHANNEL]) }, // osd_vtx_channel_pos
    { "8r",         VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_CROSSHAIRS]) }, // osd_crosshairs_pos
    { "8s",            VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_HORIZON_SIDEBARS]) }, // osd_ah_sbar_pos
    { "8t",                 VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_ARTIFICIAL_HORIZON]) }, // osd_ah_pos
    { "8u",            VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_CURRENT_DRAW]) }, // osd_current_pos
    { "8v",          VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_MAH_DRAWN]) }, // osd_mah_drawn_pos
    { "8w",         VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_MOTOR_DIAG]) }, // osd_motor_diag_pos
    { "8x",         VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_CRAFT_NAME]) }, // osd_craft_name_pos
    { "8y",       VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_DISPLAY_NAME]) }, // osd_display_name_pos
    { "8z",          VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_GPS_SPEED]) }, // osd_gps_speed_pos
    { "8A",            VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_GPS_LON]) }, // osd_gps_lon_pos
    { "8B",            VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_GPS_LAT]) }, // osd_gps_lat_pos
    { "8C",           VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_GPS_SATS]) }, // osd_gps_sats_pos
    { "8D",           VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_HOME_DIR]) }, // osd_home_dir_pos
    { "8E",          VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_HOME_DIST]) }, // osd_home_dist_pos
    { "8F",        VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_FLIGHT_DIST]) }, // osd_flight_dist_pos
    { "8G",        VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_COMPASS_BAR]) }, // osd_compass_bar_pos
    { "8H",           VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_ALTITUDE]) }, // osd_altitude_pos
    { "8I",           VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_ROLL_PIDS]) }, // osd_pid_roll_pos
    { "8J",          VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_PITCH_PIDS]) }, // osd_pid_pitch_pos
    { "8K",            VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_YAW_PIDS]) }, // osd_pid_yaw_pos
    { "8L",              VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_DEBUG]) }, // osd_debug_pos
    { "8M",              VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_POWER]) }, // osd_power_pos
    { "8N",    VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_PIDRATE_PROFILE]) }, // osd_pidrate_profile_pos
    { "8O",           VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_WARNINGS]) }, // osd_warnings_pos
    { "8P",   VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_AVG_CELL_VOLTAGE]) }, // osd_avg_cell_voltage_pos
    { "8Q",            VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_PITCH_ANGLE]) }, // osd_pit_ang_pos
    { "8R",            VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_ROLL_ANGLE]) }, // osd_rol_ang_pos
    { "8S",      VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_MAIN_BATT_USAGE]) }, // osd_battery_usage_pos
    { "8T",           VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_DISARMED]) }, // osd_disarmed_pos
    { "8U",           VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_NUMERICAL_HEADING]) }, // osd_nheading_pos
#ifdef USE_VARIO
    { "8V",             VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_NUMERICAL_VARIO]) }, // osd_nvario_pos
#endif
    { "8W",            VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_ESC_TMP]) }, // osd_nvario_pos
    { "8X",            VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_ESC_RPM]) }, // osd_esc_rpm_pos
    { "8Y",       VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_ESC_RPM_FREQ]) }, // osd_esc_rpm_freq_pos
    { "8Z",      VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_RTC_DATETIME]) }, // osd_rtc_date_time_pos
    { "90",   VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_ADJUSTMENT_RANGE]) }, // osd_adjustment_range_pos
    { "91",         VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_FLIP_ARROW]) }, // osd_flip_arrow_pos
#ifdef USE_ADC_INTERNAL
    { "92",          VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_CORE_TEMPERATURE]) }, // osd_core_temp_pos
#endif
#ifdef USE_BLACKBOX
    { "93",         VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_LOG_STATUS]) }, // osd_log_status_pos
#endif

#ifdef USE_OSD_STICK_OVERLAY
    { "94",    VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_STICK_OVERLAY_LEFT]) }, // osd_stick_overlay_left_pos
    { "95",   VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_STICK_OVERLAY_RIGHT]) }, // osd_stick_overlay_right_pos

    { "96",  VAR_UINT8   | MASTER_VALUE, .config.minmaxUnsigned = { 1, 4 }, PG_OSD_CONFIG, offsetof(osdConfig_t, overlay_radio_mode) }, // osd_stick_overlay_radio_mode
#endif

#ifdef USE_PROFILE_NAMES
    { "97",  VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_RATE_PROFILE_NAME]) }, // osd_rate_profile_name_pos
    { "98",   VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_PID_PROFILE_NAME]) }, // osd_pid_profile_name_pos
#endif

#ifdef USE_OSD_PROFILES
    { "99",   VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_PROFILE_NAME]) }, // osd_profile_name_pos
#endif

    { "9a",     VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_RC_CHANNELS]) }, // osd_profile_name_pos
    { "9b",   VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_CAMERA_FRAME]) }, // osd_camera_frame_pos
    { "9c",     VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_EFFICIENCY]) }, // osd_efficiency_pos
    { "9e",     VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_TOTAL_FLIGHTS]) }, // osd_total_flights_pos

    // OSD stats enabled flags are stored as bitmapped values inside a 32bit parameter
    // It is recommended to keep the settings order the same as the enumeration. This way the settings are displayed in the cli in the same order making it easier on the users
    { "9f",     VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = OSD_STAT_RTC_DATE_TIME,   PG_OSD_CONFIG, offsetof(osdConfig_t, enabled_stats)}, // osd_stat_rtc_date_time
    { "9g",             VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = OSD_STAT_TIMER_1,         PG_OSD_CONFIG, offsetof(osdConfig_t, enabled_stats)}, // osd_stat_tim_1
    { "9h",             VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = OSD_STAT_TIMER_2,         PG_OSD_CONFIG, offsetof(osdConfig_t, enabled_stats)}, // osd_stat_tim_2
    { "9i",           VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = OSD_STAT_MAX_SPEED,       PG_OSD_CONFIG, offsetof(osdConfig_t, enabled_stats)}, // osd_stat_max_spd
    { "9j",          VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = OSD_STAT_MAX_DISTANCE,    PG_OSD_CONFIG, offsetof(osdConfig_t, enabled_stats)}, // osd_stat_max_dist
    { "9k",          VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = OSD_STAT_MIN_BATTERY,     PG_OSD_CONFIG, offsetof(osdConfig_t, enabled_stats)}, // osd_stat_min_batt
    { "9l",           VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = OSD_STAT_END_BATTERY,     PG_OSD_CONFIG, offsetof(osdConfig_t, enabled_stats)}, // osd_stat_endbatt
    { "9m",           VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = OSD_STAT_BATTERY,         PG_OSD_CONFIG, offsetof(osdConfig_t, enabled_stats)}, // osd_stat_battery
    { "9n",          VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = OSD_STAT_MIN_RSSI,        PG_OSD_CONFIG, offsetof(osdConfig_t, enabled_stats)}, // osd_stat_min_rssi
    { "9o",          VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = OSD_STAT_MAX_CURRENT,     PG_OSD_CONFIG, offsetof(osdConfig_t, enabled_stats)}, // osd_stat_max_curr
    { "9p",          VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = OSD_STAT_USED_MAH,        PG_OSD_CONFIG, offsetof(osdConfig_t, enabled_stats)}, // osd_stat_used_mah
    { "9q",           VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = OSD_STAT_MAX_ALTITUDE,    PG_OSD_CONFIG, offsetof(osdConfig_t, enabled_stats)}, // osd_stat_max_alt
    { "9r",              VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = OSD_STAT_BLACKBOX,        PG_OSD_CONFIG, offsetof(osdConfig_t, enabled_stats)}, // osd_stat_bbox
    { "9s",             VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = OSD_STAT_BLACKBOX_NUMBER, PG_OSD_CONFIG, offsetof(osdConfig_t, enabled_stats)}, // osd_stat_bb_no
    { "9t",       VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = OSD_STAT_MAX_G_FORCE,     PG_OSD_CONFIG, offsetof(osdConfig_t, enabled_stats)}, // osd_stat_max_g_force
    { "9u",      VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = OSD_STAT_MAX_ESC_TEMP,    PG_OSD_CONFIG, offsetof(osdConfig_t, enabled_stats)}, // osd_stat_max_esc_temp
    { "9v",       VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = OSD_STAT_MAX_ESC_RPM,     PG_OSD_CONFIG, offsetof(osdConfig_t, enabled_stats)}, // osd_stat_max_esc_rpm
    { "9w",  VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = OSD_STAT_MIN_LINK_QUALITY,PG_OSD_CONFIG, offsetof(osdConfig_t, enabled_stats)}, // osd_stat_min_link_quality
    { "9x",       VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = OSD_STAT_FLIGHT_DISTANCE, PG_OSD_CONFIG, offsetof(osdConfig_t, enabled_stats)}, // osd_stat_flight_dist
#ifdef USE_GYRO_DATA_ANALYSE
    { "9y",           VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = OSD_STAT_MAX_FFT, PG_OSD_CONFIG, offsetof(osdConfig_t, enabled_stats)}, // osd_stat_max_fft
#endif
#ifdef USE_PERSISTENT_STATS
    { "9z",     VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = OSD_STAT_TOTAL_FLIGHTS, PG_OSD_CONFIG, offsetof(osdConfig_t, enabled_stats)}, // osd_stat_total_flights
    { "9A",        VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = OSD_STAT_TOTAL_TIME,    PG_OSD_CONFIG, offsetof(osdConfig_t, enabled_stats)}, // osd_stat_total_time
    { "9B",        VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = OSD_STAT_TOTAL_DIST,    PG_OSD_CONFIG, offsetof(osdConfig_t, enabled_stats)}, // osd_stat_total_time
#endif
#ifdef USE_RX_RSSI_DBM
    { "9C",      VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = OSD_STAT_MIN_RSSI_DBM,  PG_OSD_CONFIG, offsetof(osdConfig_t, enabled_stats)}, // osd_stat_min_rssi_dbm
#endif

#ifdef USE_OSD_PROFILES
    { "9D",                VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 1, OSD_PROFILE_COUNT }, PG_OSD_CONFIG, offsetof(osdConfig_t, osdProfileIndex) }, // osd_profile
    { "9E",         VAR_UINT8  | MASTER_VALUE | MODE_STRING, .config.string = { 1, OSD_PROFILE_NAME_LENGTH, STRING_FLAGS_NONE }, PG_OSD_CONFIG, offsetof(osdConfig_t, profile[0]) }, // osd_profile_1_name
    { "9F",         VAR_UINT8  | MASTER_VALUE | MODE_STRING, .config.string = { 1, OSD_PROFILE_NAME_LENGTH, STRING_FLAGS_NONE }, PG_OSD_CONFIG, offsetof(osdConfig_t, profile[1]) }, // osd_profile_2_name
    { "9G",         VAR_UINT8  | MASTER_VALUE | MODE_STRING, .config.string = { 1, OSD_PROFILE_NAME_LENGTH, STRING_FLAGS_NONE }, PG_OSD_CONFIG, offsetof(osdConfig_t, profile[2]) }, // osd_profile_3_name
#endif
    { "9H",     VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_OSD_CONFIG, offsetof(osdConfig_t, gps_sats_show_hdop) }, // osd_gps_sats_show_hdop
    { "9I",     VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OSD_DISPLAYPORT_DEVICE }, PG_OSD_CONFIG, offsetof(osdConfig_t, displayPortDevice) }, // osd_displayport_device

    { "9J",             VAR_INT8   | MASTER_VALUE | MODE_ARRAY, .config.array.length = OSD_RCCHANNELS_COUNT, PG_OSD_CONFIG, offsetof(osdConfig_t, rcChannels) }, // osd_rcchannels
    { "9K",     VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { OSD_CAMERA_FRAME_MIN_WIDTH, OSD_CAMERA_FRAME_MAX_WIDTH }, PG_OSD_CONFIG, offsetof(osdConfig_t, camera_frame_width) }, // osd_camera_frame_width
    { "9L",    VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { OSD_CAMERA_FRAME_MIN_HEIGHT, OSD_CAMERA_FRAME_MAX_HEIGHT }, PG_OSD_CONFIG, offsetof(osdConfig_t, camera_frame_height) }, // osd_camera_frame_height
    { "9M",         VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { OSD_TASK_FREQUENCY_MIN, OSD_TASK_FREQUENCY_MAX }, PG_OSD_CONFIG, offsetof(osdConfig_t, task_frequency) }, // osd_task_frequency
    { "9N",        VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_CMS_BACKGROUND }, PG_OSD_CONFIG, offsetof(osdConfig_t, cms_background_type) }, // osd_menu_background
#endif // end of #ifdef USE_OSD

// PG_SYSTEM_CONFIG
#if defined(STM32F4) || defined(STM32G4)
    { "9O",             VAR_UINT8  | HARDWARE_VALUE, .config.minmaxUnsigned = { 0, 30 }, PG_SYSTEM_CONFIG, offsetof(systemConfig_t, hseMhz) }, // system_hse_mhz
#endif
#if defined(USE_TASK_STATISTICS)
    { "9P",            VAR_INT8   | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_SYSTEM_CONFIG, offsetof(systemConfig_t, task_statistics) }, // task_statistics
#endif
    { "9Q",                 VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_DEBUG }, PG_SYSTEM_CONFIG, offsetof(systemConfig_t, debug_mode) }, // debug_mode
    { "9R",           VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_SYSTEM_CONFIG, offsetof(systemConfig_t, rateProfile6PosSwitch) }, // rate_6pos_switch
#ifdef USE_OVERCLOCK
    { "9S",              VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OVERCLOCK }, PG_SYSTEM_CONFIG, offsetof(systemConfig_t, cpu_overclock) }, // cpu_overclock
#endif
    { "9T",           VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 0, 30 }, PG_SYSTEM_CONFIG, offsetof(systemConfig_t, powerOnArmingGraceTime) }, // pwr_on_arm_grace
    { "9U",    VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON_AUTO }, PG_SYSTEM_CONFIG, offsetof(systemConfig_t, schedulerOptimizeRate) }, // scheduler_optimize_rate
    { "9V",        VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_SYSTEM_CONFIG, offsetof(systemConfig_t, enableStickArming) }, // enable_stick_arming

// PG_VTX_CONFIG
#ifdef USE_VTX_COMMON
    { "9W",                   VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 0, VTX_TABLE_MAX_BANDS }, PG_VTX_SETTINGS_CONFIG, offsetof(vtxSettingsConfig_t, band) }, // vtx_band
    { "9X",                VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 0, VTX_TABLE_MAX_CHANNELS }, PG_VTX_SETTINGS_CONFIG, offsetof(vtxSettingsConfig_t, channel) }, // vtx_channel
    { "9Y",                  VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 0, VTX_TABLE_MAX_POWER_LEVELS - 1 }, PG_VTX_SETTINGS_CONFIG, offsetof(vtxSettingsConfig_t, power) }, // vtx_power
    { "9Z",       VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_VTX_LOW_POWER_DISARM }, PG_VTX_SETTINGS_CONFIG, offsetof(vtxSettingsConfig_t, lowPowerDisarm) }, // vtx_low_power_disarm
#ifdef VTX_SETTINGS_FREQCMD
    { "a0",                   VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 0, VTX_SETTINGS_MAX_FREQUENCY_MHZ }, PG_VTX_SETTINGS_CONFIG, offsetof(vtxSettingsConfig_t, freq) }, // vtx_freq
    { "a1",          VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 0, VTX_SETTINGS_MAX_FREQUENCY_MHZ }, PG_VTX_SETTINGS_CONFIG, offsetof(vtxSettingsConfig_t, pitModeFreq) }, // vtx_pit_mode_freq
#endif
#endif

// PG_VTX_CONFIG
#if defined(USE_VTX_CONTROL) && defined(USE_VTX_COMMON)
    { "a2",             VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_VTX_CONFIG, offsetof(vtxConfig_t, halfDuplex) }, // vtx_halfduplex
#endif

// PG_VTX_IO
#ifdef USE_VTX_RTC6705
    { "a3",                VAR_UINT8  | HARDWARE_VALUE | MASTER_VALUE, .config.minmaxUnsigned = { 0, SPIDEV_COUNT }, PG_VTX_IO_CONFIG, offsetof(vtxIOConfig_t, spiDevice) }, // vtx_spi_bus
#endif

// PG_VCD_CONFIG
#if defined(USE_MAX7456) || defined(USE_FRSKYOSD)
    { "a4",           VAR_UINT8   | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_VIDEO_SYSTEM }, PG_VCD_CONFIG, offsetof(vcdProfile_t, video_system) }, // vcd_video_system
    { "a5",               VAR_INT8    | MASTER_VALUE, .config.minmax = { -32, 31 }, PG_VCD_CONFIG, offsetof(vcdProfile_t, h_offset) }, // vcd_h_offset
    { "a6",               VAR_INT8    | MASTER_VALUE, .config.minmax = { -15, 16 }, PG_VCD_CONFIG, offsetof(vcdProfile_t, v_offset) }, // vcd_v_offset
#endif

// PG_MAX7456_CONFIG
#ifdef USE_MAX7456
    { "a7",              VAR_UINT8   | HARDWARE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_MAX7456_CLOCK }, PG_MAX7456_CONFIG, offsetof(max7456Config_t, clockConfig) }, // max7456_clock
    { "a8",            VAR_UINT8   | HARDWARE_VALUE, .config.minmaxUnsigned = { 0, SPIDEV_COUNT }, PG_MAX7456_CONFIG, offsetof(max7456Config_t, spiDevice) }, // max7456_spi_bus
    { "a9",        VAR_UINT8   | HARDWARE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_MAX7456_CONFIG, offsetof(max7456Config_t, preInitOPU) }, // max7456_preinit_opu
#endif

// PG_DISPLAY_PORT_MSP_CONFIG
#ifdef USE_MSP_DISPLAYPORT
    { "aa", VAR_INT8    | MASTER_VALUE, .config.minmax = { -6, 0 }, PG_DISPLAY_PORT_MSP_CONFIG, offsetof(displayPortProfile_t, colAdjust) }, // displayport_msp_col_adjust
    { "ab", VAR_INT8    | MASTER_VALUE, .config.minmax = { -3, 0 }, PG_DISPLAY_PORT_MSP_CONFIG, offsetof(displayPortProfile_t, rowAdjust) }, // displayport_msp_row_adjust
    { "ac",     VAR_INT8    | MASTER_VALUE, .config.minmax = { SERIAL_PORT_NONE, SERIAL_PORT_IDENTIFIER_MAX }, PG_DISPLAY_PORT_MSP_CONFIG, offsetof(displayPortProfile_t, displayPortSerial) }, // displayport_msp_serial
    { "ad",      VAR_UINT8   | MASTER_VALUE | MODE_ARRAY, .config.array.length = 4, PG_DISPLAY_PORT_MSP_CONFIG, offsetof(displayPortProfile_t, attrValues) }, // displayport_msp_attrs
    { "ae",   VAR_UINT8   | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_DISPLAY_PORT_MSP_CONFIG, offsetof(displayPortProfile_t, useDeviceBlink) }, // displayport_msp_use_device_blink
#endif

// PG_DISPLAY_PORT_MSP_CONFIG
#ifdef USE_MAX7456
    { "af", VAR_INT8| MASTER_VALUE, .config.minmax = { -6, 0 }, PG_DISPLAY_PORT_MAX7456_CONFIG, offsetof(displayPortProfile_t, colAdjust) }, // displayport_max7456_col_adjust
    { "ag", VAR_INT8| MASTER_VALUE, .config.minmax = { -3, 0 }, PG_DISPLAY_PORT_MAX7456_CONFIG, offsetof(displayPortProfile_t, rowAdjust) }, // displayport_max7456_row_adjust
    { "ah",        VAR_UINT8| MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_DISPLAY_PORT_MAX7456_CONFIG, offsetof(displayPortProfile_t, invert) }, // displayport_max7456_inv
    { "ai",        VAR_UINT8| MASTER_VALUE, .config.minmaxUnsigned = { 0, 3 }, PG_DISPLAY_PORT_MAX7456_CONFIG, offsetof(displayPortProfile_t, blackBrightness) }, // displayport_max7456_blk
    { "aj",        VAR_UINT8| MASTER_VALUE, .config.minmaxUnsigned = { 0, 3 }, PG_DISPLAY_PORT_MAX7456_CONFIG, offsetof(displayPortProfile_t, whiteBrightness) }, // displayport_max7456_wht
    { "ak",      VAR_UINT8| MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_DISPLAY_PORT_MAX7456_CONFIG, offsetof(displayPortProfile_t, useBlackBackgroundInMenus) }, // osd_menu_black_background
#endif

#ifdef USE_ESC_SENSOR
    { "al",          VAR_UINT8   | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_ESC_SENSOR_CONFIG, offsetof(escSensorConfig_t, halfDuplex) }, // esc_sensor_halfduplex
    { "am",      VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, 16000 }, PG_ESC_SENSOR_CONFIG, offsetof(escSensorConfig_t, offset) }, // esc_sensor_current_offset
#endif

#ifdef USE_RX_FRSKY_SPI
    { "an",             VAR_UINT8   | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_RX_CC2500_SPI_CONFIG, offsetof(rxCc2500SpiConfig_t, autoBind) }, // frsky_spi_autobind
    { "ao",                VAR_UINT8   | MASTER_VALUE | MODE_ARRAY, .config.array.length = 3, PG_RX_CC2500_SPI_CONFIG, offsetof(rxCc2500SpiConfig_t, bindTxId) }, // frsky_spi_tx_id
    { "ap",               VAR_INT8    | MASTER_VALUE, .config.minmax = { -127, 127 }, PG_RX_CC2500_SPI_CONFIG, offsetof(rxCc2500SpiConfig_t, bindOffset) }, // frsky_spi_offset
    { "aq",        VAR_UINT8   | MASTER_VALUE | MODE_ARRAY, .config.array.length = 50, PG_RX_CC2500_SPI_CONFIG, offsetof(rxCc2500SpiConfig_t, bindHopData) }, // frsky_spi_bind_hop_data
    { "ar",                 VAR_UINT8   | MASTER_VALUE, .config.minmaxUnsigned = { 0, UINT8_MAX }, PG_RX_CC2500_SPI_CONFIG, offsetof(rxCc2500SpiConfig_t, rxNum) }, // frsky_x_rx_num
    { "as",            VAR_UINT8   | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_RX_FRSKY_SPI_A1_SOURCE }, PG_RX_CC2500_SPI_CONFIG, offsetof(rxCc2500SpiConfig_t, a1Source) }, // frsky_spi_a1_source
    { "at",         VAR_UINT8   | HARDWARE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_RX_CC2500_SPI_CONFIG, offsetof(rxCc2500SpiConfig_t, chipDetectEnabled) }, // cc2500_spi_chip_detect
#endif
    { "au",                  VAR_UINT8  | HARDWARE_VALUE, .config.minmaxUnsigned = { 0, ((1 << STATUS_LED_NUMBER) - 1) }, PG_STATUS_LED_CONFIG, offsetof(statusLedConfig_t, inversion) }, // led_inversion
#ifdef USE_DASHBOARD
    { "av",           VAR_UINT8  | HARDWARE_VALUE, .config.minmaxUnsigned = { 0, I2CDEV_COUNT }, PG_DASHBOARD_CONFIG, offsetof(dashboardConfig_t, device) }, // dashboard_i2c_bus
    { "aw",          VAR_UINT8  | HARDWARE_VALUE, .config.minmaxUnsigned = { I2C_ADDR7_MIN, I2C_ADDR7_MAX }, PG_DASHBOARD_CONFIG, offsetof(dashboardConfig_t, address) }, //
#endif

// PG_CAMERA_CONTROL_CONFIG
#ifdef USE_CAMERA_CONTROL
    { "ax", VAR_UINT8 | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_CAMERA_CONTROL_MODE }, PG_CAMERA_CONTROL_CONFIG, offsetof(cameraControlConfig_t, mode) }, // camera_control_mode
    { "ay", VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 200, 400 }, PG_CAMERA_CONTROL_CONFIG, offsetof(cameraControlConfig_t, refVoltage) }, // camera_control_ref_voltage
    { "az", VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 100, 500 }, PG_CAMERA_CONTROL_CONFIG, offsetof(cameraControlConfig_t, keyDelayMs) }, // camera_control_key_delay
    { "aA", VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 10, 1000 }, PG_CAMERA_CONTROL_CONFIG, offsetof(cameraControlConfig_t, internalResistance) }, // camera_control_internal_resistance
    { "aB",   VAR_UINT16 | MASTER_VALUE | MODE_ARRAY, .config.array.length = CAMERA_CONTROL_KEYS_COUNT, PG_CAMERA_CONTROL_CONFIG, offsetof(cameraControlConfig_t, buttonResistanceValues) }, // camera_control_button_resistance
    { "aC", VAR_UINT8 | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_CAMERA_CONTROL_CONFIG, offsetof(cameraControlConfig_t, inverted) }, // camera_control_inverted
#endif

// PG_RANGEFINDER_CONFIG
#ifdef USE_RANGEFINDER
    { "aD", VAR_UINT8 | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_RANGEFINDER_HARDWARE }, PG_RANGEFINDER_CONFIG, offsetof(rangefinderConfig_t, rangefinder_hardware) }, // rangefinder_hardware
#endif

// PG_PINIO_CONFIG
#ifdef USE_PINIO
    { "aE", VAR_UINT8 | HARDWARE_VALUE | MODE_ARRAY, .config.array.length = PINIO_COUNT, PG_PINIO_CONFIG, offsetof(pinioConfig_t, config) }, // pinio_config
#ifdef USE_PINIOBOX
    { "aF", VAR_UINT8 | HARDWARE_VALUE | MODE_ARRAY, .config.array.length = PINIO_COUNT, PG_PINIOBOX_CONFIG, offsetof(pinioBoxConfig_t, permanentId) }, // pinio_box
#endif
#endif

//PG USB
#ifdef USE_USB_CDC_HID
    { "aG", VAR_UINT8 | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_USB_CONFIG, offsetof(usbDev_t, type) }, // usb_hid_cdc
#endif
#ifdef USE_USB_MSC
    { "aH", VAR_UINT8 | HARDWARE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_USB_CONFIG, offsetof(usbDev_t, mscButtonUsePullup) }, // usb_msc_pin_pullup
#endif
// PG_FLASH_CONFIG
#ifdef USE_FLASH_CHIP
    { "aI", VAR_UINT8 | HARDWARE_VALUE, .config.minmaxUnsigned = { 0, SPIDEV_COUNT }, PG_FLASH_CONFIG, offsetof(flashConfig_t, spiDevice) }, // flash_spi_bus
#endif
// RCDEVICE
#ifdef USE_RCDEVICE
    { "aJ", VAR_UINT8 | MASTER_VALUE, .config.minmaxUnsigned = { 0, 10 }, PG_RCDEVICE_CONFIG, offsetof(rcdeviceConfig_t, initDeviceAttempts) }, // rcdevice_init_dev_attempts
    { "aK", VAR_UINT32 | MASTER_VALUE, .config.u32Max = 5000, PG_RCDEVICE_CONFIG, offsetof(rcdeviceConfig_t, initDeviceAttemptInterval) }, // rcdevice_init_dev_attempt_interval
    { "aL", VAR_UINT8 | MASTER_VALUE, .config.minmaxUnsigned = { 0, 1 }, PG_RCDEVICE_CONFIG, offsetof(rcdeviceConfig_t, protocolVersion) }, // rcdevice_protocol_version
    { "aM", VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = {0, 65535}, PG_RCDEVICE_CONFIG, offsetof(rcdeviceConfig_t, feature) }, // rcdevice_feature
#endif

// PG_GYRO_DEVICE_CONFIG
    { "aN", VAR_UINT8 | HARDWARE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_BUS_TYPE }, PG_GYRO_DEVICE_CONFIG, PG_ARRAY_ELEMENT_OFFSET(gyroDeviceConfig_t, 0, bustype) }, // gyro_1_bustype
    { "aO",  VAR_UINT8 | HARDWARE_VALUE, .config.minmaxUnsigned = { 0, SPIDEV_COUNT }, PG_GYRO_DEVICE_CONFIG, PG_ARRAY_ELEMENT_OFFSET(gyroDeviceConfig_t, 0, spiBus) }, // gyro_1_spibus
    { "aP",  VAR_UINT8 | HARDWARE_VALUE, .config.minmaxUnsigned = { 0, I2CDEV_COUNT }, PG_GYRO_DEVICE_CONFIG, PG_ARRAY_ELEMENT_OFFSET(gyroDeviceConfig_t, 0, i2cBus) }, // gyro_1_i2cBus
    { "aQ", VAR_UINT8  | HARDWARE_VALUE, .config.minmaxUnsigned = { 0, I2C_ADDR7_MAX }, PG_GYRO_DEVICE_CONFIG, PG_ARRAY_ELEMENT_OFFSET(gyroDeviceConfig_t, 0, i2cAddress) }, // gyro_1_i2c_address
    { "aR", VAR_UINT8  | HARDWARE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_ALIGNMENT }, PG_GYRO_DEVICE_CONFIG, PG_ARRAY_ELEMENT_OFFSET(gyroDeviceConfig_t, 0, alignment) }, // gyro_1_sensor_align
    { "aS", VAR_INT16  | HARDWARE_VALUE, .config.minmax = { -3600, 3600 }, PG_GYRO_DEVICE_CONFIG, PG_ARRAY_ELEMENT_OFFSET(gyroDeviceConfig_t, 0, customAlignment.roll) }, // gyro_1_align_roll
    { "aT", VAR_INT16  | HARDWARE_VALUE, .config.minmax = { -3600, 3600 }, PG_GYRO_DEVICE_CONFIG, PG_ARRAY_ELEMENT_OFFSET(gyroDeviceConfig_t, 0, customAlignment.pitch) }, // gyro_1_align_pitch
    { "aU", VAR_INT16  | HARDWARE_VALUE, .config.minmax = { -3600, 3600 }, PG_GYRO_DEVICE_CONFIG, PG_ARRAY_ELEMENT_OFFSET(gyroDeviceConfig_t, 0, customAlignment.yaw) }, // gyro_1_align_yaw
#ifdef USE_MULTI_GYRO
    { "aV", VAR_UINT8 | HARDWARE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_BUS_TYPE }, PG_GYRO_DEVICE_CONFIG, PG_ARRAY_ELEMENT_OFFSET(gyroDeviceConfig_t, 1, bustype) }, // gyro_2_bustype
    { "aW",  VAR_UINT8 | HARDWARE_VALUE, .config.minmaxUnsigned = { 0, SPIDEV_COUNT }, PG_GYRO_DEVICE_CONFIG, PG_ARRAY_ELEMENT_OFFSET(gyroDeviceConfig_t, 1, spiBus) }, // gyro_2_spibus
    { "aX",  VAR_UINT8 | HARDWARE_VALUE, .config.minmaxUnsigned = { 0, I2CDEV_COUNT }, PG_GYRO_DEVICE_CONFIG, PG_ARRAY_ELEMENT_OFFSET(gyroDeviceConfig_t, 1, i2cBus) }, // gyro_2_i2cBus
    { "aY", VAR_UINT8  | HARDWARE_VALUE, .config.minmaxUnsigned = { 0, I2C_ADDR7_MAX }, PG_GYRO_DEVICE_CONFIG, PG_ARRAY_ELEMENT_OFFSET(gyroDeviceConfig_t, 1, i2cAddress) }, // gyro_2_i2c_address
    { "aZ", VAR_UINT8  | HARDWARE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_ALIGNMENT }, PG_GYRO_DEVICE_CONFIG, PG_ARRAY_ELEMENT_OFFSET(gyroDeviceConfig_t, 1, alignment) }, // gyro_2_sensor_align
    { "b0", VAR_INT16  | HARDWARE_VALUE, .config.minmax = { -3600, 3600 }, PG_GYRO_DEVICE_CONFIG, PG_ARRAY_ELEMENT_OFFSET(gyroDeviceConfig_t, 1, customAlignment.roll) }, // gyro_2_align_roll
    { "b1", VAR_INT16  | HARDWARE_VALUE, .config.minmax = { -3600, 3600 }, PG_GYRO_DEVICE_CONFIG, PG_ARRAY_ELEMENT_OFFSET(gyroDeviceConfig_t, 1, customAlignment.pitch) }, // gyro_2_align_pitch
    { "b3", VAR_INT16  | HARDWARE_VALUE, .config.minmax = { -3600, 3600 }, PG_GYRO_DEVICE_CONFIG, PG_ARRAY_ELEMENT_OFFSET(gyroDeviceConfig_t, 1, customAlignment.yaw) }, // gyro_2_align_yaw
#endif
#ifdef I2C_FULL_RECONFIGURABILITY
#ifdef USE_I2C_DEVICE_1
    { "b4",    VAR_UINT8  | HARDWARE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_I2C_CONFIG, PG_ARRAY_ELEMENT_OFFSET(i2cConfig_t, 0, pullUp) }, // i2c1_pullup
    { "b5", VAR_UINT8  | HARDWARE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_I2C_CONFIG, PG_ARRAY_ELEMENT_OFFSET(i2cConfig_t, 0, overClock) }, // i2c1_overclock
#endif
#ifdef USE_I2C_DEVICE_2
    { "b6",    VAR_UINT8  | HARDWARE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_I2C_CONFIG, PG_ARRAY_ELEMENT_OFFSET(i2cConfig_t, 1, pullUp) }, // i2c2_pullup
    { "b7", VAR_UINT8  | HARDWARE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_I2C_CONFIG, PG_ARRAY_ELEMENT_OFFSET(i2cConfig_t, 1, overClock) }, // i2c2_overclock
#endif
#ifdef USE_I2C_DEVICE_3
    { "b8",    VAR_UINT8  | HARDWARE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_I2C_CONFIG, PG_ARRAY_ELEMENT_OFFSET(i2cConfig_t, 2, pullUp) }, // i2c3_pullup
    { "b9", VAR_UINT8  | HARDWARE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_I2C_CONFIG, PG_ARRAY_ELEMENT_OFFSET(i2cConfig_t, 2, overClock) }, // i2c3_overclock
#endif
#ifdef USE_I2C_DEVICE_4
    { "ba",    VAR_UINT8  | HARDWARE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_I2C_CONFIG, PG_ARRAY_ELEMENT_OFFSET(i2cConfig_t, 3, pullUp) }, // i2c4_pullup
    { "bb", VAR_UINT8  | HARDWARE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_I2C_CONFIG, PG_ARRAY_ELEMENT_OFFSET(i2cConfig_t, 3, overClock) }, // i2c4_overclock
#endif
#endif
#ifdef USE_MCO
#ifdef STM32G4
    { "bc",     VAR_UINT8  | HARDWARE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_MCO_CONFIG, PG_ARRAY_ELEMENT_OFFSET(mcoConfig_t, 0, enabled) }, // mco_on_pa8
    { "bd",     VAR_UINT8  | HARDWARE_VALUE, .config.minmaxUnsigned = { 0, MCO_SOURCE_COUNT - 1 }, PG_MCO_CONFIG, PG_ARRAY_ELEMENT_OFFSET(mcoConfig_t, 0, source) }, // mco_source
    { "be",    VAR_UINT8  | HARDWARE_VALUE, .config.minmaxUnsigned = { 0, MCO_DIVIDER_COUNT - 1 }, PG_MCO_CONFIG, PG_ARRAY_ELEMENT_OFFSET(mcoConfig_t, 0, divider) }, // mco_divider
#else
    { "bf",    VAR_UINT8  | HARDWARE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_MCO_CONFIG, PG_ARRAY_ELEMENT_OFFSET(mcoConfig_t, 1, enabled) }, // mco2_on_pc9
#endif
#endif
#ifdef USE_RX_SPEKTRUM
    { "bg",     VAR_UINT8 | MASTER_VALUE, .config.minmaxUnsigned = { 0, UINT8_MAX }, PG_RX_SPEKTRUM_SPI_CONFIG, offsetof(spektrumConfig_t, protocol) }, // spektrum_spi_protocol
    { "bh",       VAR_UINT8 | MASTER_VALUE | MODE_ARRAY, .config.array.length = 4, PG_RX_SPEKTRUM_SPI_CONFIG, offsetof(spektrumConfig_t, mfgId) }, // spektrum_spi_mfg_id
    { "bi", VAR_UINT8 | MASTER_VALUE, .config.minmaxUnsigned = { 0, DSM_MAX_CHANNEL_COUNT }, PG_RX_SPEKTRUM_SPI_CONFIG, offsetof(spektrumConfig_t, numChannels) }, // spektrum_spi_num_channels
#endif

// PG_TIMECONFIG
#ifdef USE_RTC_TIME
    { "bj",  VAR_INT16 | MASTER_VALUE, .config.minmax = { TIMEZONE_OFFSET_MINUTES_MIN, TIMEZONE_OFFSET_MINUTES_MAX }, PG_TIME_CONFIG, offsetof(timeConfig_t, tz_offsetMinutes) }, // timezone_offset_minutes
#endif

#ifdef USE_RPM_FILTER
    { "bk",  VAR_UINT8 | MASTER_VALUE, .config.minmaxUnsigned = { 0, 3 }, PG_RPM_FILTER_CONFIG, offsetof(rpmFilterConfig_t, gyro_rpm_notch_harmonics) }, // gyro_rpm_notch_harmonics
    { "bl",  VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 250, 3000 }, PG_RPM_FILTER_CONFIG, offsetof(rpmFilterConfig_t, gyro_rpm_notch_q) }, // gyro_rpm_notch_q
    { "bm",  VAR_UINT8 | MASTER_VALUE, .config.minmaxUnsigned = { 50, 200 }, PG_RPM_FILTER_CONFIG, offsetof(rpmFilterConfig_t, gyro_rpm_notch_min) }, // gyro_rpm_notch_min
    { "bn",  VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 100, 500 }, PG_RPM_FILTER_CONFIG, offsetof(rpmFilterConfig_t, rpm_lpf) }, // rpm_notch_lpf
#endif

#ifdef USE_RX_FLYSKY
    { "bo",       VAR_UINT32 | MASTER_VALUE, .config.u32Max = UINT32_MAX, PG_FLYSKY_CONFIG, offsetof(flySkyConfig_t, txId) }, // flysky_spi_tx_id
    { "bp", VAR_UINT8 | MASTER_VALUE | MODE_ARRAY, .config.array.length = 16, PG_FLYSKY_CONFIG, offsetof(flySkyConfig_t, rfChannelMap) }, // flysky_spi_rf_channels
#endif

#ifdef USE_PERSISTENT_STATS
    { "bq",   VAR_INT8   | MASTER_VALUE, .config.minmax = { STATS_OFF, INT8_MAX }, PG_STATS_CONFIG, offsetof(statsConfig_t, stats_min_armed_time_s) }, // stats_min_armed_time_s
    { "br",    VAR_UINT32 | MASTER_VALUE, .config.u32Max = UINT32_MAX, PG_STATS_CONFIG, offsetof(statsConfig_t, stats_total_flights) }, // stats_total_flights

    { "bs",     VAR_UINT32 | MASTER_VALUE, .config.u32Max = UINT32_MAX, PG_STATS_CONFIG, offsetof(statsConfig_t, stats_total_time_s) }, // stats_total_time_s
    { "bt",     VAR_UINT32 | MASTER_VALUE, .config.u32Max = UINT32_MAX, PG_STATS_CONFIG, offsetof(statsConfig_t, stats_total_dist_m) }, // stats_total_dist_m
#endif
    { "bu",             VAR_UINT8  | MASTER_VALUE | MODE_STRING, .config.string = { 1, MAX_NAME_LENGTH, STRING_FLAGS_NONE }, PG_PILOT_CONFIG, offsetof(pilotConfig_t, name) }, // name
#ifdef USE_OSD
    { "bv",     VAR_UINT8  | MASTER_VALUE | MODE_STRING, .config.string = { 1, MAX_NAME_LENGTH, STRING_FLAGS_NONE }, PG_PILOT_CONFIG, offsetof(pilotConfig_t, displayName) }, // display_name
#endif

// PG_POSITION
    { "bw",           VAR_INT8   | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_POSITION_ALT_SOURCE }, PG_POSITION, offsetof(positionConfig_t, altSource) }, // position_alt_source

// PG_MODE_ACTIVATION_CONFIG
#if defined(USE_CUSTOM_BOX_NAMES)
    { "bx", VAR_UINT8 | HARDWARE_VALUE | MODE_STRING, .config.string = { 1, MAX_BOX_USER_NAME_LENGTH, STRING_FLAGS_NONE }, PG_MODE_ACTIVATION_CONFIG, offsetof(modeActivationConfig_t, box_user_1_name) }, // box_user_1_name
    { "by", VAR_UINT8 | HARDWARE_VALUE | MODE_STRING, .config.string = { 1, MAX_BOX_USER_NAME_LENGTH, STRING_FLAGS_NONE }, PG_MODE_ACTIVATION_CONFIG, offsetof(modeActivationConfig_t, box_user_2_name) }, // box_user_2_name
    { "bz", VAR_UINT8 | HARDWARE_VALUE | MODE_STRING, .config.string = { 1, MAX_BOX_USER_NAME_LENGTH, STRING_FLAGS_NONE }, PG_MODE_ACTIVATION_CONFIG, offsetof(modeActivationConfig_t, box_user_3_name) }, // box_user_3_name
    { "bA", VAR_UINT8 | HARDWARE_VALUE | MODE_STRING, .config.string = { 1, MAX_BOX_USER_NAME_LENGTH, STRING_FLAGS_NONE }, PG_MODE_ACTIVATION_CONFIG, offsetof(modeActivationConfig_t, box_user_4_name) }, // box_user_4_name
#endif
};

const uint16_t valueTableEntryCount = ARRAYLEN(valueTable);

STATIC_ASSERT(LOOKUP_TABLE_COUNT == ARRAYLEN(lookupTables), LOOKUP_TABLE_COUNT_incorrect);
