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
#include <string.h>

#include "platform.h"

#ifdef USE_TARGET_CONFIG

#include "io/osd.h"
#include "io/vtx.h"
#include "io/ledstrip.h"
#include "io/serial.h"

#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/pid.h"

#include "fc/rc_modes.h"
#include "fc/rc_controls.h"
#include "fc/config.h"
#include "fc/controlrate_profile.h"

#include "rx/rx.h"

#include "drivers/pwm_output.h"

#include "sensors/battery.h"
#include "sensors/gyro.h"

void targetConfiguration(void) {
#ifdef USE_OSD_BEESIGN
    osdConfigMutable()->item_pos[OSD_CRAFT_NAME]        = OSD_POS(6, 9)  | VISIBLE_FLAG;
    osdConfigMutable()->item_pos[OSD_MAIN_BATT_VOLTAGE] = OSD_POS(19, 8) | VISIBLE_FLAG;
    osdConfigMutable()->item_pos[OSD_ITEM_TIMER_2]      = OSD_POS(0,  8) | VISIBLE_FLAG;
    osdConfigMutable()->item_pos[OSD_FLYMODE]           = OSD_POS(14, 8) | VISIBLE_FLAG;
    osdConfigMutable()->item_pos[OSD_VTX_CHANNEL]       = OSD_POS(7,  8) | VISIBLE_FLAG;
    osdConfigMutable()->item_pos[OSD_RSSI_VALUE]        = OSD_POS(0, 9)  | VISIBLE_FLAG;
    osdConfigMutable()->item_pos[OSD_WARNINGS]          = OSD_POS(7, 9);
    osdConfigMutable()->item_pos[OSD_CURRENT_DRAW]      = OSD_POS(18, 9)  | VISIBLE_FLAG;
#else
    osdConfigMutable()->item_pos[OSD_CRAFT_NAME]        = OSD_POS(9, 10) | VISIBLE_FLAG;
    osdConfigMutable()->item_pos[OSD_MAIN_BATT_VOLTAGE] = OSD_POS(23, 9) | VISIBLE_FLAG;
    osdConfigMutable()->item_pos[OSD_ITEM_TIMER_2]      = OSD_POS(2,  9) | VISIBLE_FLAG;
    osdConfigMutable()->item_pos[OSD_FLYMODE]           = OSD_POS(17, 9) | VISIBLE_FLAG;
    osdConfigMutable()->item_pos[OSD_VTX_CHANNEL]       = OSD_POS(9,  9) | VISIBLE_FLAG;
    osdConfigMutable()->item_pos[OSD_RSSI_VALUE]        = OSD_POS(2, 10) | VISIBLE_FLAG;
    osdConfigMutable()->item_pos[OSD_WARNINGS]          = OSD_POS(9, 10);
    osdConfigMutable()->item_pos[OSD_CURRENT_DRAW]      = OSD_POS(22, 10) | VISIBLE_FLAG;
#endif
    batteryConfigMutable()->batteryCapacity = 250;
    batteryConfigMutable()->vbatmincellvoltage = 28;
    batteryConfigMutable()->vbatwarningcellvoltage = 32;
//     vtxSettingsConfigMutable()->band = 5;
//     vtxSettingsConfigMutable()->channel = 8;
//     vtxSettingsConfigMutable()->power = 2;
// #if defined(HMBF4_PRO_US)
//     uint16_t vtxTableFrequency[6][8] = {
//         { 5865, 5845, 5825, 5805, 5785, 5765, 5745, 5725 }, // Boscam A
//         { 5733, 5752, 5771, 5790, 5809, 5828, 5847, 5866 }, // Boscam B
//         { 5705, 5685, 5665,    0, 5885, 5905,    0,    0 }, // Boscam E
//         { 5740, 5760, 5780, 5800, 5820, 5840, 5860, 5880 }, // FatShark
//         { 5658, 5695, 5732, 5769, 5806, 5843, 5880, 5917 }, // RaceBand
//         { 5732, 5765, 5828, 5840, 5866, 5740,    0,    0 }, // IMD6
//     };
// #else
//     uint16_t vtxTableFrequency[6][8] = {
//         { 5865, 5845, 5825, 5805, 5785, 5765, 5745, 5725 }, // Boscam A
//         { 5733, 5752, 5771, 5790, 5809, 5828, 5847, 5866 }, // Boscam B
//         { 5705, 5685, 5665, 5645, 5885, 5905, 5925, 5945 }, // Boscam E
//         { 5740, 5760, 5780, 5800, 5820, 5840, 5860, 5880 }, // FatShark
//         { 5658, 5695, 5732, 5769, 5806, 5843, 5880, 5917 }, // RaceBand
//         { 5732, 5765, 5828, 5840, 5866, 5740,    0,    0 }, // IMD6
//     };
// #endif
//     const char * vtxTableBandNames[6] = {
//             "BOSCAM A",
//             "BOSCAM B",
//             "BOSCAM E",
//             "FATSHARK",
//             "RACEBAND",
//             "IMD6"
//     };
//     char vtxTableBandLetters[7] = "ABEFRI";
    vtxSettingsConfigMutable()->band = 5;
    vtxSettingsConfigMutable()->channel = 8;
    vtxSettingsConfigMutable()->power = 2;
    // for (uint8_t i = 0; i < 6; i++) {
    //     for (uint8_t j = 0; j < 8; j++) {
    //         vtxTableConfigMutable()->frequency[i][j] = vtxTableFrequency[i][j];
    //     }
    // }
    // for (uint8_t i = 0; i < 6; i++) {
    //     strcpy(vtxTableConfigMutable()->bandNames[i], vtxTableBandNames[i]);
    //     vtxTableConfigMutable()->bandLetters[i] = vtxTableBandLetters[i];
    // }
    // strcpy(vtxTableConfigMutable()->channelNames[0], "1");
    // strcpy(vtxTableConfigMutable()->channelNames[1], "2");
    // strcpy(vtxTableConfigMutable()->channelNames[2], "3");
    // strcpy(vtxTableConfigMutable()->channelNames[3], "4");
    // strcpy(vtxTableConfigMutable()->channelNames[4], "5");
    // strcpy(vtxTableConfigMutable()->channelNames[5], "6");
    // strcpy(vtxTableConfigMutable()->channelNames[6], "7");
    // strcpy(vtxTableConfigMutable()->channelNames[7], "8");
    // vtxTableConfigMutable()->powerLevels = 2;
    // vtxTableConfigMutable()->powerValues[0] = 0;
    // vtxTableConfigMutable()->powerValues[1] = 1;
    // strcpy(vtxTableConfigMutable()->powerLabels[0], "5  ");
    // strcpy(vtxTableConfigMutable()->powerLabels[1], "25 ");
    imuConfigMutable()->small_angle = 180;
    modeActivationConditionsMutable(0)->modeId           = BOXARM;
    modeActivationConditionsMutable(0)->auxChannelIndex  = AUX1 - NON_AUX_CHANNEL_COUNT;
    modeActivationConditionsMutable(0)->range.startStep  = CHANNEL_VALUE_TO_STEP(1700);
    modeActivationConditionsMutable(0)->range.endStep    = CHANNEL_VALUE_TO_STEP(2100);
    modeActivationConditionsMutable(1)->modeId           = BOXANGLE;
    modeActivationConditionsMutable(1)->auxChannelIndex  = AUX2 - NON_AUX_CHANNEL_COUNT;
    modeActivationConditionsMutable(1)->range.startStep  = CHANNEL_VALUE_TO_STEP(900);
    modeActivationConditionsMutable(1)->range.endStep    = CHANNEL_VALUE_TO_STEP(1300);
    modeActivationConditionsMutable(2)->modeId           = BOXHORIZON;
    modeActivationConditionsMutable(2)->auxChannelIndex  = AUX2 - NON_AUX_CHANNEL_COUNT;
    modeActivationConditionsMutable(2)->range.startStep  = CHANNEL_VALUE_TO_STEP(1300);
    modeActivationConditionsMutable(2)->range.endStep    = CHANNEL_VALUE_TO_STEP(1700);
    modeActivationConditionsMutable(4)->modeId           = BOXFLIPOVERAFTERCRASH;
    modeActivationConditionsMutable(4)->auxChannelIndex  = AUX3 - NON_AUX_CHANNEL_COUNT;
    modeActivationConditionsMutable(4)->range.startStep  = CHANNEL_VALUE_TO_STEP(1700);
    modeActivationConditionsMutable(4)->range.endStep    = CHANNEL_VALUE_TO_STEP(2100);
    strcpy(pilotConfigMutable()->name, "Humming Bird");
    gyroConfigMutable()->gyro_sync_denom = 1; // This looks like 8k in GUI but actually 3.2k
    gyroConfigMutable()->gyro_lowpass_type = FILTER_PT1;
    gyroConfigMutable()->gyro_lowpass_hz[ROLL] = 200;
    gyroConfigMutable()->gyro_lowpass_hz[PITCH] = 200;
    gyroConfigMutable()->gyro_lowpass_hz[YAW] = 200;
    gyroConfigMutable()->gyro_lowpass2_hz[ROLL] = 250;
    gyroConfigMutable()->gyro_lowpass2_hz[PITCH] = 250;
    gyroConfigMutable()->gyro_lowpass2_hz[YAW] = 250;
    gyroConfigMutable()->yaw_spin_threshold = 1950;
    // gyroConfigMutable()->dyn_lpf_gyro_min_hz = 200;
    // gyroConfigMutable()->dyn_lpf_gyro_max_hz = 500;
    rxConfigMutable()->mincheck = 1075;
    rxConfigMutable()->maxcheck = 1900;
    rxConfigMutable()->rc_smoothing_type = RC_SMOOTHING_TYPE_FILTER;
    rxConfigMutable()->fpvCamAngleDegrees = 0;
    motorConfigMutable()->digitalIdleOffsetValue = 1000;
    motorConfigMutable()->dev.useBurstDshot = true;
    motorConfigMutable()->motorPoleCount = 12;
    motorConfigMutable()->dev.motorPwmProtocol = PWM_TYPE_DSHOT600;
    batteryConfigMutable()->batteryCapacity = 300;
    batteryConfigMutable()->vbatmaxcellvoltage = 44;
    batteryConfigMutable()->vbatfullcellvoltage = 42;
    batteryConfigMutable()->vbatmincellvoltage = 29;
    batteryConfigMutable()->vbatwarningcellvoltage = 32;
    voltageSensorADCConfigMutable(0)->vbatscale = 110;
    mixerConfigMutable()->yaw_motors_reversed = false;
    mixerConfigMutable()->crashflip_motor_percent = 0;
    imuConfigMutable()->small_angle = 180;
    pidConfigMutable()->pid_process_denom = 1;
    pidConfigMutable()->runaway_takeoff_prevention = true;
    osdConfigMutable()->cap_alarm = 2200;
    pidProfilesMutable(0)->dterm_filter_type = FILTER_PT1;
    pidProfilesMutable(0)->dterm_filter2_type = FILTER_PT1;
    // pidProfilesMutable(0)->dyn_lpf_dterm_min_hz = 77;
    // pidProfilesMutable(0)->dyn_lpf_dterm_max_hz = 187;
    // pidProfilesMutable(0)->dterm_lowpass_hz = 150;
    // pidProfilesMutable(0)->dterm_lowpass2_hz = 165;
    // pidProfilesMutable(0)->dterm_notch_cutoff = 0;
    // pidProfilesMutable(0)->vbatPidCompensation = true;
    // pidProfilesMutable(0)->iterm_rotation = true;
    // pidProfilesMutable(0)->itermThrottleThreshold = 250;
    // pidProfilesMutable(0)->yawRateAccelLimit = 0;
    // pidProfilesMutable(0)->iterm_relax = ITERM_RELAX_RPY;
    // pidProfilesMutable(0)->iterm_relax_type = ITERM_RELAX_GYRO;
    // pidProfilesMutable(0)->pidSumLimit = 500;
    // pidProfilesMutable(0)->pidSumLimitYaw = 400;
    // pidProfilesMutable(0)->pid[PID_PITCH].P = 54;
    // pidProfilesMutable(0)->pid[PID_PITCH].I = 75;
    // pidProfilesMutable(0)->pid[PID_PITCH].D = 34;
    // pidProfilesMutable(0)->pid[PID_PITCH].F = 20;
    // pidProfilesMutable(0)->pid[PID_ROLL].P  = 59;
    // pidProfilesMutable(0)->pid[PID_ROLL].I  = 75;
    // pidProfilesMutable(0)->pid[PID_ROLL].D  = 36;
    // pidProfilesMutable(0)->pid[PID_ROLL].F  = 20;
    // pidProfilesMutable(0)->pid[PID_YAW].P   = 46;
    // pidProfilesMutable(0)->pid[PID_YAW].I   = 57;
    // pidProfilesMutable(0)->pid[PID_YAW].D   = 0;
    // pidProfilesMutable(0)->pid[PID_YAW].F   = 0;
    // pidProfilesMutable(0)->pid[PID_LEVEL].P = 70;
    // pidProfilesMutable(0)->pid[PID_LEVEL].I = 70;
    // pidProfilesMutable(0)->pid[PID_LEVEL].D = 100;
    // pidProfilesMutable(0)->levelAngleLimit  = 85;
    // pidProfilesMutable(0)->horizon_tilt_effect = 75;
    // pidProfilesMutable(0)->d_min[FD_ROLL] = 0;
    // pidProfilesMutable(0)->d_min[FD_PITCH] = 0;
    // pidProfilesMutable(0)->d_min_gain = 25;
    // pidProfilesMutable(0)->d_min_advance = 1;
    // pidProfilesMutable(0)->horizon_tilt_expert_mode = false;
    controlRateProfilesMutable(0)->rcRates[FD_YAW] = 100;
    controlRateProfilesMutable(0)->rates[FD_ROLL] = 73;
    controlRateProfilesMutable(0)->rates[FD_PITCH] = 73;
    controlRateProfilesMutable(0)->rates[FD_YAW] = 73;
    controlRateProfilesMutable(0)->rcExpo[FD_ROLL] = 15;
    controlRateProfilesMutable(0)->rcExpo[FD_PITCH] = 15;
    controlRateProfilesMutable(0)->rcExpo[FD_YAW]  = 15;
    // controlRateProfilesMutable(0)->dynThrPID = 65;
    controlRateProfilesMutable(0)->tpa_breakpoint = 1250;
    ledStripConfigMutable()->ledConfigs[0] = DEFINE_LED(7,  7,  8, 0, LF(COLOR), LO(LARSON_SCANNER) | LO(THROTTLE), 0);
    ledStripConfigMutable()->ledConfigs[1] = DEFINE_LED(8,  7, 13, 0, LF(COLOR), LO(LARSON_SCANNER) | LO(THROTTLE), 0);
    ledStripConfigMutable()->ledConfigs[2] = DEFINE_LED(9,  7, 11, 0, LF(COLOR), LO(LARSON_SCANNER) | LO(THROTTLE), 0);
    ledStripConfigMutable()->ledConfigs[3] = DEFINE_LED(10, 7, 4,  0, LF(COLOR), LO(LARSON_SCANNER) | LO(THROTTLE), 0);
    serialConfigMutable()->portConfigs[findSerialPortIndexByIdentifier(SERIAL_PORT_USART1)].functionMask = FUNCTION_VTX_BEESIGN;
}
#endif
