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

#include "common/axis.h"
#include "common/maths.h"

#include "config/feature.h"

#include "drivers/light_led.h"
#include "drivers/pwm_esc_detect.h"
#include "drivers/pwm_output.h"

#include "fc/config.h"
#include "fc/controlrate_profile.h"
#include "fc/rc_modes.h"
#include "fc/rc_controls.h"
#include "fc/rc_adjustments.h"

#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/imu.h"

#include "rx/rx.h"
#include "rx/cc2500_frsky_common.h"

#include "io/serial.h"
#include "io/osd.h"
#include "io/vtx.h"
#include "io/ledstrip.h"

#include "sensors/battery.h"
#include "sensors/gyro.h"
#include "sensors/voltage.h"

#include "telemetry/telemetry.h"

void targetConfiguration(void) {
    // for (uint8_t pidProfileIndex = 0; pidProfileIndex < MAX_PROFILE_COUNT; pidProfileIndex++) {
    //     pidProfile_t *pidProfile = pidProfilesMutable(pidProfileIndex);
    //     pidProfile->pid[PID_ROLL].P  = 86;
    //     pidProfile->pid[PID_ROLL].I  = 50;
    //     pidProfile->pid[PID_ROLL].D  = 60;
    //     pidProfile->pid[PID_PITCH].P = 90;
    //     pidProfile->pid[PID_PITCH].I = 55;
    //     pidProfile->pid[PID_PITCH].D = 60;
    //     pidProfile->pid[PID_YAW].P   = 120;
    //     pidProfile->pid[PID_YAW].I   = 75;
    //     pidProfile->pid[PID_YAW].D   = 20;
    //     pidProfile->dterm_notch_cutoff = 0;
    // }
    // for (uint8_t rateProfileIndex = 0; rateProfileIndex < CONTROL_RATE_PROFILE_COUNT; rateProfileIndex++) {
    //     controlRateConfig_t *controlRateConfig = controlRateProfilesMutable(rateProfileIndex);
    //     controlRateConfig->rcRates[FD_YAW] = 100;
    //     controlRateConfig->rcExpo[FD_ROLL] = 15;
    //     controlRateConfig->rcExpo[FD_PITCH] = 15;
    //     controlRateConfig->rcExpo[FD_YAW]  = 15;
    //     controlRateConfig->rates[FD_ROLL]  = 80;
    //     controlRateConfig->rates[FD_PITCH] = 80;
    //     controlRateConfig->rates[FD_YAW] = 80;
    //     controlRateConfig->dynThrPID = 50;
    // }
    osdConfigMutable()->item_pos[OSD_CRAFT_NAME]        = OSD_POS(9, 10) | VISIBLE_FLAG;
    osdConfigMutable()->item_pos[OSD_MAIN_BATT_VOLTAGE] = OSD_POS(23, 9) | VISIBLE_FLAG;
    osdConfigMutable()->item_pos[OSD_ITEM_TIMER_2]      = OSD_POS(2,  9) | VISIBLE_FLAG;
    osdConfigMutable()->item_pos[OSD_FLYMODE]           = OSD_POS(17, 9) | VISIBLE_FLAG;
    osdConfigMutable()->item_pos[OSD_VTX_CHANNEL]       = OSD_POS(10, 9) | VISIBLE_FLAG;
    osdConfigMutable()->item_pos[OSD_RSSI_VALUE]        = OSD_POS(2, 10) | VISIBLE_FLAG;
    osdConfigMutable()->item_pos[OSD_WARNINGS]          = OSD_POS(9, 10);
    vtxSettingsConfigMutable()->band = 5;
    vtxSettingsConfigMutable()->channel = 8;
    // batteryConfigMutable()->batteryCapacity = 250;
    // batteryConfigMutable()->vbatmincellvoltage = 28;
    // batteryConfigMutable()->vbatwarningcellvoltage = 32;
    // imuConfigMutable()->small_angle = 180;
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
    modeActivationConditionsMutable(3)->modeId           = BOXFPVANGLEMIX;
    modeActivationConditionsMutable(3)->auxChannelIndex  = AUX2 - NON_AUX_CHANNEL_COUNT;
    modeActivationConditionsMutable(3)->range.startStep  = CHANNEL_VALUE_TO_STEP(900);
    modeActivationConditionsMutable(3)->range.endStep    = CHANNEL_VALUE_TO_STEP(1300);
    modeActivationConditionsMutable(4)->modeId           = BOXFLIPOVERAFTERCRASH;
    modeActivationConditionsMutable(4)->auxChannelIndex  = AUX3 - NON_AUX_CHANNEL_COUNT;
    modeActivationConditionsMutable(4)->range.startStep  = CHANNEL_VALUE_TO_STEP(900);
    modeActivationConditionsMutable(4)->range.endStep    = CHANNEL_VALUE_TO_STEP(1300);
    ledStripConfigMutable()->ledConfigs[0] = DEFINE_LED(0, 0,  1, 0, LF(COLOR), 0, 0);
    ledStripConfigMutable()->ledConfigs[1] = DEFINE_LED(1, 0, 10, 0, LF(COLOR), LO(LARSON_SCANNER), 0);
    ledStripConfigMutable()->ledConfigs[2] = DEFINE_LED(2, 0,  2, 0, LF(COLOR), LO(LARSON_SCANNER), 0);
    adjustmentRangesMutable(0)->adjustmentIndex = 1;
    adjustmentRangesMutable(0)->auxChannelIndex = 1;
    adjustmentRangesMutable(0)->range.startStep = CHANNEL_VALUE_TO_STEP(1400);
    adjustmentRangesMutable(0)->range.endStep = CHANNEL_VALUE_TO_STEP(1600);
    adjustmentRangesMutable(0)->adjustmentFunction = 12;
    adjustmentRangesMutable(0)->auxSwitchChannelIndex = 1;
    strcpy(pilotConfigMutable()->name, "BeeBrain BL");
    // rxConfigMutable()->rssi_channel = BB_LITE_RSSI_CH_IDX;
    // rxFailsafeChannelConfig_t *channelFailsafeConfig = rxFailsafeChannelConfigsMutable(BB_LITE_RSSI_CH_IDX - 1);
    // channelFailsafeConfig->mode = RX_FAILSAFE_MODE_SET;
    // channelFailsafeConfig->step = CHANNEL_VALUE_TO_RXFAIL_STEP(1000);
    // for (uint8_t rxRangeIndex = 0; rxRangeIndex < NON_AUX_CHANNEL_COUNT; rxRangeIndex++) {
    //     rxChannelRangeConfig_t *channelRangeConfig = rxChannelRangeConfigsMutable(rxRangeIndex);
    //     rxChannelRangeConfigsMutable(rxRangeIndex)->min = 102;
    //     rxChannelRangeConfigsMutable(rxRangeIndex)->max = 1840;
    // }
    gyroConfigMutable()->gyro_lowpass_type = FILTER_PT2;
    gyroConfigMutable()->gyro_lowpass_hz[ROLL] = 150;
    gyroConfigMutable()->gyro_lowpass_hz[PITCH] = 150;
    gyroConfigMutable()->gyro_lowpass_hz[YAW] = 150;
    gyroConfigMutable()->yaw_spin_threshold = 1400;
    rxConfigMutable()->mincheck = 1004;
    rxConfigMutable()->maxcheck = 2000;
    rxConfigMutable()->rc_smoothing_type = RC_SMOOTHING_TYPE_FILTER;
    rxConfigMutable()->fpvCamAngleDegrees = 12;
    motorConfigMutable()->digitalIdleOffsetValue = 1000;
    motorConfigMutable()->dev.motorPwmProtocol = PWM_TYPE_DSHOT600;
    batteryConfigMutable()->batteryCapacity = 250;
    batteryConfigMutable()->vbatmaxcellvoltage = 46;
    batteryConfigMutable()->vbatfullcellvoltage = 40;
    batteryConfigMutable()->vbatmincellvoltage = 29;
    batteryConfigMutable()->vbatwarningcellvoltage = 29;
    voltageSensorADCConfigMutable(0)->vbatscale = 111;
    mixerConfigMutable()->crashflip_motor_percent = 50;
    imuConfigMutable()->small_angle = 180;
    pidConfigMutable()->pid_process_denom = 1;
    pidConfigMutable()->runaway_takeoff_prevention = false;
    osdConfigMutable()->enabledWarnings &= ~(1 << OSD_WARNING_CORE_TEMPERATURE);
    osdConfigMutable()->cap_alarm = 255;
    pidProfilesMutable(0)->dterm_filter_type = FILTER_PT2;
    pidProfilesMutable(0)->dterm_filter2_type = FILTER_PT2;
    pidProfilesMutable(0)->dFilter[ROLL].dLpf = 200;
    pidProfilesMutable(0)->dFilter[PITCH].dLpf = 200;
    pidProfilesMutable(0)->dFilter[YAW].dLpf = 200;
    pidProfilesMutable(0)->yawRateAccelLimit = 0;
    pidProfilesMutable(0)->pidSumLimit = 1000;
    pidProfilesMutable(0)->pidSumLimitYaw = 1000;
    pidProfilesMutable(0)->pid[PID_PITCH].P = 78;
    pidProfilesMutable(0)->pid[PID_PITCH].I = 75;
    pidProfilesMutable(0)->pid[PID_PITCH].D = 35;
    pidProfilesMutable(0)->pid[PID_PITCH].F = 155;
    pidProfilesMutable(0)->pid[PID_ROLL].P  = 75;
    pidProfilesMutable(0)->pid[PID_ROLL].I  = 70;
    pidProfilesMutable(0)->pid[PID_ROLL].D  = 30;
    pidProfilesMutable(0)->pid[PID_ROLL].F  = 155;
    pidProfilesMutable(0)->pid[PID_YAW].P   = 95;
    pidProfilesMutable(0)->pid[PID_YAW].I   = 70;
    pidProfilesMutable(0)->pid[PID_YAW].F   = 100;
    pidProfilesMutable(0)->levelAngleLimit  = 70;
    pidProfilesMutable(0)->horizon_tilt_effect = 80;
    controlRateProfilesMutable(0)->rcRates[FD_YAW] = 207;
    controlRateProfilesMutable(0)->rates[FD_ROLL] = 80;
    controlRateProfilesMutable(0)->rates[FD_PITCH] = 80;
    controlRateProfilesMutable(0)->rates[FD_YAW] = 25;
    controlRateProfilesMutable(0)->tpa_breakpoint = 1750;
}
#endif
