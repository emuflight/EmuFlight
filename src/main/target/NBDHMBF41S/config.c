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
/*
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#ifdef USE_TARGET_CONFIG

#include "common/axis.h"
#include "common/maths.h"

#include "config/feature.h"

#include "drivers/barometer/barometer.h"
#include "drivers/light_led.h"
#include "drivers/pwm_esc_detect.h"
#include "drivers/pwm_output.h"

#include "fc/config.h"
#include "fc/controlrate_profile.h"
#include "fc/core.h"
#include "fc/rc_adjustments.h"
#include "fc/rc_modes.h"
#include "fc/rc_controls.h"

#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/imu.h"

#include "pg/vcd.h"
#include "pg/rx.h"
#include "pg/motor.h"
#include "pg/vtx_table.h"

#include "rx/rx.h"
#include "rx/cc2500_frsky_common.h"

#include "pg/rx.h"
#include "pg/rx_spi.h"
#include "pg/rx_spi_cc2500.h"
#include "pg/vcd.h"

#include "osd/osd.h"

#include "io/serial.h"
#include "io/vtx.h"
#include "io/ledstrip.h"

#include "sensors/battery.h"
#include "sensors/gyro.h"
#include "sensors/barometer.h"

#include "telemetry/telemetry.h"

void targetConfiguration(void)
{
    osdConfigMutable()->item_pos[OSD_CRAFT_NAME]        = OSD_POS(9, 10) | OSD_PROFILE_1_FLAG;
    osdConfigMutable()->item_pos[OSD_MAIN_BATT_VOLTAGE] = OSD_POS(23, 9) | OSD_PROFILE_1_FLAG;
    osdConfigMutable()->item_pos[OSD_ITEM_TIMER_2]      = OSD_POS(2,  9) | OSD_PROFILE_1_FLAG;
    osdConfigMutable()->item_pos[OSD_FLYMODE]           = OSD_POS(17, 9) | OSD_PROFILE_1_FLAG;
    osdConfigMutable()->item_pos[OSD_VTX_CHANNEL]       = OSD_POS(9,  9) | OSD_PROFILE_1_FLAG;
    osdConfigMutable()->item_pos[OSD_RSSI_VALUE]        = OSD_POS(2, 10) | OSD_PROFILE_1_FLAG;
    osdConfigMutable()->item_pos[OSD_WARNINGS]          = OSD_POS(9, 10);
    osdConfigMutable()->item_pos[OSD_CURRENT_DRAW]      = OSD_POS(22,10) | OSD_PROFILE_1_FLAG;


    batteryConfigMutable()->batteryCapacity = 250;
    batteryConfigMutable()->vbatmincellvoltage = 28;
    batteryConfigMutable()->vbatwarningcellvoltage = 32;

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
    modeActivationConditionsMutable(4)->range.startStep  = CHANNEL_VALUE_TO_STEP(900);
    modeActivationConditionsMutable(4)->range.endStep    = CHANNEL_VALUE_TO_STEP(1300);

    strcpy(pilotConfigMutable()->name, "Humming Bird");

    gyroConfigMutable()->gyro_lowpass_type = FILTER_PT1;
    gyroConfigMutable()->gyro_lowpass_hz = 200;
    gyroConfigMutable()->gyro_lowpass2_hz = 200;
    gyroConfigMutable()->yaw_spin_threshold = 1950;
    gyroConfigMutable()->dyn_lpf_gyro_min_hz = 160;
    gyroConfigMutable()->dyn_lpf_gyro_max_hz = 400;
    rxConfigMutable()->mincheck = 1075;
    rxConfigMutable()->maxcheck = 1900;
    rxConfigMutable()->rc_smoothing_type = RC_SMOOTHING_TYPE_FILTER;
    rxConfigMutable()->fpvCamAngleDegrees = 0;
    rxConfigMutable()->rssi_channel = 9;
    motorConfigMutable()->digitalIdleOffsetValue = 1000;
    motorConfigMutable()->dev.useBurstDshot = true;
    motorConfigMutable()->dev.useDshotTelemetry = false;
    motorConfigMutable()->motorPoleCount = 12;
    motorConfigMutable()->dev.motorPwmProtocol = PWM_TYPE_DSHOT600;
    batteryConfigMutable()->batteryCapacity = 300;
    batteryConfigMutable()->vbatmaxcellvoltage = 450;
    batteryConfigMutable()->vbatfullcellvoltage = 400;
    batteryConfigMutable()->vbatmincellvoltage = 290;
    batteryConfigMutable()->vbatwarningcellvoltage = 320;
    voltageSensorADCConfigMutable(0)->vbatscale = 110;
    mixerConfigMutable()->yaw_motors_reversed = false;
    mixerConfigMutable()->crashflip_motor_percent = 0;
    imuConfigMutable()->small_angle = 180;
    pidConfigMutable()->pid_process_denom = 1;
    pidConfigMutable()->runaway_takeoff_prevention = true;
    //osdConfigMutable()->enabledWarnings &= ~(1 << OSD_WARNING_CORE_TEMPERATURE);
    osdConfigMutable()->cap_alarm = 2200;

    pidProfilesMutable(0)->dterm_filter_type = FILTER_PT1;
    pidProfilesMutable(0)->dyn_lpf_dterm_min_hz = 56;
    pidProfilesMutable(0)->dyn_lpf_dterm_max_hz = 136;
    pidProfilesMutable(0)->dterm_lowpass_hz = 150;
    pidProfilesMutable(0)->dterm_lowpass2_hz = 120;
    pidProfilesMutable(0)->dterm_notch_cutoff = 0;
    pidProfilesMutable(0)->vbatPidCompensation = true;
    pidProfilesMutable(0)->iterm_rotation = true;
    pidProfilesMutable(0)->itermThrottleThreshold = 250;
    pidProfilesMutable(0)->yawRateAccelLimit = 0;
    pidProfilesMutable(0)->pidSumLimit = 500;
    pidProfilesMutable(0)->pidSumLimitYaw = 400;
    pidProfilesMutable(0)->pid[PID_PITCH].P = 55;
    pidProfilesMutable(0)->pid[PID_PITCH].I = 62;
    pidProfilesMutable(0)->pid[PID_PITCH].D = 38;
    pidProfilesMutable(0)->pid[PID_PITCH].F = 20;
    pidProfilesMutable(0)->pid[PID_ROLL].P  = 58;
    pidProfilesMutable(0)->pid[PID_ROLL].I  = 57;
    pidProfilesMutable(0)->pid[PID_ROLL].D  = 35;
    pidProfilesMutable(0)->pid[PID_ROLL].F  = 20;
    pidProfilesMutable(0)->pid[PID_YAW].P   = 48;
    pidProfilesMutable(0)->pid[PID_YAW].I   = 55;
    pidProfilesMutable(0)->pid[PID_YAW].D   = 0;
    pidProfilesMutable(0)->pid[PID_YAW].F   = 0;
    pidProfilesMutable(0)->pid[PID_LEVEL].P = 70;
    pidProfilesMutable(0)->pid[PID_LEVEL].I = 70;
    pidProfilesMutable(0)->pid[PID_LEVEL].D = 100;
    pidProfilesMutable(0)->levelAngleLimit  = 85;
    pidProfilesMutable(0)->horizon_tilt_effect = 75;
    pidProfilesMutable(0)->d_min[FD_ROLL] = 20;
    pidProfilesMutable(0)->d_min[FD_PITCH] = 18;
    pidProfilesMutable(0)->d_min_gain = 25;
    pidProfilesMutable(0)->d_min_advance = 1;
    pidProfilesMutable(0)->horizon_tilt_expert_mode = false;

    controlRateProfilesMutable(0)->rcRates[FD_YAW] = 100;
    controlRateProfilesMutable(0)->rates[FD_ROLL] = 73;
    controlRateProfilesMutable(0)->rates[FD_PITCH] = 73;
    controlRateProfilesMutable(0)->rates[FD_YAW] = 73;
    controlRateProfilesMutable(0)->rcExpo[FD_ROLL] = 15;
    controlRateProfilesMutable(0)->rcExpo[FD_PITCH] = 15;
    controlRateProfilesMutable(0)->rcExpo[FD_YAW]  = 15;
    controlRateProfilesMutable(0)->dynThrPID = 65;
    controlRateProfilesMutable(0)->tpa_breakpoint = 1250;

    ledStripStatusModeConfigMutable()->ledConfigs[0] = DEFINE_LED(7,  7,  8, 0, LF(COLOR), LO(LARSON_SCANNER) | LO(THROTTLE), 0);
    ledStripStatusModeConfigMutable()->ledConfigs[1] = DEFINE_LED(8,  7, 13, 0, LF(COLOR), LO(LARSON_SCANNER) | LO(THROTTLE), 0);
    ledStripStatusModeConfigMutable()->ledConfigs[2] = DEFINE_LED(9,  7, 11, 0, LF(COLOR), LO(LARSON_SCANNER) | LO(THROTTLE), 0);
    ledStripStatusModeConfigMutable()->ledConfigs[3] = DEFINE_LED(10, 7, 4,  0, LF(COLOR), LO(LARSON_SCANNER) | LO(THROTTLE), 0);

    do {
        // T8SG
        uint8_t defaultTXHopTable[50] = {0,30,60,91,120,150,180,210,5,35,65,95,125,155,185,215,10,40,70,100,130,160,190,221,15,45,75,105,135,165,195,225,20,50,80,110,140,170,200,230,25,55,85,115,145,175,205,0,0,0};
        rxCc2500SpiConfigMutable()->bindOffset  = 33;
        rxCc2500SpiConfigMutable()->bindTxId[0] = 198;
        rxCc2500SpiConfigMutable()->bindTxId[1] = 185;
        for (uint8_t i = 0; i < 50; i++) {
            rxCc2500SpiConfigMutable()->bindHopData[i] = defaultTXHopTable[i];
        }
    } while (0);
    serialConfigMutable()->portConfigs[findSerialPortIndexByIdentifier(SERIAL_PORT_USART2)].functionMask = FUNCTION_MSP;
}
#endif
*/
