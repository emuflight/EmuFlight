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

#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/imu.h"

#include "rx/rx.h"

#include "rx/cc2500_frsky_common.h"

#include "pg/rx.h"
#include "pg/rx_spi.h"
#include "pg/pinio.h"
#include "pg/piniobox.h"

#include "io/serial.h"
#include "io/osd.h"
#include "io/vtx.h"
#include "io/ledstrip.h"

#include "sensors/battery.h"
#include "sensors/gyro.h"

#include "telemetry/telemetry.h"

#ifdef BRUSHED_MOTORS_PWM_RATE
#undef BRUSHED_MOTORS_PWM_RATE
#endif

#define BRUSHED_MOTORS_PWM_RATE 25000           // 25kHz

#if (defined(BEEBRAIN_PRO_DSM_US) || defined(BEEBRAIN_PRO_DSM_INTL))
#define BB_LITE_RSSI_CH_IDX     9
#endif

void targetConfiguration(void) {
    pinioConfigMutable()->config[0] = PINIO_CONFIG_MODE_OUT_PP;
    pinioBoxConfigMutable()->permanentId[0] = 40;
    flight3DConfigMutable()->neutral3d = 1500;
    flight3DConfigMutable()->deadband3d_high = 1550;
    flight3DConfigMutable()->deadband3d_low = 1450;
    // rxConfigMutable()->midrc = 1500;
    // rxConfigMutable()->mincheck = 1450;
    // rxConfigMutable()->maxcheck = 1550;
    if (hardwareMotorType == MOTOR_BRUSHED) {
        motorConfigMutable()->dev.motorPwmRate = BRUSHED_MOTORS_PWM_RATE;
        motorConfigMutable()->minthrottle = 1030;
        pidConfigMutable()->pid_process_denom = 1;
    }
    for (uint8_t pidProfileIndex = 0; pidProfileIndex < 3; pidProfileIndex++) {
        pidProfile_t *pidProfile = pidProfilesMutable(pidProfileIndex);
        pidProfile->pid[PID_ROLL].P  = 84;
        pidProfile->pid[PID_ROLL].I  = 50;
        pidProfile->pid[PID_ROLL].D  = 58;
        pidProfile->pid[PID_PITCH].P = 87;
        pidProfile->pid[PID_PITCH].I = 55;
        pidProfile->pid[PID_PITCH].D = 58;
        pidProfile->pid[PID_YAW].P   = 110;
        pidProfile->pid[PID_YAW].I   = 75;
        pidProfile->pid[PID_YAW].D   = 25;
    }
    for (uint8_t rateProfileIndex = 0; rateProfileIndex < CONTROL_RATE_PROFILE_COUNT; rateProfileIndex++) {
        controlRateConfig_t *controlRateConfig = controlRateProfilesMutable(rateProfileIndex);
        controlRateConfig->rcRates[FD_YAW] = 100;
        controlRateConfig->rcExpo[FD_ROLL] = 15;
        controlRateConfig->rcExpo[FD_PITCH] = 15;
        controlRateConfig->rcExpo[FD_YAW]  = 15;
        controlRateConfig->rates[FD_ROLL]  = 73;
        controlRateConfig->rates[FD_PITCH] = 73;
        controlRateConfig->rates[FD_YAW] = 73;
    }
    osdConfigMutable()->item_pos[OSD_CRAFT_NAME]        = OSD_POS(9, 10) | VISIBLE_FLAG;
    osdConfigMutable()->item_pos[OSD_MAIN_BATT_VOLTAGE] = OSD_POS(23, 9) | VISIBLE_FLAG;
    osdConfigMutable()->item_pos[OSD_ITEM_TIMER_2]      = OSD_POS(2,  9) | VISIBLE_FLAG;
    osdConfigMutable()->item_pos[OSD_FLYMODE]           = OSD_POS(17, 9) | VISIBLE_FLAG;
    osdConfigMutable()->item_pos[OSD_VTX_CHANNEL]       = OSD_POS(10, 9) | VISIBLE_FLAG;
    osdConfigMutable()->item_pos[OSD_RSSI_VALUE]        = OSD_POS(2, 10) | VISIBLE_FLAG;
    osdConfigMutable()->item_pos[OSD_WARNINGS]          = OSD_POS(9, 10);
    vtxSettingsConfigMutable()->band = 5;
    vtxSettingsConfigMutable()->channel = 1;
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
    modeActivationConditionsMutable(3)->modeId           =  BOXUSER1;
    modeActivationConditionsMutable(3)->auxChannelIndex  = AUX3 - NON_AUX_CHANNEL_COUNT;
    modeActivationConditionsMutable(3)->range.startStep  = CHANNEL_VALUE_TO_STEP(1700);
    modeActivationConditionsMutable(3)->range.endStep    = CHANNEL_VALUE_TO_STEP(2100);
    // modeActivationConditionsMutable(3)->modeId           = BOXAIRMODE;
    // modeActivationConditionsMutable(3)->auxChannelIndex  = AUX2 - NON_AUX_CHANNEL_COUNT;
    // modeActivationConditionsMutable(3)->range.startStep  = CHANNEL_VALUE_TO_STEP(1700);
    // modeActivationConditionsMutable(3)->range.endStep    = CHANNEL_VALUE_TO_STEP(2100);
    ledStripConfigMutable()->ledConfigs[0] = DEFINE_LED(7, 7,  8, 0, LF(COLOR), LO(LARSON_SCANNER) | LO(THROTTLE), 0);
    ledStripConfigMutable()->ledConfigs[1] = DEFINE_LED(8, 7, 13, 0, LF(COLOR), LO(LARSON_SCANNER) | LO(THROTTLE), 0);
    ledStripConfigMutable()->ledConfigs[2] = DEFINE_LED(9, 7, 11, 0, LF(COLOR), LO(LARSON_SCANNER) | LO(THROTTLE), 0);
    strcpy(pilotConfigMutable()->name, "BeeBrain Pro");
#if (defined(BEEBRAIN_PRO_DSM_US) || defined(BEEBRAIN_PRO_DSM_INTL))
    // DSM version
    rxConfigMutable()->rssi_channel = BB_LITE_RSSI_CH_IDX;
    rxFailsafeChannelConfig_t *channelFailsafeConfig = rxFailsafeChannelConfigsMutable(BB_LITE_RSSI_CH_IDX - 1);
    channelFailsafeConfig->mode = RX_FAILSAFE_MODE_SET;
    channelFailsafeConfig->step = CHANNEL_VALUE_TO_RXFAIL_STEP(1000);
    for (uint8_t rxRangeIndex = 0; rxRangeIndex < NON_AUX_CHANNEL_COUNT; rxRangeIndex++) {
        rxChannelRangeConfig_t *channelRangeConfig = rxChannelRangeConfigsMutable(rxRangeIndex);
        channelRangeConfig->min = 1160;
        channelRangeConfig->max = 1840;
    }
#endif
#if (defined(BEEBRAIN_PRO_FRSKY_US) || defined(BEEBRAIN_PRO_FRSKY_INTL))
    do {
        // T8SG
        uint8_t defaultTXHopTable[50] = {0, 30, 60, 91, 120, 150, 180, 210, 5, 35, 65, 95, 125, 155, 185, 215, 10, 40, 70, 100, 130, 160, 190, 221, 15, 45, 75, 105, 135, 165, 195, 225, 20, 50, 80, 110, 140, 170, 200, 230, 25, 55, 85, 115, 145, 175, 205, 0, 0, 0};
        rxFrSkySpiConfigMutable()->bindOffset  = -42;
        rxFrSkySpiConfigMutable()->bindTxId[0] = 0;
        rxFrSkySpiConfigMutable()->bindTxId[1] = 191;
        for (uint8_t i = 0; i < 50; i++) {
            rxFrSkySpiConfigMutable()->bindHopData[i] = defaultTXHopTable[i];
        }
    } while (0);
#endif
}
#endif
