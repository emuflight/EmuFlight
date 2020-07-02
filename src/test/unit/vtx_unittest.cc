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

#include <stdint.h>

extern "C" {
    #include "blackbox/blackbox.h"
    #include "build/debug.h"
    #include "common/maths.h"
    #include "config/feature.h"
    #include "pg/pg.h"
    #include "pg/pg_ids.h"
    #include "pg/rx.h"
    #include "fc/config.h"
    #include "fc/controlrate_profile.h"
    #include "fc/fc_core.h"
    #include "fc/rc_controls.h"
    #include "fc/rc_modes.h"
    #include "fc/runtime_config.h"
    #include "flight/failsafe.h"
    #include "flight/imu.h"
    #include "flight/mixer.h"
    #include "flight/pid.h"
    #include "flight/servos.h"
    #include "io/beeper.h"
    #include "io/gps.h"
    #include "io/vtx.h"
    #include "rx/rx.h"
    #include "scheduler/scheduler.h"
    #include "sensors/acceleration.h"
    #include "sensors/gyro.h"
    #include "telemetry/telemetry.h"

    vtxSettingsConfig_t vtxGetSettings(void);

    PG_REGISTER(accelerometerConfig_t, accelerometerConfig, PG_ACCELEROMETER_CONFIG, 0);
    PG_REGISTER(blackboxConfig_t, blackboxConfig, PG_BLACKBOX_CONFIG, 0);
    PG_REGISTER(gyroConfig_t, gyroConfig, PG_GYRO_CONFIG, 0);
    PG_REGISTER(mixerConfig_t, mixerConfig, PG_MIXER_CONFIG, 0);
    PG_REGISTER(pidConfig_t, pidConfig, PG_PID_CONFIG, 0);
    PG_REGISTER(rxConfig_t, rxConfig, PG_RX_CONFIG, 0);
    PG_REGISTER(servoConfig_t, servoConfig, PG_SERVO_CONFIG, 0);
    PG_REGISTER(systemConfig_t, systemConfig, PG_SYSTEM_CONFIG, 0);
    PG_REGISTER(telemetryConfig_t, telemetryConfig, PG_TELEMETRY_CONFIG, 0);
    PG_REGISTER(failsafeConfig_t, failsafeConfig, PG_FAILSAFE_CONFIG, 0);

    float rcCommand[4];
    int16_t rcData[MAX_SUPPORTED_RC_CHANNEL_COUNT];
    uint16_t averageSystemLoadPercent = 0;
    uint8_t cliMode = 0;
    uint8_t debugMode = 0;
    int16_t debug[DEBUG16_VALUE_COUNT];
    pidProfile_t *currentPidProfile;
    controlRateConfig_t *currentControlRateProfile;
    attitudeEulerAngles_t attitude;
    gpsSolutionData_t gpsSol;
    uint32_t targetPidLooptime;
    bool cmsInMenu = false;
    float axisPID_P[3], axisPID_I[3], axisPID_D[3], axisPIDSum[3];
    rxRuntimeConfig_t rxRuntimeConfig = {};
}

uint32_t simulationFeatureFlags = 0;
uint32_t simulationTime = 0;
bool gyroCalibDone = false;
bool simulationHaveRx = false;

#include "gtest/gtest.h"

TEST(VtxTest, PitMode)
{
    // given
    modeActivationConditionsMutable(0)->auxChannelIndex = 0;
    modeActivationConditionsMutable(0)->modeId = BOXVTXPITMODE;
    modeActivationConditionsMutable(0)->range.startStep = CHANNEL_VALUE_TO_STEP(1750);
    modeActivationConditionsMutable(0)->range.endStep = CHANNEL_VALUE_TO_STEP(CHANNEL_RANGE_MAX);

    // and
    vtxSettingsConfigMutable()->band = 0;
    vtxSettingsConfigMutable()->freq = 5800;
    vtxSettingsConfigMutable()->pitModeFreq = 5300;

    // expect
    EXPECT_EQ(5800, vtxGetSettings().freq);

    // and
    // enable vtx pit mode
    rcData[AUX1] = 1800;

    // when
    updateActivatedModes();

    // expect
    EXPECT_TRUE(IS_RC_MODE_ACTIVE(BOXVTXPITMODE));
    EXPECT_EQ(5300, vtxGetSettings().freq);
}

// STUBS
extern "C" {
    uint32_t micros(void) { return simulationTime; }
    uint32_t millis(void) { return micros() / 1000; }
    bool rxIsReceivingSignal(void) { return simulationHaveRx; }

    bool feature(uint32_t f) { return simulationFeatureFlags & f; }
    void warningLedFlash(void) {}
    void warningLedDisable(void) {}
    void warningLedUpdate(void) {}
    void beeper(beeperMode_e) {}
    void beeperConfirmationBeeps(uint8_t) {}
    void beeperWarningBeeps(uint8_t) {}
    void beeperSilence(void) {}
    void systemBeep(bool) {}
    void saveConfigAndNotify(void) {}
    void blackboxFinish(void) {}
    bool accIsCalibrationComplete(void) { return true; }
    bool isBaroCalibrationComplete(void) { return true; }
    bool isGyroCalibrationComplete(void) { return gyroCalibDone; }
    void gyroStartCalibration(bool) {}
    bool isFirstArmingGyroCalibrationRunning(void) { return false; }
    void pidController(const pidProfile_t *, const rollAndPitchTrims_t *, timeUs_t) {}
    void pidStabilisationState(pidStabilisationState_e) {}
    void mixTable(timeUs_t) {};
    void writeMotors(void) {};
    void writeServos(void) {};
    bool calculateRxChannelsAndUpdateFailsafe(timeUs_t) { return true; }
    bool isMixerUsingServos(void) { return false; }
    void gyroUpdate(timeUs_t) {}
    timeDelta_t getTaskDeltaTime(cfTaskId_e) { return 0; }
    void updateRSSI(timeUs_t) {}
    bool failsafeIsMonitoring(void) { return false; }
    void failsafeStartMonitoring(void) {}
    void failsafeUpdateState(void) {}
    bool failsafeIsActive(void) { return false; }
    void pidResetITerm(void) {}
    void updateAdjustmentStates(void) {}
    void processRcAdjustments(controlRateConfig_t *) {}
    void updateGpsWaypointsAndMode(void) {}
    void mspSerialReleaseSharedTelemetryPorts(void) {}
    void telemetryCheckState(void) {}
    void mspSerialAllocatePorts(void) {}
    void gyroReadTemperature(void) {}
    void updateRcCommands(void) {}
    void applyAltHold(void) {}
    void resetYawAxis(void) {}
    int16_t calculateThrottleAngleCorrection(uint8_t) { return 0; }
    void processRcCommand(void) {}
    void updateGpsStateForHomeAndHoldMode(void) {}
    void blackboxUpdate(timeUs_t) {}
    void transponderUpdate(timeUs_t) {}
    void GPS_reset_home_position(void) {}
    void accSetCalibrationCycles(uint16_t) {}
    void baroSetCalibrationCycles(uint16_t) {}
    void changePidProfile(uint8_t) {}
    void changeControlRateProfile(uint8_t) {}
    void dashboardEnablePageCycling(void) {}
    void dashboardDisablePageCycling(void) {}
    bool imuQuaternionHeadfreeOffsetSet(void) { return true; }
    void rescheduleTask(cfTaskId_e, uint32_t) {}
    bool usbCableIsInserted(void) { return false; }
    bool usbVcpIsConnected(void) { return false; }
}
