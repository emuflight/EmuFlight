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
    #include "pg/pg.h"
    #include "pg/pg_ids.h"

    #include "flight/mixer.h"

//PG_DECLARE(motorConfig_t, motorConfig);

PG_REGISTER_WITH_RESET_TEMPLATE(motorConfig_t, motorConfig, PG_MOTOR_CONFIG, 1);

PG_RESET_TEMPLATE(motorConfig_t, motorConfig,
    .dev = {
        .motorPwmRate = 400,
        .motorPwmProtocol = 0,
        .motorPwmInversion = 0,
        .useUnsyncedPwm = 0,
        .useBurstDshot = 0,
        .ioTags = {0}
    },
    .digitalIdleOffsetValue = 0,
    .minthrottle = 1150,
    .maxthrottle = 1850,
    .mincommand = 1000,
    .motorPoleCount = 0
);
}


#include "unittest_macros.h"
#include "gtest/gtest.h"

TEST(ParameterGroupsfTest, Test_pgResetAll)
{
    memset(motorConfigMutable(), 0xFF, sizeof(motorConfig_t));  // Use non-zero sentinel to verify pgResetAll actually initializes all fields
    pgResetAll();
    EXPECT_EQ(1150, motorConfig()->minthrottle);
    EXPECT_EQ(1850, motorConfig()->maxthrottle);
    EXPECT_EQ(1000, motorConfig()->mincommand);
    EXPECT_EQ(400, motorConfig()->dev.motorPwmRate);
    // Verify newly initialized fields
    EXPECT_EQ(0, motorConfig()->digitalIdleOffsetValue);
    EXPECT_EQ(0, motorConfig()->motorPoleCount);
    EXPECT_EQ(0, motorConfig()->dev.motorPwmProtocol);
    EXPECT_EQ(0, motorConfig()->dev.motorPwmInversion);
    EXPECT_EQ(0, motorConfig()->dev.useUnsyncedPwm);
    EXPECT_EQ(0, motorConfig()->dev.useBurstDshot);
    EXPECT_EQ(0, motorConfig()->dev.ioTags[0]);  // Verify ioTags array is zero-initialized
}

TEST(ParameterGroupsfTest, Test_pgFind)
{
    memset(motorConfigMutable(), 0xFF, sizeof(motorConfig_t));  // Use non-zero sentinel to verify pgReset actually initializes all fields
    const pgRegistry_t *pgRegistry = pgFind(PG_MOTOR_CONFIG);
    pgReset(pgRegistry);
    EXPECT_EQ(1150, motorConfig()->minthrottle);
    EXPECT_EQ(1850, motorConfig()->maxthrottle);
    EXPECT_EQ(1000, motorConfig()->mincommand);
    EXPECT_EQ(400, motorConfig()->dev.motorPwmRate);
    // Verify newly initialized fields
    EXPECT_EQ(0, motorConfig()->digitalIdleOffsetValue);
    EXPECT_EQ(0, motorConfig()->motorPoleCount);
    EXPECT_EQ(0, motorConfig()->dev.motorPwmProtocol);
    EXPECT_EQ(0, motorConfig()->dev.motorPwmInversion);
    EXPECT_EQ(0, motorConfig()->dev.useUnsyncedPwm);
    EXPECT_EQ(0, motorConfig()->dev.useBurstDshot);
    EXPECT_EQ(0, motorConfig()->dev.ioTags[0]);  // Verify ioTags array is zero-initialized

    motorConfig_t motorConfig2;
    memset(&motorConfig2, 0xFF, sizeof(motorConfig_t));  // Use non-zero sentinel to verify pgStore actually initializes all fields
    motorConfigMutable()->dev.motorPwmRate = 500;
    pgStore(pgRegistry, &motorConfig2, sizeof(motorConfig_t));
    EXPECT_EQ(1150, motorConfig2.minthrottle);
    EXPECT_EQ(1850, motorConfig2.maxthrottle);
    EXPECT_EQ(1000, motorConfig2.mincommand);
    EXPECT_EQ(500, motorConfig2.dev.motorPwmRate);
    // Verify newly initialized fields in pgStore
    EXPECT_EQ(0, motorConfig2.digitalIdleOffsetValue);
    EXPECT_EQ(0, motorConfig2.motorPoleCount);
    EXPECT_EQ(0, motorConfig2.dev.motorPwmProtocol);
    EXPECT_EQ(0, motorConfig2.dev.motorPwmInversion);
    EXPECT_EQ(0, motorConfig2.dev.useUnsyncedPwm);
    EXPECT_EQ(0, motorConfig2.dev.useBurstDshot);
    EXPECT_EQ(0, motorConfig2.dev.ioTags[0]);  // Verify ioTags array is zero-initialized

    motorConfig_t motorConfig3;
    memset(&motorConfig3, 0xFF, sizeof(motorConfig_t));  // Use non-zero sentinel to verify pgResetCopy actually initializes all fields
    pgResetCopy(&motorConfig3, PG_MOTOR_CONFIG);
    EXPECT_EQ(1150, motorConfig3.minthrottle);
    EXPECT_EQ(1850, motorConfig3.maxthrottle);
    EXPECT_EQ(1000, motorConfig3.mincommand);
    EXPECT_EQ(400, motorConfig3.dev.motorPwmRate);
    // Verify newly initialized fields in pgResetCopy
    EXPECT_EQ(0, motorConfig3.digitalIdleOffsetValue);
    EXPECT_EQ(0, motorConfig3.motorPoleCount);
    EXPECT_EQ(0, motorConfig3.dev.motorPwmProtocol);
    EXPECT_EQ(0, motorConfig3.dev.motorPwmInversion);
    EXPECT_EQ(0, motorConfig3.dev.useUnsyncedPwm);
    EXPECT_EQ(0, motorConfig3.dev.useBurstDshot);
    EXPECT_EQ(0, motorConfig3.dev.ioTags[0]);  // Verify ioTags array is zero-initialized
}

// STUBS

extern "C" {
}
