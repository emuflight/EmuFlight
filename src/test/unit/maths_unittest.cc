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
#include <stdbool.h>

#include <limits.h>
#include <cfloat>

#include <math.h>

#define USE_BARO

extern "C" {
    #include "common/maths.h"
}

#include "unittest_macros.h"
#include "gtest/gtest.h"


TEST(MathsUnittest, TestScaleRange)
{
    // Within bounds
    EXPECT_EQ(scaleRange(0, 0, 10, 0, 100), 0);
    EXPECT_EQ(scaleRange(10, 0, 10, 0, 100), 100);
    EXPECT_EQ(scaleRange(0, 0, 100, 0, 10), 0);
    EXPECT_EQ(scaleRange(100, 0, 100, 0, 10), 10);

    // Scale up
    EXPECT_EQ(scaleRange(1, 0, 10, 0, 100), 10);
    EXPECT_EQ(scaleRange(2, 0, 10, 0, 100), 20);
    EXPECT_EQ(scaleRange(5, 0, 10, 0, 100), 50);

    // Scale down
    EXPECT_EQ(scaleRange(10, 0, 100, 0, 10), 1);
    EXPECT_EQ(scaleRange(20, 0, 100, 0, 10), 2);
    EXPECT_EQ(scaleRange(50, 0, 100, 0, 10), 5);
}

TEST(MathsUnittest, TestScaleRangeNegatives)
{
    // Within bounds
    EXPECT_EQ(scaleRange(0, -10, 0, -100, 0), 0);
    EXPECT_EQ(scaleRange(-10, -10, 0, -100, 0), -100);
    EXPECT_EQ(scaleRange(0, -100, 0, -10, 0), 0);
    EXPECT_EQ(scaleRange(-100, -100, 0, -10, 0), -10);

    // Scale up
    EXPECT_EQ(scaleRange(-1, -10, 0, -100, 0), -10);
    EXPECT_EQ(scaleRange(-2, -10, 0, -100, 0), -20);
    EXPECT_EQ(scaleRange(-5, -10, 0, -100, 0), -50);

    // Scale down
    EXPECT_EQ(scaleRange(-10, -100, 0, -10, 0), -1);
    EXPECT_EQ(scaleRange(-20, -100, 0, -10, 0), -2);
    EXPECT_EQ(scaleRange(-50, -100, 0, -10, 0), -5);
}

TEST(MathsUnittest, TestScaleRangeNegativePositive)
{
    // Within bounds
    EXPECT_EQ(scaleRange(0, -10, 0, 0, 100), 100);
    EXPECT_EQ(scaleRange(-10, -10, 0, 0, 100), 0);
    EXPECT_EQ(scaleRange(0, -100, 0, 0, 10), 10);
    EXPECT_EQ(scaleRange(-100, -100, 0, 0, 10), 0);

    // Scale up
    EXPECT_EQ(scaleRange(-1, -10, 0, 0, 100), 90);
    EXPECT_EQ(scaleRange(-2, -10, 0, 0, 100), 80);
    EXPECT_EQ(scaleRange(-5, -10, 0, 0, 100), 50);

    // Scale down
    EXPECT_EQ(scaleRange(-10, -100, 0, 0, 10), 9);
    EXPECT_EQ(scaleRange(-20, -100, 0, 0, 10), 8);
    EXPECT_EQ(scaleRange(-50, -100, 0, 0, 10), 5);
}

TEST(MathsUnittest, TestScaleRangeReverse)
{
    // Within bounds
    EXPECT_EQ(scaleRange(0, 0, 10, 100, 0), 100);
    EXPECT_EQ(scaleRange(10, 0, 10, 100, 0), 0);
    EXPECT_EQ(scaleRange(0, 0, 100, 10, 0), 10);
    EXPECT_EQ(scaleRange(100, 0, 100, 10, 0), 0);

    // Scale up
    EXPECT_EQ(scaleRange(1, 0, 10, 100, 0), 90);
    EXPECT_EQ(scaleRange(2, 0, 10, 100, 0), 80);
    EXPECT_EQ(scaleRange(5, 0, 10, 100, 0), 50);

    // Scale down
    EXPECT_EQ(scaleRange(10, 0, 100, 10, 0), 9);
    EXPECT_EQ(scaleRange(20, 0, 100, 10, 0), 8);
    EXPECT_EQ(scaleRange(50, 0, 100, 10, 0), 5);
}

TEST(MathsUnittest, TestConstrain)
{
    // Within bounds
    EXPECT_EQ(constrain(0, 0, 0), 0);
    EXPECT_EQ(constrain(1, 1, 1), 1);
    EXPECT_EQ(constrain(1, 0, 2), 1);

    // Equal to bottom bound.
    EXPECT_EQ(constrain(1, 1, 2), 1);
    // Equal to top bound.
    EXPECT_EQ(constrain(1, 0, 1), 1);

    // Equal to both bottom and top bound.
    EXPECT_EQ(constrain(1, 1, 1), 1);

    // Above top bound.
    EXPECT_EQ(constrain(2, 0, 1), 1);
    // Below bottom bound.
    EXPECT_EQ(constrain(0, 1, 2), 1);
}

TEST(MathsUnittest, TestConstrainNegatives)
{
    // Within bounds.
    EXPECT_EQ(constrain(-1, -1, -1), -1);
    EXPECT_EQ(constrain(-1, -2, 0), -1);

    // Equal to bottom bound.
    EXPECT_EQ(constrain(-1, -1, 0), -1);
    // Equal to top bound.
    EXPECT_EQ(constrain(-1, -2, -1), -1);

    // Equal to both bottom and top bound.
    EXPECT_EQ(constrain(-1, -1, -1), -1);

    // Above top bound.
    EXPECT_EQ(constrain(-1, -3, -2), -2);
    // Below bottom bound.
    EXPECT_EQ(constrain(-3, -2, -1), -2);
}

TEST(MathsUnittest, TestConstrainf)
{
    // Within bounds.
    EXPECT_FLOAT_EQ(constrainf(1.0f, 0.0f, 2.0f), 1.0f);

    // Equal to bottom bound.
    EXPECT_FLOAT_EQ(constrainf(1.0f, 1.0f, 2.0f), 1.0f);
    // Equal to top bound.
    EXPECT_FLOAT_EQ(constrainf(1.0f, 0.0f, 1.0f), 1.0f);

    // Equal to both bottom and top bound.
    EXPECT_FLOAT_EQ(constrainf(1.0f, 1.0f, 1.0f), 1.0f);

    // Above top bound.
    EXPECT_FLOAT_EQ(constrainf(2.0f, 0.0f, 1.0f), 1.0f);
    // Below bottom bound.
    EXPECT_FLOAT_EQ(constrainf(0, 1.0f, 2.0f), 1.0f);

    // Above bouth bounds.
    EXPECT_FLOAT_EQ(constrainf(2.0f, 0.0f, 1.0f), 1.0f);
    // Below bouth bounds.
    EXPECT_FLOAT_EQ(constrainf(0, 1.0f, 2.0f), 1.0f);
}

TEST(MathsUnittest, TestDegreesToRadians)
{
    EXPECT_FLOAT_EQ(degreesToRadians(0), 0.0f);
    EXPECT_FLOAT_EQ(degreesToRadians(90), 0.5f * M_PIf);
    EXPECT_FLOAT_EQ(degreesToRadians(180), M_PIf);
    EXPECT_FLOAT_EQ(degreesToRadians(-180), - M_PIf);
}

TEST(MathsUnittest, TestApplyDeadband)
{
    EXPECT_EQ(applyDeadband(0, 0), 0);
    EXPECT_EQ(applyDeadband(1, 0), 1);
    EXPECT_EQ(applyDeadband(-1, 0), -1);

    EXPECT_EQ(applyDeadband(0, 10), 0);
    EXPECT_EQ(applyDeadband(1, 10), 0);
    EXPECT_EQ(applyDeadband(10, 10), 0);

    EXPECT_EQ(applyDeadband(11, 10), 1);
    EXPECT_EQ(applyDeadband(-11, 10), -1);
}

void expectVectorsAreEqual(struct fp_vector *a, struct fp_vector *b, float absTol)
{
    EXPECT_NEAR(a->X, b->X, absTol);
    EXPECT_NEAR(a->Y, b->Y, absTol);
    EXPECT_NEAR(a->Z, b->Z, absTol);
}

#if defined(FAST_MATH)
TEST(MathsUnittest, TestFastTrigonometrySinCos)
{
    // Matches BF #14790's own test range/tolerance (+-10*pi).
    double sinError = 0;
    for (float x = -10 * M_PI; x < 10 * M_PI; x += M_PI / 300) {
        double approxResult = sin_approx(x);
        double libmResult = sinf(x);
        sinError = MAX(sinError, fabs(approxResult - libmResult));
    }
    printf("sin_approx maximum absolute error = %e\n", sinError);
    EXPECT_LE(sinError, 3.2e-6);

    double cosError = 0;
    for (float x = -10 * M_PI; x < 10 * M_PI; x += M_PI / 300) {
        double approxResult = cos_approx(x);
        double libmResult = cosf(x);
        cosError = MAX(cosError, fabs(approxResult - libmResult));
    }
    printf("cos_approx maximum absolute error = %e\n", cosError);
    EXPECT_LE(cosError, 3.5e-6);
}

TEST(MathsUnittest, TestFastTrigonometryEdgeCases)
{
    const float epsilon = 3.2e-6f;

    EXPECT_NEAR(sin_approx(0.0f), 0.0f, epsilon);
    EXPECT_NEAR(cos_approx(0.0f), 1.0f, epsilon);

    EXPECT_NEAR(sin_approx(M_PIf), sinf(M_PIf), epsilon);
    EXPECT_NEAR(cos_approx(M_PIf), -1.0f, epsilon);

    EXPECT_NEAR(sin_approx(-M_PIf), sinf(-M_PIf), epsilon);
    EXPECT_NEAR(cos_approx(-M_PIf), -1.0f, epsilon);

    EXPECT_NEAR(sin_approx(0.5f * M_PIf), 1.0f, epsilon);
    EXPECT_NEAR(cos_approx(0.5f * M_PIf), 0.0f, epsilon);

    EXPECT_NEAR(sin_approx(-0.5f * M_PIf), -1.0f, epsilon);
    EXPECT_NEAR(cos_approx(-0.5f * M_PIf), 0.0f, epsilon);

    // Old EF code clamped |x| > 32 rad to 0.0f; this port removes that clamp, so verify large multiples of 2*pi with a magnitude-scaled epsilon (float32 mantissa precision loss dominates at this range).
    for (int k = -50; k <= 50; k += 10) {
        const float x = (float)k * 2.0f * M_PIf;
        const float epsilonAtX = epsilon + fabsf(x) * FLT_EPSILON;
        EXPECT_NEAR(sin_approx(x), sinf(x), epsilonAtX) << "k=" << k << " x=" << x;
        EXPECT_NEAR(cos_approx(x), cosf(x), epsilonAtX) << "k=" << k << " x=" << x;
    }
}

TEST(MathsUnittest, TestSincosfApproxMatchesSeparateCalls)
{
    // Combined-call result must match independent sin_approx/cos_approx calls on the same angle.
    const float epsilon = 1e-6f;
    for (float x = -20 * M_PIf; x < 20 * M_PIf; x += M_PIf / 97) {
        float sinCombined = 0.0f, cosCombined = 0.0f;
        sincosf_approx(x, &sinCombined, &cosCombined);
        EXPECT_NEAR(sinCombined, sin_approx(x), epsilon);
        EXPECT_NEAR(cosCombined, cos_approx(x), epsilon);
    }
}

TEST(MathsUnittest, TestFastTrigonometryATan2)
{
    double error = 0;
    for (float x = -1.0f; x < 1.0f; x += 0.01) {
        for (float y = -1.0f; x < 1.0f; x += 0.001) {
            double approxResult = atan2_approx(y, x);
            double libmResult = atan2f(y, x);
            error = MAX(error, fabs(approxResult - libmResult));
        }
    }
    printf("atan2_approx maximum absolute error = %e rads (%e degree)\n", error, error / M_PI * 180.0f);
    EXPECT_LE(error, 1e-6);
}

TEST(MathsUnittest, TestFastTrigonometryACos)
{
    double error = 0;
    for (float x = -1.0f; x < 1.0f; x += 0.001) {
        double approxResult = acos_approx(x);
        double libmResult = acos(x);
        error = MAX(error, fabs(approxResult - libmResult));
    }
    printf("acos_approx maximum absolute error = %e rads (%e degree)\n", error, error / M_PI * 180.0f);
    EXPECT_LE(error, 1e-4);
}
#endif // defined(FAST_MATH)
