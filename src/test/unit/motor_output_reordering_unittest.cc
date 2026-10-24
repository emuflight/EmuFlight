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

// Expected values come from the mapping's definition: the array must stay a permutation of
// 0..size-1, anything else falls back to the identity mapping.

extern "C" {

#include "platform.h"
#include "drivers/pwm_output.h"

}

#include "unittest_macros.h"
#include "gtest/gtest.h"

static void expectIdentity(const uint8_t *a, unsigned size)
{
    for (unsigned i = 0; i < size; i++) {
        EXPECT_EQ(a[i], i) << "index " << i;
    }
}

TEST(MotorOutputReorderingUnittest, IdentityIsKept)
{
    uint8_t a[8] = {0, 1, 2, 3, 4, 5, 6, 7};
    validateAndfixMotorOutputReordering(a, 8);
    expectIdentity(a, 8);
}

TEST(MotorOutputReorderingUnittest, ValidPermutationIsKept)
{
    uint8_t a[8] = {3, 2, 1, 0, 7, 6, 5, 4};
    const uint8_t expected[8] = {3, 2, 1, 0, 7, 6, 5, 4};
    validateAndfixMotorOutputReordering(a, 8);
    for (int i = 0; i < 8; i++) {
        EXPECT_EQ(a[i], expected[i]);
    }
}

TEST(MotorOutputReorderingUnittest, DuplicateResetsToIdentity)
{
    uint8_t a[8] = {0, 1, 2, 3, 4, 5, 6, 6};
    validateAndfixMotorOutputReordering(a, 8);
    expectIdentity(a, 8);
}

TEST(MotorOutputReorderingUnittest, FirstAndLastDuplicateResetsToIdentity)
{
    uint8_t a[8] = {7, 1, 2, 3, 4, 5, 6, 7};
    validateAndfixMotorOutputReordering(a, 8);
    expectIdentity(a, 8);
}

TEST(MotorOutputReorderingUnittest, OutOfRangeResetsToIdentity)
{
    uint8_t a[8] = {1, 0, 2, 3, 4, 5, 6, 8};
    validateAndfixMotorOutputReordering(a, 8);
    expectIdentity(a, 8);

    uint8_t b[8] = {1, 0, 2, 3, 4, 5, 6, 255};
    validateAndfixMotorOutputReordering(b, 8);
    expectIdentity(b, 8);
}

TEST(MotorOutputReorderingUnittest, AllZeroResetsToIdentity)
{
    uint8_t a[8] = {};
    validateAndfixMotorOutputReordering(a, 8);
    expectIdentity(a, 8);
}

TEST(MotorOutputReorderingUnittest, OtherSizes)
{
    uint8_t one[1] = {0};
    validateAndfixMotorOutputReordering(one, 1);
    EXPECT_EQ(one[0], 0);

    uint8_t oneBad[1] = {1};
    validateAndfixMotorOutputReordering(oneBad, 1);
    EXPECT_EQ(oneBad[0], 0);

    uint8_t twelve[12] = {11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0};
    validateAndfixMotorOutputReordering(twelve, 12);
    EXPECT_EQ(twelve[0], 11);
    EXPECT_EQ(twelve[11], 0);

    uint8_t twelveBad[12] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 10};
    validateAndfixMotorOutputReordering(twelveBad, 12);
    expectIdentity(twelveBad, 12);

    validateAndfixMotorOutputReordering(one, 0);
}
