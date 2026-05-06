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

#include <math.h>

extern "C" {
    #include "common/filter.h"
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

TEST(FilterUnittest, TestPt1FilterInit)
{
    pt1Filter_t filter;
    pt1FilterInit(&filter, 0.0f);
    EXPECT_EQ(0, filter.k);

    pt1FilterInit(&filter, 1.0f);
    EXPECT_EQ(1.0, filter.k);
}

TEST(FilterUnittest, TestPt1FilterGain)
{
    EXPECT_FLOAT_EQ(0.999949, pt1FilterGain(100, 31.25f));
    // handle cases over uint8_t boundary
    EXPECT_FLOAT_EQ(0.99998301, pt1FilterGain(300, 31.25f));
}

TEST(FilterUnittest, TestPt1FilterApply)
{
    pt1Filter_t filter;
    pt1FilterInit(&filter, pt1FilterGain(100, 31.25f));
    EXPECT_EQ(0, filter.state);

    pt1FilterApply(&filter, 1800.0f);
    EXPECT_FLOAT_EQ(1799.9083, filter.state);

    pt1FilterApply(&filter, -1800.0f);
    EXPECT_FLOAT_EQ(-1799.8165, filter.state);

    pt1FilterApply(&filter, -200.0f);
    EXPECT_FLOAT_EQ(-200.08142, filter.state);
}

TEST(FilterUnittest, TestSlewFilterInit)
{
    slewFilter_t filter;

    slewFilterInit(&filter, 0.0f, 0.0f);
    EXPECT_EQ(0, filter.state);
    EXPECT_EQ(0, filter.slewLimit);
    EXPECT_EQ(0, filter.threshold);

    slewFilterInit(&filter, 1800.0f, 1900.0f);
    EXPECT_EQ(0, filter.state);
    EXPECT_EQ(1800, filter.slewLimit);
    EXPECT_EQ(1900, filter.threshold);
}

TEST(FilterUnittest, TestSlewFilter)
{
    slewFilter_t filter;
    slewFilterInit(&filter, 2000.0f, 1900.0f);
    EXPECT_EQ(0, filter.state);
    EXPECT_EQ(2000, filter.slewLimit);
    EXPECT_EQ(1900, filter.threshold);

    slewFilterApply(&filter, 1800.0f);
    EXPECT_EQ(1800, filter.state);
    slewFilterApply(&filter, -1800.0f);
    EXPECT_EQ(-1800, filter.state);
    slewFilterApply(&filter, -200.0f);
    EXPECT_EQ(-200, filter.state);

    slewFilterApply(&filter, 1900.0f);
    EXPECT_EQ(1900, filter.state);
    slewFilterApply(&filter, -2000.0f);
    EXPECT_EQ(1900, filter.state);
    slewFilterApply(&filter, -200.0f);
    EXPECT_EQ(1900, filter.state);
    slewFilterApply(&filter, 1800.0f);
    EXPECT_EQ(1800, filter.state);
    slewFilterApply(&filter, -200.0f);
    EXPECT_EQ(-200, filter.state);

    slewFilterApply(&filter, -1900.0f);
    EXPECT_EQ(-1900, filter.state);
    slewFilterApply(&filter, 2000.0f);
    EXPECT_EQ(-1900, filter.state);
    slewFilterApply(&filter, 200.0f);
    EXPECT_EQ(-1900, filter.state);
    slewFilterApply(&filter, -1800.0f);
    EXPECT_EQ(-1800, filter.state);
    slewFilterApply(&filter, 200.0f);
    EXPECT_EQ(200, filter.state);
}

// ---- Biquad filter tests ----

TEST(FilterUnittest, TestBiquadFilterLPFInit)
{
    biquadFilter_t filter;
    // 100 Hz LPF, 1 kHz loop rate (1000 µs)
    biquadFilterInitLPF(&filter, 100.0f, 1000);

    // For a Butterworth LPF the DC gain must be 1.0: sum(b) / (1 + sum(a)) == 1
    // In code: a1 and a2 are stored with their sign as used in the update equation,
    // so DC gain = (b0+b1+b2) / (1 - (-a1) - (-a2)) = (b0+b1+b2) / (1+a1+a2)  ... but
    // a1 is stored as -2*cs/a0, a negative number, so (1+a1+a2) with stored values.
    // Easier: verify x and y states are zeroed after init
    EXPECT_FLOAT_EQ(0.0f, filter.x1);
    EXPECT_FLOAT_EQ(0.0f, filter.x2);
    EXPECT_FLOAT_EQ(0.0f, filter.y1);
    EXPECT_FLOAT_EQ(0.0f, filter.y2);

    // Coefficients must be non-trivial (filter is actually doing something)
    EXPECT_NE(0.0f, filter.b0);
    EXPECT_NE(0.0f, filter.b1);
    EXPECT_NE(0.0f, filter.b2);
}

TEST(FilterUnittest, TestBiquadFilterLPFApplyDC)
{
    biquadFilter_t filter;
    biquadFilterInitLPF(&filter, 100.0f, 1000);

    // Apply DC input (constant 1.0) for 200 samples; output must converge to 1.0
    float output = 0.0f;
    for (int i = 0; i < 200; i++) {
        output = biquadFilterApply(&filter, 1.0f);
    }
    EXPECT_NEAR(1.0f, output, 0.001f);

    // Apply DC input (constant -500.0) for 200 samples; output must converge to -500.0
    biquadFilterInitLPF(&filter, 100.0f, 1000);
    for (int i = 0; i < 200; i++) {
        output = biquadFilterApply(&filter, -500.0f);
    }
    EXPECT_NEAR(-500.0f, output, 0.5f);
}

TEST(FilterUnittest, TestBiquadFilterLPFAttenuation)
{
    biquadFilter_t filter;
    // 100 Hz Butterworth LPF at 1 kHz: LPF strongly attenuates Nyquist (500 Hz), gain approaches 0
    biquadFilterInitLPF(&filter, 100.0f, 1000);

    // Feed alternating ±1000 (Nyquist-frequency signal) for 100 cycles to settle
    float output = 0.0f;
    for (int i = 0; i < 200; i++) {
        const float input = (i % 2 == 0) ? 1000.0f : -1000.0f;
        output = biquadFilterApply(&filter, input);
    }
    // Output amplitude must be negligible vs input amplitude of 1000
    EXPECT_NEAR(0.0f, output, 1.0f)
        << "LPF should strongly attenuate Nyquist-frequency input; output=" << output;
}

TEST(FilterUnittest, TestBiquadFilterNotchDCPass)
{
    // A notch filter passes DC (gain = 1 at f=0)
    biquadFilter_t filter;
    const float notchHz  = 200.0f;
    const float cutoffHz = 180.0f;
    const float Q = filterGetNotchQ(notchHz, cutoffHz);
    biquadFilterInit(&filter, notchHz, 1000, Q, FILTER_NOTCH);

    float output = 0.0f;
    for (int i = 0; i < 200; i++) {
        output = biquadFilterApply(&filter, 1.0f);
    }
    EXPECT_NEAR(1.0f, output, 0.001f)
        << "Notch filter must pass DC (f=0) with gain~1; output=" << output;
}

TEST(FilterUnittest, TestFilterGetNotchQ)
{
    // Q = f0 * f1 / (f0^2 - f1^2)
    const float q = filterGetNotchQ(200.0f, 180.0f);
    // 200*180 / (40000-32400) = 36000/7600 ≈ 4.7368
    EXPECT_NEAR(4.7368f, q, 0.001f);

    // Symmetric check: higher cutoff → higher Q (narrower notch)
    const float qNarrow = filterGetNotchQ(200.0f, 195.0f);
    EXPECT_GT(qNarrow, q);
}

// ---- PTn filter tests ----

TEST(FilterUnittest, TestPtnFilterInit)
{
    ptnFilter_t filter;
    // PT2, 100 Hz, 1 kHz loop (dT=0.001)
    ptnFilterInit(&filter, 2, 100, 0.001f);

    EXPECT_EQ(2, filter.order);
    EXPECT_NE(0.0f, filter.k);  // k must be computed and non-zero

    // ptnFilterInit zeroes state[1..order] (the filter taps); state[0] is the input slot
    for (int i = 1; i <= filter.order; i++) {
        EXPECT_FLOAT_EQ(0.0f, filter.state[i]);
    }
}

TEST(FilterUnittest, TestPtnFilterPT2Apply)
{
    ptnFilter_t filter;
    ptnFilterInit(&filter, 2, 100, 0.001f);  // PT2, 100 Hz, dT=1ms

    // Apply DC input 1.0 for 100 samples; output should converge to 1.0
    float output = 0.0f;
    for (int i = 0; i < 100; i++) {
        output = ptnFilterApply(&filter, 1.0f);
    }
    EXPECT_NEAR(1.0f, output, 0.01f)
        << "PT2 filter should converge to DC input after 100 samples; output=" << output;
}

TEST(FilterUnittest, TestPtnFilterPT3Apply)
{
    ptnFilter_t filter;
    ptnFilterInit(&filter, 3, 100, 0.001f);  // PT3, 100 Hz, dT=1ms

    float output = 0.0f;
    for (int i = 0; i < 100; i++) {
        output = ptnFilterApply(&filter, 1.0f);
    }
    EXPECT_NEAR(1.0f, output, 0.01f)
        << "PT3 filter should converge to DC input after 100 samples; output=" << output;
}

TEST(FilterUnittest, TestPtnFilterStepAttenuation)
{
    ptnFilter_t filterPT1;
    ptnFilter_t filterPT2;
    ptnFilterInit(&filterPT1, 1, 100, 0.001f);
    ptnFilterInit(&filterPT2, 2, 100, 0.001f);

    // On first sample of a step: PT2 should be more attenuated than PT1
    const float outPT1 = ptnFilterApply(&filterPT1, 1000.0f);
    const float outPT2 = ptnFilterApply(&filterPT2, 1000.0f);
    EXPECT_LT(fabsf(outPT2), fabsf(outPT1))
        << "PT2 step response should be more attenuated than PT1 on first sample"
        << " PT1=" << outPT1 << " PT2=" << outPT2;
}

// ---- AlphaBetaGamma (ABG) filter tests ----

TEST(FilterUnittest, TestABGFilterInit)
{
    alphaBetaGammaFilter_t filter;
    // alpha=300 (0.3 effective), no boost, no half-life, dT=1ms
    ABGInit(&filter, 300, 0, 0, 0.001f);

    // State must start at zero
    EXPECT_FLOAT_EQ(0.0f, filter.xk);
    EXPECT_FLOAT_EQ(0.0f, filter.vk);
    EXPECT_FLOAT_EQ(0.0f, filter.ak);
    EXPECT_FLOAT_EQ(0.0f, filter.jk);

    // Internal gains must be non-trivial
    EXPECT_NE(0.0f, filter.a);
    EXPECT_NE(0.0f, filter.b);
    EXPECT_NE(0.0f, filter.g);

    // dT fields must be set
    EXPECT_FLOAT_EQ(0.001f, filter.dT);
}

TEST(FilterUnittest, TestABGFilterApplyDC)
{
    alphaBetaGammaFilter_t filter;
    ABGInit(&filter, 300, 0, 0, 0.001f);

    // Apply constant DC input 100.0 for 1000 samples; ABG should track to 100.0
    float output = 0.0f;
    for (int i = 0; i < 1000; i++) {
        output = alphaBetaGammaApply(&filter, 100.0f);
    }
    EXPECT_NEAR(100.0f, output, 1.0f)
        << "ABG filter should converge to constant DC input; output=" << output;
}

TEST(FilterUnittest, TestABGFilterStepResponse)
{
    alphaBetaGammaFilter_t filter;
    ABGInit(&filter, 300, 0, 0, 0.001f);

    // ABG is a predictive tracker: it reacts quickly but may overshoot before settling.
    // Verify: output responds meaningfully to the step and stays within a reasonable range.
    float output = 0.0f;
    for (int i = 0; i < 50; i++) {
        output = alphaBetaGammaApply(&filter, 100.0f);
    }
    // After 50 samples the filter must have reacted: output should be above 50% of target
    EXPECT_GT(output, 50.0f)
        << "ABG should have exceeded 50% of step target after 50 samples; output=" << output;
    // And should not have diverged far from the target (within 50% overshoot)
    EXPECT_LT(output, 150.0f)
        << "ABG output should stay within 150% of target; output=" << output;
}
