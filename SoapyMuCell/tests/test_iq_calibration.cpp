// SPDX-License-Identifier: MIT
//
// Unit tests for iq_calibration.hpp. No SoapySDR, ALSA or hardware access,
// so this runs on any machine.

#include "../iq_calibration.hpp"

#include <cassert>
#include <cmath>
#include <cstdio>
#include <vector>

static void test_apply_iq_correction_nulls_known_distortion()
{
    // Synthetic hardware distortion: gain_ratio and phase_corr applied to Q,
    // plus a DC offset on both channels, matching the model in
    // apply_iq_correction's comment.
    const IqCal distortion { 0.05f, -0.03f, 1.08f, 0.07f };

    for (float i_ideal : { -0.6f, 0.0f, 0.3f, 0.9f }) {
        for (float q_ideal : { -0.4f, 0.0f, 0.5f, 0.8f }) {
            float i = i_ideal;
            float q = distortion.gain_ratio *
                (std::cos(distortion.phase_corr) * q_ideal + std::sin(distortion.phase_corr) * i_ideal);
            i += distortion.dc_i;
            q += distortion.dc_q;

            apply_iq_correction(i, q, distortion);

            assert(std::fabs(i - i_ideal) < 1e-4f);
            assert(std::fabs(q - q_ideal) < 1e-4f);
        }
    }
}

static void test_apply_iq_correction_identity_is_noop()
{
    IqCal identity;
    float i = 0.42f, q = -0.17f;
    apply_iq_correction(i, q, identity);
    assert(std::fabs(i - 0.42f) < 1e-6f);
    assert(std::fabs(q - (-0.17f)) < 1e-6f);
}

static void test_bin_power_detects_pure_tone()
{
    const size_t n = 512;
    const double cycles = 16.0;
    std::vector<float> iq(2 * n);
    for (size_t k = 0; k < n; k++) {
        double phase = 2.0 * M_PI * cycles * (double)k / (double)n;
        iq[2*k]   = (float)std::cos(phase);
        iq[2*k+1] = (float)std::sin(phase);
    }

    double power_at_tone = bin_power(iq.data(), n, cycles);
    double power_at_image = bin_power(iq.data(), n, -cycles);
    double power_at_dc = bin_power(iq.data(), n, 0.0);
    double power_at_other = bin_power(iq.data(), n, cycles + 5.0);

    // A unit-amplitude complex exponential at +cycles should have bin power
    // near 1 (magnitude 1, squared), and be far larger than any other bin
    // for a signal with an integer number of cycles in the block.
    assert(power_at_tone > 0.9 && power_at_tone < 1.1);
    assert(power_at_image < 1e-6);
    assert(power_at_dc < 1e-6);
    assert(power_at_other < 1e-6);
}

static void test_pattern_search_2d_finds_quadratic_minimum()
{
    // cost(a, b) minimized at (a, b) = (0.3, -0.2)
    auto cost = [](double a, double b) {
        double da = a - 0.3, db = b + 0.2;
        return da * da + db * db;
    };

    double a = 0.0, b = 0.0;
    pattern_search_2d(a, b, 0.2, 0.2, 12, cost);

    assert(std::fabs(a - 0.3) < 0.02);
    assert(std::fabs(b - (-0.2)) < 0.02);
}

int main()
{
    test_apply_iq_correction_nulls_known_distortion();
    test_apply_iq_correction_identity_is_noop();
    test_bin_power_detects_pure_tone();
    test_pattern_search_2d_finds_quadratic_minimum();
    std::printf("All iq_calibration tests passed.\n");
    return 0;
}
