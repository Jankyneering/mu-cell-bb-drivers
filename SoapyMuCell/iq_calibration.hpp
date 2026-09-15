// SPDX-License-Identifier: MIT
//
// Pure math for I/Q imbalance and DC offset calibration: no SoapySDR, ALSA
// or hardware access here, so it can be unit tested without a device.
#pragma once

#include <cmath>
#include <cstddef>

// Correction to apply to a complex baseband sample stream to undo
// a DC offset plus a gain/phase mismatch between the I and Q channels.
//
// The mismatch is modeled as
//   Q_measured = gain_ratio * (cos(phase_corr) * Q_ideal + sin(phase_corr) * I_ideal)
// on top of an additive (dc_i, dc_q) offset. This is the standard model for
// quadrature gain and phase error used to calibrate zero-IF transceivers.
//
// The same struct and correction function serve both directions:
// on RX it is applied to samples after the ADC to undo the RX chain's own
// mismatch, and on TX it is applied to samples before the DAC, which
// pre-distorts them so that after the TX chain applies its own mismatch
// (of the same form), the transmitted signal comes out undistorted.
struct IqCal {
    float dc_i = 0.0f;
    float dc_q = 0.0f;
    float gain_ratio = 1.0f;
    float phase_corr = 0.0f;
};

inline void apply_iq_correction(float &i, float &q, const IqCal &cal)
{
    const float ci = i - cal.dc_i;
    const float cq = q - cal.dc_q;
    const float cos_p = std::cos(cal.phase_corr);
    const float sin_p = std::sin(cal.phase_corr);
    i = ci;
    q = (cq - cal.gain_ratio * sin_p * ci) / (cal.gain_ratio * cos_p);
}

// Complex single-bin DFT of an interleaved I/Q sample buffer, evaluated at
// the given number of cycles across the whole buffer (may be negative, for
// an image frequency). Used to measure the strength of an injected tone,
// its image, or a DC/LO leakage term (cycles_per_block == 0), during
// calibration. Exact (no spectral leakage) when cycles_per_block is an
// integer, which calibration always arranges for.
struct ComplexBin { double re, im; };

inline ComplexBin dft_bin(const float *iq_interleaved, size_t n_samples, double cycles_per_block)
{
    double re = 0.0, im = 0.0;
    for (size_t n = 0; n < n_samples; n++) {
        const double phase = -2.0 * M_PI * cycles_per_block * (double)n / (double)n_samples;
        const double c = std::cos(phase), s = std::sin(phase);
        const double i = iq_interleaved[2*n];
        const double q = iq_interleaved[2*n + 1];
        re += i * c - q * s;
        im += i * s + q * c;
    }
    return { re / (double)n_samples, im / (double)n_samples };
}

inline double bin_power(const float *iq_interleaved, size_t n_samples, double cycles_per_block)
{
    ComplexBin b = dft_bin(iq_interleaved, n_samples, cycles_per_block);
    return b.re * b.re + b.im * b.im;
}

// Two-parameter pattern search (Hooke-Jeeves style) minimizing cost(a, b).
// Tries +-step_a on a and +-step_b on b, accepts the first improving move,
// halves both steps whenever neither axis improves, for max_rounds rounds.
// No derivative and no tuned learning rate needed, which suits a cost
// function that is expensive to evaluate (each call re-measures through
// real hardware) and only mildly non-quadratic.
template <typename CostFn>
void pattern_search_2d(double &a, double &b, double step_a, double step_b,
                        int max_rounds, CostFn cost)
{
    double best = cost(a, b);
    for (int round = 0; round < max_rounds; round++) {
        bool improved = false;
        for (double d : { step_a, -step_a }) {
            double c = cost(a + d, b);
            if (c < best) { a += d; best = c; improved = true; break; }
        }
        for (double d : { step_b, -step_b }) {
            double c = cost(a, b + d);
            if (c < best) { b += d; best = c; improved = true; break; }
        }
        if (!improved) {
            step_a *= 0.5;
            step_b *= 0.5;
        }
    }
}
