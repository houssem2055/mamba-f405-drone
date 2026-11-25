#include "biquad.h"
#include <math.h>

#define PI_F 3.14159265358979323846f

void Biquad_Init(
    Biquad_Filter_t *filter,
    Biquad_Type_t type,
    float cutoff_hz,
    float sample_rate_hz,
    float q_factor
) {
    if (!filter || sample_rate_hz <= 0.0f || cutoff_hz <= 0.0f) {
        return;
    }

    /* Prevent Nyquist violation */
    float nyquist = 0.48f * sample_rate_hz;
    if (cutoff_hz > nyquist) {
        cutoff_hz = nyquist;
    }
    if (q_factor < 0.1f) {
        q_factor = 0.70710678f;
    }

    float omega = (2.0f * PI_F * cutoff_hz) / sample_rate_hz;
    float sn = sinf(omega);
    float cs = cosf(omega);
    float alpha = sn / (2.0f * q_factor);

    float b0, b1, b2, a0, a1, a2;

    if (type == BIQUAD_NOTCH) {
        b0 = 1.0f;
        b1 = -2.0f * cs;
        b2 = 1.0f;
        a0 = 1.0f + alpha;
        a1 = -2.0f * cs;
        a2 = 1.0f - alpha;
    } else {
        /* 2nd-order Low-Pass Filter */
        b0 = (1.0f - cs) * 0.5f;
        b1 = 1.0f - cs;
        b2 = (1.0f - cs) * 0.5f;
        a0 = 1.0f + alpha;
        a1 = -2.0f * cs;
        a2 = 1.0f - alpha;
    }

    float inv_a0 = 1.0f / a0;
    filter->b0 = b0 * inv_a0;
    filter->b1 = b1 * inv_a0;
    filter->b2 = b2 * inv_a0;
    filter->a1 = a1 * inv_a0;
    filter->a2 = a2 * inv_a0;
    filter->z1 = 0.0f;
    filter->z2 = 0.0f;
}

void Biquad_Reset(Biquad_Filter_t *filter, float initial_value) {
    if (!filter) return;
    /* Set delay states for DC steady state: z1 = (b1 + b2 - a1 - a2) * x ... */
    if (initial_value == 0.0f) {
        filter->z1 = 0.0f;
        filter->z2 = 0.0f;
    } else {
        filter->z2 = (filter->b2 - filter->a2) * initial_value;
        filter->z1 = (filter->b1 - filter->a1) * initial_value + filter->z2;
    }
}

float Biquad_Apply(Biquad_Filter_t *filter, float input) {
    if (!filter) return input;
    float output = filter->b0 * input + filter->z1;
    filter->z1 = filter->b1 * input - filter->a1 * output + filter->z2;
    filter->z2 = filter->b2 * input - filter->a2 * output;
    return output;
}
