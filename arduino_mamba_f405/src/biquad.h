#ifndef BIQUAD_H
#define BIQUAD_H

#include <stdint.h>

typedef enum {
    BIQUAD_LPF = 0,
    BIQUAD_NOTCH = 1
} Biquad_Type_t;

typedef struct {
    float b0, b1, b2;
    float a1, a2;
    float z1, z2;
} Biquad_Filter_t;

/**
 * @brief Initialize a 2nd-order Butterworth LPF or Notch filter.
 * @param filter Pointer to filter state
 * @param type   BIQUAD_LPF or BIQUAD_NOTCH
 * @param cutoff_hz Center/cutoff frequency in Hz
 * @param sample_rate_hz Loop frequency in Hz (e.g., 1000.0f)
 * @param q_factor Quality factor (0.70710678f for Butterworth LPF, 2.0-5.0f for Notch)
 */
void Biquad_Init(
    Biquad_Filter_t *filter,
    Biquad_Type_t type,
    float cutoff_hz,
    float sample_rate_hz,
    float q_factor
);

/**
 * @brief Reset filter delay lines to zero (or steady-state value).
 */
void Biquad_Reset(Biquad_Filter_t *filter, float initial_value);

/**
 * @brief Process one sample using Transposed Direct Form II.
 */
float Biquad_Apply(Biquad_Filter_t *filter, float input);

#endif /* BIQUAD_H */
