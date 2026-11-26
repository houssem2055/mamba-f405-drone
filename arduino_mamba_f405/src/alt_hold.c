#include "alt_hold.h"
#include <stddef.h>

static float s_alt_m = 0.0f;
static float s_vz_mps = 0.0f;
static float s_target_alt_m = 0.0f;
static float s_vz_int = 0.0f;

static float clampf(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

void AltHold_Init(float initial_alt_m) {
    s_alt_m = initial_alt_m;
    s_vz_mps = 0.0f;
    s_target_alt_m = initial_alt_m;
    s_vz_int = 0.0f;
}

void AltHold_Set_Target(float target_alt_m) {
    s_target_alt_m = target_alt_m;
}

void AltHold_Update(
    bool active,
    float baro_alt_m,
    float az_world_mps2,
    float dt,
    AltHold_State_t *out
) {
    if (!out || dt <= 0.0f) return;

    /* Complementary Baro + Vertical Accel fusion */
    const float k_alt = 0.04f;
    const float k_vz  = 0.02f;
    float pred_alt = s_alt_m + s_vz_mps * dt + 0.5f * az_world_mps2 * dt * dt;
    float pred_vz  = s_vz_mps + az_world_mps2 * dt;
    float err_alt  = baro_alt_m - pred_alt;

    s_alt_m  = pred_alt + k_alt * err_alt;
    s_vz_mps = pred_vz + k_vz * (err_alt / dt);

    out->est_alt_m = s_alt_m;
    out->est_vz_mps = s_vz_mps;
    out->target_alt_m = s_target_alt_m;
    out->active = active;

    if (!active) {
        s_vz_int = 0.0f;
        out->throttle_delta = 0.0f;
        return;
    }

    /* Cascaded Alt P -> Vertical Velocity PI */
    float alt_err = s_target_alt_m - s_alt_m;
    float target_vz = clampf(1.2f * alt_err, -2.5f, 2.5f);
    float vz_err = target_vz - s_vz_mps;

    s_vz_int = clampf(s_vz_int + 0.15f * vz_err * dt, -0.15f, 0.15f);
    out->throttle_delta = clampf(0.18f * vz_err + s_vz_int, -0.25f, 0.25f);
}
