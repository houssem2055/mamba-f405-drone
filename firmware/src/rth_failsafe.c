#include "rth_failsafe.h"
#include <math.h>
#include <stddef.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static bool   s_home_set = false;
static double s_home_lat = 0.0;
static double s_home_lon = 0.0;
static float  s_home_alt = 0.0f;

static float clampf(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

void RTH_Init(void) {
    s_home_set = false;
    s_home_lat = 0.0;
    s_home_lon = 0.0;
    s_home_alt = 0.0f;
}

void RTH_Set_Home(double lat_deg, double lon_deg, float alt_m) {
    s_home_lat = lat_deg;
    s_home_lon = lon_deg;
    s_home_alt = alt_m;
    s_home_set = true;
}

void RTH_Update(
    bool armed,
    bool rc_ok,
    const RTH_Nav_Input_t *nav,
    RTH_Output_t *out
) {
    if (!out) return;

    out->active = false;
    out->state = RTH_STATE_IDLE;
    out->target_roll_deg = 0.0f;
    out->target_pitch_deg = 0.0f;
    out->target_yaw_deg = nav ? nav->heading_deg : 0.0f;
    out->target_climb_mps = 0.0f;
    out->dist_to_home_m = 0.0f;

    if (!armed || !nav) {
        return;
    }

    /* Record home automatically on first valid GPS fix while RC is OK */
    if (rc_ok && nav->gps_valid && !s_home_set) {
        RTH_Set_Home(nav->lat_deg, nav->lon_deg, nav->alt_m);
    }

    if (rc_ok) {
        return; /* Normal piloted flight */
    }

    /* RC signal lost -> Activate RTH Failsafe */
    out->active = true;

    if (!nav->gps_valid || !s_home_set) {
        /* Safe fallback without GPS: level attitude + controlled descent */
        out->state = RTH_STATE_LAND_IN_PLACE;
        out->target_climb_mps = -0.6f;
        return;
    }

    /* Flat-earth local tangent approximation (meters) */
    double dlat_m = (s_home_lat - nav->lat_deg) * 111320.0;
    double cos_lat = cos(nav->lat_deg * M_PI / 180.0);
    double dlon_m = (s_home_lon - nav->lon_deg) * 111320.0 * cos_lat;
    float dist_m = (float)sqrt(dlat_m * dlat_m + dlon_m * dlon_m);
    out->dist_to_home_m = dist_m;

    float rel_alt = nav->alt_m - s_home_alt;

    if (dist_m <= RTH_ARRIVAL_RADIUS_M) {
        out->state = RTH_STATE_LAND;
        out->target_climb_mps = -0.5f;
        return;
    }

    if (rel_alt < RTH_SAFE_ALT_M) {
        out->state = RTH_STATE_CLIMB;
        out->target_climb_mps = 1.5f;
        return;
    }

    /* Cruise back to home */
    out->state = RTH_STATE_RETURN;
    float bearing_deg = (float)(atan2(dlon_m, dlat_m) * 180.0 / M_PI);
    if (bearing_deg < 0.0f) bearing_deg += 360.0f;
    out->target_yaw_deg = bearing_deg;
    out->target_pitch_deg = -clampf(dist_m * 0.4f, 3.0f, RTH_MAX_TILT_DEG);
}
