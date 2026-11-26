#ifndef RTH_FAILSAFE_H
#define RTH_FAILSAFE_H

#include <stdint.h>
#include <stdbool.h>

typedef enum {
    RTH_STATE_IDLE = 0,
    RTH_STATE_CLIMB,
    RTH_STATE_RETURN,
    RTH_STATE_LAND,
    RTH_STATE_LAND_IN_PLACE
} RTH_State_t;

typedef struct {
    bool   gps_valid;
    double lat_deg;
    double lon_deg;
    float  alt_m;
    float  heading_deg;
} RTH_Nav_Input_t;

typedef struct {
    RTH_State_t state;
    bool        active;
    float       target_roll_deg;
    float       target_pitch_deg;
    float       target_yaw_deg;
    float       target_climb_mps;
    float       dist_to_home_m;
} RTH_Output_t;

#define RTH_SAFE_ALT_M       15.0f
#define RTH_ARRIVAL_RADIUS_M  2.0f
#define RTH_MAX_TILT_DEG     12.0f

void RTH_Init(void);
void RTH_Set_Home(double lat_deg, double lon_deg, float alt_m);
void RTH_Update(
    bool armed,
    bool rc_ok,
    const RTH_Nav_Input_t *nav,
    RTH_Output_t *out
);

#endif /* RTH_FAILSAFE_H */
