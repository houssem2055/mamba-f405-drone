#ifndef ALT_HOLD_H
#define ALT_HOLD_H

#include <stdbool.h>

typedef struct {
    float est_alt_m;
    float est_vz_mps;
    float target_alt_m;
    float throttle_delta; /* [-0.25 .. +0.25] normalized adjustment */
    bool  active;
} AltHold_State_t;

void AltHold_Init(float initial_alt_m);
void AltHold_Set_Target(float target_alt_m);
void AltHold_Update(
    bool active,
    float baro_alt_m,
    float az_world_mps2,
    float dt,
    AltHold_State_t *out
);

#endif /* ALT_HOLD_H */
