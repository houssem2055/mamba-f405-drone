#ifndef GPS_NAV_H
#define GPS_NAV_H

#include <stdint.h>
#include <stdbool.h>
#include "rth_failsafe.h"

typedef struct {
    bool    fix_valid;
    uint8_t satellites;
    double  lat_deg;
    double  lon_deg;
    float   alt_m;
    float   speed_mps;
    float   course_deg;
} GPS_Data_t;

void GPS_Nav_Init(void);
bool GPS_Nav_Process_Byte(uint8_t byte, GPS_Data_t *out_gps);
void GPS_Nav_To_RTH(const GPS_Data_t *gps, RTH_Nav_Input_t *out_rth);

#endif /* GPS_NAV_H */
