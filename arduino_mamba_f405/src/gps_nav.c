#include "gps_nav.h"
#include <stdlib.h>
#include <string.h>

static char    s_nmea_buf[128];
static uint8_t s_nmea_idx = 0;

static double nmea_to_deg(const char *coord, char hemi) {
    if (!coord || !*coord) return 0.0;
    double raw = atof(coord);
    int deg = (int)(raw / 100.0);
    double mins = raw - (double)(deg * 100);
    double res = (double)deg + mins / 60.0;
    if (hemi == 'S' || hemi == 'W') res = -res;
    return res;
}

static bool parse_gga(char *line, GPS_Data_t *out) {
    /* $GPGGA,time,lat,NS,lon,EW,fix,sats,hdop,alt,M,... */
    char *fields[15];
    int n = 0;
    char *p = line;
    fields[n++] = p;
    while (*p && n < 15) {
        if (*p == ',' || *p == '*') {
            *p = '\0';
            fields[n++] = p + 1;
        }
        p++;
    }
    if (n < 10) return false;

    int fix = atoi(fields[6]);
    int sats = atoi(fields[7]);
    out->fix_valid = (fix > 0 && sats >= 4);
    out->satellites = (uint8_t)sats;
    if (out->fix_valid) {
        out->lat_deg = nmea_to_deg(fields[2], fields[3][0]);
        out->lon_deg = nmea_to_deg(fields[4], fields[5][0]);
        out->alt_m   = (float)atof(fields[9]);
    }
    return true;
}

void GPS_Nav_Init(void) {
    s_nmea_idx = 0;
    memset(s_nmea_buf, 0, sizeof(s_nmea_buf));
}

bool GPS_Nav_Process_Byte(uint8_t byte, GPS_Data_t *out_gps) {
    if (!out_gps) return false;
    if (byte == '$') {
        s_nmea_idx = 0;
    }
    if (byte == '\r' || byte == '\n') {
        if (s_nmea_idx > 6U) {
            s_nmea_buf[s_nmea_idx] = '\0';
            s_nmea_idx = 0;
            if (strncmp(s_nmea_buf, "$GPGGA", 6) == 0 || strncmp(s_nmea_buf, "$GNGGA", 6) == 0) {
                return parse_gga(s_nmea_buf, out_gps);
            }
        }
        s_nmea_idx = 0;
        return false;
    }
    if (s_nmea_idx < (uint8_t)(sizeof(s_nmea_buf) - 1U)) {
        s_nmea_buf[s_nmea_idx++] = (char)byte;
    } else {
        s_nmea_idx = 0;
    }
    return false;
}

void GPS_Nav_To_RTH(const GPS_Data_t *gps, RTH_Nav_Input_t *out_rth) {
    if (!gps || !out_rth) return;
    out_rth->gps_valid   = gps->fix_valid;
    out_rth->lat_deg     = gps->lat_deg;
    out_rth->lon_deg     = gps->lon_deg;
    out_rth->alt_m       = gps->alt_m;
    out_rth->heading_deg = gps->course_deg;
}
