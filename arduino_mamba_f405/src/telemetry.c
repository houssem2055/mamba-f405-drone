#include "telemetry.h"
#include "config.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

extern void UART6_SendString(const char *str);
__attribute__((weak)) void System_Enter_DFU_Bootloader(void) {}

static char    s_cmd_buf[96];
static uint8_t s_cmd_idx = 0;

void Telemetry_Init(void) {
    s_cmd_idx = 0;
    memset(s_cmd_buf, 0, sizeof(s_cmd_buf));
}

static void Fmt_Float(char *dst, size_t dst_sz, float val, uint8_t decimals) {
    if (!dst || dst_sz == 0U) return;
    if (val != val) {
        snprintf(dst, dst_sz, "0");
        return;
    }
    if (val > 1000000.0f) {
        snprintf(dst, dst_sz, "999999");
        return;
    }
    if (val < -1000000.0f) {
        snprintf(dst, dst_sz, "-999999");
        return;
    }

    static const uint32_t pow10_tbl[6] = {1UL, 10UL, 100UL, 1000UL, 10000UL, 100000UL};
    if (decimals > 5U) decimals = 5U;
    uint32_t scale = pow10_tbl[decimals];

    bool neg = (val < 0.0f);
    double abs_v = neg ? -(double)val : (double)val;
    uint64_t scaled = (uint64_t)(abs_v * (double)scale + 0.5);
    if (scaled == 0ULL) {
        neg = false;
    }

    unsigned int whole = (unsigned int)(scaled / scale);
    unsigned int frac  = (unsigned int)(scaled % scale);
    if (whole > 9999999U) whole = 9999999U;
    if (frac  >= 100000U) frac  = 99999U;

    switch (decimals) {
        case 0U: snprintf(dst, dst_sz, "%s%u",      neg ? "-" : "", whole);       break;
        case 1U: snprintf(dst, dst_sz, "%s%u.%01u", neg ? "-" : "", whole, frac); break;
        case 2U: snprintf(dst, dst_sz, "%s%u.%02u", neg ? "-" : "", whole, frac); break;
        case 3U: snprintf(dst, dst_sz, "%s%u.%03u", neg ? "-" : "", whole, frac); break;
        case 4U: snprintf(dst, dst_sz, "%s%u.%04u", neg ? "-" : "", whole, frac); break;
        default: snprintf(dst, dst_sz, "%s%u.%05u", neg ? "-" : "", whole, frac); break;
    }
}

void Telemetry_Send_Frame(
    const Attitude_t *att,
    const Motor_Outputs_t *motors,
    float vbat,
    bool armed,
    uint8_t mode
) {
    if (!att || !motors) return;

    char f_r[20], f_p[20], f_y[20], f_gx[20], f_gy[20], f_gz[20];
    char f_m1[20], f_m2[20], f_m3[20], f_m4[20], f_vb[20];
    Fmt_Float(f_r,  sizeof(f_r),  att->roll_deg, 2U);
    Fmt_Float(f_p,  sizeof(f_p),  att->pitch_deg, 2U);
    Fmt_Float(f_y,  sizeof(f_y),  att->yaw_deg, 2U);
    Fmt_Float(f_gx, sizeof(f_gx), att->gx_filt, 3U);
    Fmt_Float(f_gy, sizeof(f_gy), att->gy_filt, 3U);
    Fmt_Float(f_gz, sizeof(f_gz), att->gz_filt, 3U);
    Fmt_Float(f_m1, sizeof(f_m1), motors->m1, 0U);
    Fmt_Float(f_m2, sizeof(f_m2), motors->m2, 0U);
    Fmt_Float(f_m3, sizeof(f_m3), motors->m3, 0U);
    Fmt_Float(f_m4, sizeof(f_m4), motors->m4, 0U);
    Fmt_Float(f_vb, sizeof(f_vb), vbat, 2U);

    char buffer[320];
    /* Format: $TLM,roll,pitch,yaw,gx,gy,gz,m1,m2,m3,m4,vbat,armed,mode\r\n */
    int len = snprintf(buffer, sizeof(buffer),
        "$TLM,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%u,%u\r\n",
        f_r, f_p, f_y, f_gx, f_gy, f_gz, f_m1, f_m2, f_m3, f_m4, f_vb,
        armed ? 1U : 0U, (unsigned int)mode
    );

    if (len > 0) {
        UART6_SendString(buffer);
    }
}

void Telemetry_Send_AI_Frame(const Attitude_t *att, const AI_Diagnostics_t *ai) {
    if (!att || !ai) return;

    char f_h[20], f_dur[20], f_dup[20], f_jit[20], f_asy[20];
    Fmt_Float(f_h,   sizeof(f_h),   ai->health_score, 3U);
    Fmt_Float(f_dur, sizeof(f_dur), att->ai_du[0], 4U);
    Fmt_Float(f_dup, sizeof(f_dup), att->ai_du[1], 4U);
    Fmt_Float(f_jit, sizeof(f_jit), ai->feat_gyro_jitter, 3U);
    Fmt_Float(f_asy, sizeof(f_asy), ai->feat_motor_asym, 1U);

    char buffer[200];
    /* Format: $AI,health,fault_cls,du_r,du_p,gyro_jitter,motor_asym,ai_on\r\n */
    int len = snprintf(buffer, sizeof(buffer),
        "$AI,%s,%u,%s,%s,%s,%s,%u\r\n",
        f_h, (unsigned int)ai->fault_class, f_dur, f_dup, f_jit, f_asy,
        Flight_Controller_Get_AI_Enabled() ? 1U : 0U
    );

    if (len > 0) {
        UART6_SendString(buffer);
    }
}

void Telemetry_Send_Nav_Frame(const RTH_Output_t *rth, const Safety_Status_t *safety, float est_alt_m, uint8_t gps_sats) {
    if (!rth || !safety) return;

    char f_alt[20], f_dist[20];
    Fmt_Float(f_alt,  sizeof(f_alt),  est_alt_m, 2U);
    Fmt_Float(f_dist, sizeof(f_dist), rth->dist_to_home_m, 1U);

    char buffer[160];
    /* Format: $NAV,alt_m,dist_home_m,rth_state,safety_flags,gps_sats\r\n */
    int len = snprintf(buffer, sizeof(buffer),
        "$NAV,%s,%s,%u,%u,%u\r\n",
        f_alt, f_dist,
        (unsigned int)rth->state,
        (unsigned int)safety->flags,
        (unsigned int)gps_sats
    );

    if (len > 0) {
        UART6_SendString(buffer);
    }
}

void Telemetry_Send_PID_Gains(void) {
    PID_Gains_t g;
    Flight_Controller_Get_Gains(&g);

    char f[13][20];
    Fmt_Float(f[0],  sizeof(f[0]),  g.Kp_angle, 3U);
    Fmt_Float(f[1],  sizeof(f[1]),  g.Kp_p, 4U);
    Fmt_Float(f[2],  sizeof(f[2]),  g.Ki_p, 4U);
    Fmt_Float(f[3],  sizeof(f[3]),  g.Kd_p, 5U);
    Fmt_Float(f[4],  sizeof(f[4]),  g.Kf_p, 4U);
    Fmt_Float(f[5],  sizeof(f[5]),  g.Kp_q, 4U);
    Fmt_Float(f[6],  sizeof(f[6]),  g.Ki_q, 4U);
    Fmt_Float(f[7],  sizeof(f[7]),  g.Kd_q, 5U);
    Fmt_Float(f[8],  sizeof(f[8]),  g.Kf_q, 4U);
    Fmt_Float(f[9],  sizeof(f[9]),  g.Kp_r, 4U);
    Fmt_Float(f[10], sizeof(f[10]), g.Ki_r, 4U);
    Fmt_Float(f[11], sizeof(f[11]), g.Kd_r, 5U);
    Fmt_Float(f[12], sizeof(f[12]), g.Kf_r, 4U);

    char buffer[320];
    int len = snprintf(buffer, sizeof(buffer),
        "$PID,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\r\n",
        f[0], f[1], f[2], f[3], f[4], f[5], f[6],
        f[7], f[8], f[9], f[10], f[11], f[12]
    );

    if (len > 0) {
        UART6_SendString(buffer);
    }
}

static void Handle_Command_Line(char *line) {
    if (strcmp(line, "$CMD,PING") == 0) {
        UART6_SendString("$PONG,MAMBA_F405_AI_V3\r\n");
        return;
    }
    if (strcmp(line, "$CMD,GET_PID") == 0) {
        Telemetry_Send_PID_Gains();
        return;
    }
    if (strcmp(line, "$CMD,AI_ON") == 0) {
        Flight_Controller_Set_AI(true, true);
        UART6_SendString("$ACK,AI_ON\r\n");
        return;
    }
    if (strcmp(line, "$CMD,AI_OFF") == 0) {
        Flight_Controller_Set_AI(false, false);
        UART6_SendString("$ACK,AI_OFF\r\n");
        return;
    }
    if (strcmp(line, "$CMD,AI_RESET") == 0) {
        Flight_Controller_Reset_AI();
        UART6_SendString("$ACK,AI_RESET\r\n");
        return;
    }
    if (strcmp(line, "$CMD,BOOTLOADER") == 0 || strcmp(line, "$CMD,DFU") == 0) {
        UART6_SendString("$ACK,BOOTLOADER\r\n");
        System_Enter_DFU_Bootloader();
        return;
    }

    if (strncmp(line, "$SET,", 5) == 0) {
        char *param = line + 5;
        char *comma = strchr(param, ',');
        if (!comma) return;
        *comma = '\0';
        float val = (float)atof(comma + 1);

        PID_Gains_t g;
        Flight_Controller_Get_Gains(&g);

        if      (strcmp(param, "KP_ANGLE") == 0) g.Kp_angle = val;
        else if (strcmp(param, "KP_P") == 0)     g.Kp_p = val;
        else if (strcmp(param, "KI_P") == 0)     g.Ki_p = val;
        else if (strcmp(param, "KD_P") == 0)     g.Kd_p = val;
        else if (strcmp(param, "KF_P") == 0)     g.Kf_p = val;
        else if (strcmp(param, "KP_Q") == 0)     g.Kp_q = val;
        else if (strcmp(param, "KI_Q") == 0)     g.Ki_q = val;
        else if (strcmp(param, "KD_Q") == 0)     g.Kd_q = val;
        else if (strcmp(param, "KF_Q") == 0)     g.Kf_q = val;
        else if (strcmp(param, "KP_R") == 0)     g.Kp_r = val;
        else if (strcmp(param, "KI_R") == 0)     g.Ki_r = val;
        else if (strcmp(param, "KD_R") == 0)     g.Kd_r = val;
        else if (strcmp(param, "KF_R") == 0)     g.Kf_r = val;
        else {
            UART6_SendString("$ERR,UNKNOWN_PARAM\r\n");
            return;
        }

        Flight_Controller_Set_Gains(&g);
        UART6_SendString("$ACK,SET_OK\r\n");
    }
}

void Telemetry_Process_Rx_Byte(uint8_t byte) {
    if (byte == '\r' || byte == '\n') {
        if (s_cmd_idx > 0) {
            s_cmd_buf[s_cmd_idx] = '\0';
            Handle_Command_Line(s_cmd_buf);
            s_cmd_idx = 0;
        }
        return;
    }

    if (s_cmd_idx < (uint8_t)(sizeof(s_cmd_buf) - 1U)) {
        s_cmd_buf[s_cmd_idx++] = (char)byte;
    } else {
        s_cmd_idx = 0;
    }
}
