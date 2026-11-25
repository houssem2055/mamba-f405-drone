#include "telemetry.h"
#include "config.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

extern void UART6_SendString(const char *str);

static char    s_cmd_buf[96];
static uint8_t s_cmd_idx = 0;

void Telemetry_Init(void) {
    s_cmd_idx = 0;
    memset(s_cmd_buf, 0, sizeof(s_cmd_buf));
}

void Telemetry_Send_Frame(
    const Attitude_t *att,
    const Motor_Outputs_t *motors,
    float vbat,
    bool armed,
    uint8_t mode
) {
    if (!att || !motors) return;

    char buffer[160];
    /* Format: $TLM,roll,pitch,yaw,gx,gy,gz,m1,m2,m3,m4,vbat,armed,mode\r\n */
    int len = snprintf(buffer, sizeof(buffer),
        "$TLM,%.2f,%.2f,%.2f,%.3f,%.3f,%.3f,%.0f,%.0f,%.0f,%.0f,%.2f,%u,%u\r\n",
        att->roll_deg, att->pitch_deg, att->yaw_deg,
        att->gx_filt, att->gy_filt, att->gz_filt,
        motors->m1, motors->m2, motors->m3, motors->m4,
        vbat, armed ? 1U : 0U, (unsigned int)mode
    );

    if (len > 0) {
        UART6_SendString(buffer);
    }
}

void Telemetry_Send_AI_Frame(const Attitude_t *att, const AI_Diagnostics_t *ai) {
    if (!att || !ai) return;

    char buffer[160];
    /* Format: $AI,health,fault_cls,du_r,du_p,gyro_jitter,motor_asym,ai_on\r\n */
    int len = snprintf(buffer, sizeof(buffer),
        "$AI,%.3f,%u,%.4f,%.4f,%.3f,%.1f,%u\r\n",
        ai->health_score,
        (unsigned int)ai->fault_class,
        att->ai_du[0],
        att->ai_du[1],
        ai->feat_gyro_jitter,
        ai->feat_motor_asym,
        Flight_Controller_Get_AI_Enabled() ? 1U : 0U
    );

    if (len > 0) {
        UART6_SendString(buffer);
    }
}

void Telemetry_Send_PID_Gains(void) {
    PID_Gains_t g;
    Flight_Controller_Get_Gains(&g);

    char buffer[160];
    int len = snprintf(buffer, sizeof(buffer),
        "$PID,%.3f,%.4f,%.4f,%.5f,%.4f,%.4f,%.4f,%.5f,%.4f,%.4f,%.4f,%.5f,%.4f\r\n",
        g.Kp_angle,
        g.Kp_p, g.Ki_p, g.Kd_p, g.Kf_p,
        g.Kp_q, g.Ki_q, g.Kd_q, g.Kf_q,
        g.Kp_r, g.Ki_r, g.Kd_r, g.Kf_r
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
