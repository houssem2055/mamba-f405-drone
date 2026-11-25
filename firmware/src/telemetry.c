#include "telemetry.h"
#include "config.h"
#include <stdio.h>
#include <string.h>

extern void UART6_SendString(const char *str);

void Telemetry_Init(void) {
    /* Telemetry UART initialized at 115200 baud */
}

void Telemetry_Send_Frame(const Attitude_t *att, const Motor_Outputs_t *motors, float vbat) {
    if (!att || !motors) return;

    char buffer[128];
    /* Format: $TLM,roll_deg,pitch_deg,yaw_deg,m1,m2,m3,m4,vbat\n */
    int len = snprintf(buffer, sizeof(buffer),
        "$TLM,%.2f,%.2f,%.2f,%.0f,%.0f,%.0f,%.0f,%.2f\r\n",
        att->roll_deg, att->pitch_deg, att->yaw_deg,
        motors->m1, motors->m2, motors->m3, motors->m4,
        vbat
    );

    if (len > 0) {
        UART6_SendString(buffer);
    }
}
