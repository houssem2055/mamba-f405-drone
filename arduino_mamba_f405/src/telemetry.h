#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdint.h>
#include <stdbool.h>
#include "flight_controller.h"
#include "ai_fault_detector.h"
#include "motors.h"
#include "rth_failsafe.h"
#include "safety_monitor.h"

void Telemetry_Init(void);
void Telemetry_Send_Frame(
    const Attitude_t *att,
    const Motor_Outputs_t *motors,
    float vbat,
    bool armed,
    uint8_t mode
);
void Telemetry_Send_AI_Frame(const Attitude_t *att, const AI_Diagnostics_t *ai);
void Telemetry_Send_Nav_Frame(const RTH_Output_t *rth, const Safety_Status_t *safety, float est_alt_m, uint8_t gps_sats);
void Telemetry_Send_PID_Gains(void);
void Telemetry_Process_Rx_Byte(uint8_t byte);

#endif /* TELEMETRY_H */
