#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdint.h>
#include <stdbool.h>
#include "flight_controller.h"
#include "motors.h"

void Telemetry_Init(void);
void Telemetry_Send_Frame(
    const Attitude_t *att,
    const Motor_Outputs_t *motors,
    float vbat,
    bool armed,
    uint8_t mode
);
void Telemetry_Send_PID_Gains(void);
void Telemetry_Process_Rx_Byte(uint8_t byte);

#endif /* TELEMETRY_H */
