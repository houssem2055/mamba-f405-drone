#ifndef TELEMETRY_H
#define TELEMETRY_H

#include "flight_controller.h"
#include "motors.h"

void Telemetry_Init(void);
void Telemetry_Send_Frame(const Attitude_t *att, const Motor_Outputs_t *motors, float vbat);

#endif /* TELEMETRY_H */
