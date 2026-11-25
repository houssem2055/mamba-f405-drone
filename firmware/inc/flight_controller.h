#ifndef FLIGHT_CONTROLLER_H
#define FLIGHT_CONTROLLER_H

#include <stdint.h>
#include <stdbool.h>
#include "mpu6000.h"
#include "rc_receiver.h"
#include "motors.h"

typedef struct {
    float roll;      /* Estimated roll angle (rad) */
    float pitch;     /* Estimated pitch angle (rad) */
    float yaw;       /* Estimated yaw angle (rad) */
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
} Attitude_t;

typedef struct {
    /* Outer loop: Angle gains */
    float Kp_angle;
    
    /* Inner loop: Roll Rate PID */
    float Kp_p, Ki_p, Kd_p;
    
    /* Inner loop: Pitch Rate PID */
    float Kp_q, Ki_q, Kd_q;
    
    /* Inner loop: Yaw Rate PID */
    float Kp_r, Ki_r, Kd_r;
} PID_Gains_t;

void Flight_Controller_Init(void);
void Flight_Controller_Update(
    const RC_Channels_t *rc,
    const MPU6000_Data_t *imu,
    bool is_armed,
    float dt,
    Motor_Outputs_t *motors_out,
    Attitude_t *attitude_out
);

#endif /* FLIGHT_CONTROLLER_H */
