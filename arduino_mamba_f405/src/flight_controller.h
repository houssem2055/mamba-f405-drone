#ifndef FLIGHT_CONTROLLER_H
#define FLIGHT_CONTROLLER_H

#include <stdint.h>
#include <stdbool.h>
#include "mpu6000.h"
#include "rc_receiver.h"
#include "motors.h"

typedef enum {
    FLIGHT_MODE_ACRO    = 0,
    FLIGHT_MODE_ANGLE   = 1,
    FLIGHT_MODE_HORIZON = 2
} Flight_Mode_t;

typedef struct {
    float roll;      /* Estimated roll angle (rad) */
    float pitch;     /* Estimated pitch angle (rad) */
    float yaw;       /* Estimated yaw angle (rad) */
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
    float q[4];      /* Unit quaternion [q0, q1, q2, q3] */
    float gx_filt;   /* Filtered roll rate (rad/s) */
    float gy_filt;   /* Filtered pitch rate (rad/s) */
    float gz_filt;   /* Filtered yaw rate (rad/s) */
    float err_p;     /* Rate tracking error p (rad/s) */
    float err_q;     /* Rate tracking error q (rad/s) */
    float ai_du[3];  /* Residual compensation from TinyMLP [roll, pitch, yaw] */
} Attitude_t;

typedef struct {
    /* Outer loop: Angle gain */
    float Kp_angle;

    /* Inner loop: Roll Rate PID + Feedforward */
    float Kp_p, Ki_p, Kd_p, Kf_p;

    /* Inner loop: Pitch Rate PID + Feedforward */
    float Kp_q, Ki_q, Kd_q, Kf_q;

    /* Inner loop: Yaw Rate PID + Feedforward */
    float Kp_r, Ki_r, Kd_r, Kf_r;
} PID_Gains_t;

void Flight_Controller_Init(void);
void Flight_Controller_Set_Gains(const PID_Gains_t *new_gains);
void Flight_Controller_Get_Gains(PID_Gains_t *out_gains);
void Flight_Controller_Set_AI(bool enable, bool online_adapt);
bool Flight_Controller_Get_AI_Enabled(void);
void Flight_Controller_Reset_AI(void);
void Flight_Controller_Reset_Attitude(float ax, float ay, float az);

void Flight_Controller_Update(
    const RC_Channels_t *rc,
    const MPU6000_Data_t *imu,
    bool is_armed,
    bool airmode,
    float vbat_volts,
    float dt,
    Motor_Outputs_t *motors_out,
    Attitude_t *attitude_out
);

#endif /* FLIGHT_CONTROLLER_H */
