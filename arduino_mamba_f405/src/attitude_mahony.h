#ifndef ATTITUDE_MAHONY_H
#define ATTITUDE_MAHONY_H

#include <stdbool.h>

typedef struct {
    float q0, q1, q2, q3;         /* Unit quaternion (Body -> Earth NED) */
    float ix, iy, iz;             /* Integral gyro bias correction terms */
    float kp;                     /* Proportional accel correction gain */
    float ki;                     /* Integral gyro bias learning gain */
    bool  initialized;
} Mahony_AHRS_t;

void Mahony_Init(Mahony_AHRS_t *ahrs, float kp, float ki);
void Mahony_Reset_From_Accel(Mahony_AHRS_t *ahrs, float ax, float ay, float az);
void Mahony_Update(
    Mahony_AHRS_t *ahrs,
    float gx, float gy, float gz,
    float ax, float ay, float az,
    float dt,
    float *out_roll,
    float *out_pitch,
    float *out_yaw
);

#endif /* ATTITUDE_MAHONY_H */
