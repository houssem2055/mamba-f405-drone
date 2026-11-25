#include "attitude_mahony.h"
#include <math.h>

static inline float Clamp_Unit(float x) {
    if (x > 1.0f) return 1.0f;
    if (x < -1.0f) return -1.0f;
    return x;
}

void Mahony_Init(Mahony_AHRS_t *ahrs, float kp, float ki) {
    if (!ahrs) return;
    ahrs->q0 = 1.0f;
    ahrs->q1 = 0.0f;
    ahrs->q2 = 0.0f;
    ahrs->q3 = 0.0f;
    ahrs->ix = 0.0f;
    ahrs->iy = 0.0f;
    ahrs->iz = 0.0f;
    ahrs->kp = kp;
    ahrs->ki = ki;
    ahrs->initialized = false;
}

void Mahony_Reset_From_Accel(Mahony_AHRS_t *ahrs, float ax, float ay, float az) {
    if (!ahrs) return;
    float norm_acc = sqrtf(ax * ax + ay * ay + az * az);
    if (norm_acc < 3.0f || norm_acc > 18.0f) {
        ahrs->q0 = 1.0f; ahrs->q1 = 0.0f; ahrs->q2 = 0.0f; ahrs->q3 = 0.0f;
    } else {
        float roll0  = atan2f(ay, az);
        float pitch0 = atan2f(-ax, sqrtf(ay * ay + az * az));
        float cr = cosf(roll0 * 0.5f);
        float sr = sinf(roll0 * 0.5f);
        float cp = cosf(pitch0 * 0.5f);
        float sp = sinf(pitch0 * 0.5f);
        /* yaw0 = 0 -> cy = 1, sy = 0 */
        ahrs->q0 = cr * cp;
        ahrs->q1 = sr * cp;
        ahrs->q2 = cr * sp;
        ahrs->q3 = -sr * sp;
    }
    ahrs->ix = 0.0f;
    ahrs->iy = 0.0f;
    ahrs->iz = 0.0f;
    ahrs->initialized = true;
}

void Mahony_Update(
    Mahony_AHRS_t *ahrs,
    float gx, float gy, float gz,
    float ax, float ay, float az,
    float dt,
    float *out_roll,
    float *out_pitch,
    float *out_yaw
) {
    if (!ahrs || dt <= 0.0f) return;

    if (!ahrs->initialized) {
        Mahony_Reset_From_Accel(ahrs, ax, ay, az);
    }

    float q0 = ahrs->q0;
    float q1 = ahrs->q1;
    float q2 = ahrs->q2;
    float q3 = ahrs->q3;

    /* 1. Compute accelerometer correction only when total accel is near 1g */
    float norm_acc = sqrtf(ax * ax + ay * ay + az * az);
    if (norm_acc > 3.0f && norm_acc < 18.0f) {
        float inv_norm = 1.0f / norm_acc;
        float ax_n = ax * inv_norm;
        float ay_n = ay * inv_norm;
        float az_n = az * inv_norm;

        /* Estimated gravity direction in body frame from quaternion */
        float vx = 2.0f * (q1 * q3 - q0 * q2);
        float vy = 2.0f * (q0 * q1 + q2 * q3);
        float vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

        /* Cross product error: accel_norm x v_est */
        float ex = ay_n * vz - az_n * vy;
        float ey = az_n * vx - ax_n * vz;
        float ez = ax_n * vy - ay_n * vx;

        /* Scale gain down smoothly if far from 1g (vibration/acceleration rejection) */
        float g_err = fabsf(norm_acc - 9.81f) / 9.81f;
        float weight = 1.0f - (g_err * 1.5f);
        if (weight < 0.0f) weight = 0.0f;

        if (ahrs->ki > 0.0f) {
            ahrs->ix += ahrs->ki * weight * ex * dt;
            ahrs->iy += ahrs->ki * weight * ey * dt;
            ahrs->iz += ahrs->ki * weight * ez * dt;
        }

        gx += ahrs->kp * weight * ex + ahrs->ix;
        gy += ahrs->kp * weight * ey + ahrs->iy;
        gz += ahrs->kp * weight * ez + ahrs->iz;
    }

    /* 2. Integrate quaternion kinematics: q_dot = 0.5 * q * [0, gx, gy, gz] */
    float half_dt = 0.5f * dt;
    float q0_new = q0 + half_dt * (-q1 * gx - q2 * gy - q3 * gz);
    float q1_new = q1 + half_dt * ( q0 * gx + q2 * gz - q3 * gy);
    float q2_new = q2 + half_dt * ( q0 * gy - q1 * gz + q3 * gx);
    float q3_new = q3 + half_dt * ( q0 * gz + q1 * gy - q2 * gx);

    /* 3. Normalize quaternion */
    float q_norm = sqrtf(q0_new * q0_new + q1_new * q1_new + q2_new * q2_new + q3_new * q3_new);
    if (q_norm > 1e-8f) {
        float inv_q = 1.0f / q_norm;
        q0 = q0_new * inv_q;
        q1 = q1_new * inv_q;
        q2 = q2_new * inv_q;
        q3 = q3_new * inv_q;
    }

    ahrs->q0 = q0;
    ahrs->q1 = q1;
    ahrs->q2 = q2;
    ahrs->q3 = q3;

    /* 4. Extract Z-Y-X Euler angles (singularity-free internal state) */
    if (out_roll) {
        *out_roll = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2));
    }
    if (out_pitch) {
        *out_pitch = asinf(Clamp_Unit(2.0f * (q0 * q2 - q3 * q1)));
    }
    if (out_yaw) {
        *out_yaw = atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3));
    }
}
