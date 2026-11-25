#include "flight_controller.h"
#include "config.h"
#include <math.h>

#define PI_F 3.141592653589793f

/* Persistent controller states */
static float s_est_roll = 0.0f;
static float s_est_pitch = 0.0f;
static float s_est_yaw = 0.0f;

static float s_i_roll = 0.0f;
static float s_i_pitch = 0.0f;
static float s_i_yaw = 0.0f;

static float s_prev_err_p = 0.0f;
static float s_prev_err_q = 0.0f;
static float s_prev_err_r = 0.0f;

static float s_d_roll_f = 0.0f;
static float s_d_pitch_f = 0.0f;
static float s_d_yaw_f = 0.0f;

/* Default PID gains (tuned in MATLAB) */
static PID_Gains_t s_gains = {
    .Kp_angle = 6.0f,
    .Kp_p = 0.13f, .Ki_p = 0.05f, .Kd_p = 0.0012f,
    .Kp_q = 0.13f, .Ki_q = 0.05f, .Kd_q = 0.0012f,
    .Kp_r = 0.50f, .Ki_r = 0.20f, .Kd_r = 0.0000f
};

static inline float Clamp(float val, float min, float max) {
    if (val < min) return min;
    if (val > max) return max;
    return val;
}

void Flight_Controller_Init(void) {
    s_est_roll = 0.0f;
    s_est_pitch = 0.0f;
    s_est_yaw = 0.0f;
    s_i_roll = 0.0f;
    s_i_pitch = 0.0f;
    s_i_yaw = 0.0f;
    s_prev_err_p = 0.0f;
    s_prev_err_q = 0.0f;
    s_prev_err_r = 0.0f;
    s_d_roll_f = 0.0f;
    s_d_pitch_f = 0.0f;
    s_d_yaw_f = 0.0f;
}

void Flight_Controller_Update(
    const RC_Channels_t *rc,
    const MPU6000_Data_t *imu,
    bool is_armed,
    float dt,
    Motor_Outputs_t *motors_out,
    Attitude_t *attitude_out
) {
    if (!rc || !imu || !motors_out || !attitude_out) return;

    /* 1. Attitude Estimation (Complementary Filter) */
    float norm_acc = sqrtf(imu->ax * imu->ax + imu->ay * imu->ay + imu->az * imu->az);
    float acc_roll = s_est_roll;
    float acc_pitch = s_est_pitch;

    if (norm_acc > 3.0f && norm_acc < 18.0f) {
        acc_roll  = atan2f(imu->ay, imu->az);
        acc_pitch = atan2f(-imu->ax, sqrtf(imu->ay * imu->ay + imu->az * imu->az));
    }

    float sr = sinf(s_est_roll);
    float cr = cosf(s_est_roll);
    float tp = tanf(s_est_pitch);
    float cp = cosf(s_est_pitch);
    if (fabsf(cp) < 0.05f) cp = 0.05f;

    float roll_dot  = imu->gx + (imu->gy * sr + imu->gz * cr) * tp;
    float pitch_dot = imu->gy * cr - imu->gz * sr;
    float yaw_dot   = (imu->gy * sr + imu->gz * cr) / cp;

    const float alpha = 0.995f;
    s_est_roll  = alpha * (s_est_roll  + roll_dot  * dt) + (1.0f - alpha) * acc_roll;
    s_est_pitch = alpha * (s_est_pitch + pitch_dot * dt) + (1.0f - alpha) * acc_pitch;
    s_est_yaw   = s_est_yaw + yaw_dot * dt;

    attitude_out->roll  = s_est_roll;
    attitude_out->pitch = s_est_pitch;
    attitude_out->yaw   = s_est_yaw;
    attitude_out->roll_deg  = s_est_roll  * (180.0f / PI_F);
    attitude_out->pitch_deg = s_est_pitch * (180.0f / PI_F);
    attitude_out->yaw_deg   = s_est_yaw   * (180.0f / PI_F);

    /* 2. Safety / Disarmed Handling */
    if (!is_armed || rc->throttle < MIN_THROTTLE_FOR_ARM) {
        s_i_roll = 0.0f;
        s_i_pitch = 0.0f;
        s_i_yaw = 0.0f;
        s_prev_err_p = 0.0f;
        s_prev_err_q = 0.0f;
        s_prev_err_r = 0.0f;
        s_d_roll_f = 0.0f;
        s_d_pitch_f = 0.0f;
        s_d_yaw_f = 0.0f;

        motors_out->m1 = MOTOR_MIN_PWM_US;
        motors_out->m2 = MOTOR_MIN_PWM_US;
        motors_out->m3 = MOTOR_MIN_PWM_US;
        motors_out->m4 = MOTOR_MIN_PWM_US;
        return;
    }

    /* 3. Outer Loop: Desired Rates Calculation */
    float p_des, q_des, r_des;
    if (rc->flight_mode == 1) { /* Angle Mode */
        const float max_angle = 0.785f; /* 45 degrees max tilt */
        float target_roll  = Clamp(rc->roll_cmd  * max_angle, -max_angle, max_angle);
        float target_pitch = Clamp(rc->pitch_cmd * max_angle, -max_angle, max_angle);

        p_des = s_gains.Kp_angle * (target_roll  - s_est_roll);
        q_des = s_gains.Kp_angle * (target_pitch - s_est_pitch);
        r_des = rc->yaw_cmd * 3.5f; /* Max 3.5 rad/s yaw rate */
    } else { /* Rate / Acro Mode */
        p_des = rc->roll_cmd  * 4.5f;
        q_des = rc->pitch_cmd * 4.5f;
        r_des = rc->yaw_cmd   * 3.5f;
    }

    /* 4. Inner Loop: Angular Rate PID */
    float err_p = p_des - imu->gx;
    float err_q = q_des - imu->gy;
    float err_r = r_des - imu->gz;

    /* Integrators with Anti-Windup */
    const float pwm_scale = 800.0f;
    const float max_i = 120.0f / pwm_scale; /* Max integral contribution = 120 us */
    s_i_roll  = Clamp(s_i_roll  + s_gains.Ki_p * err_p * dt, -max_i, max_i);
    s_i_pitch = Clamp(s_i_pitch + s_gains.Ki_q * err_q * dt, -max_i, max_i);
    s_i_yaw   = Clamp(s_i_yaw   + s_gains.Ki_r * err_r * dt, -max_i, max_i);

    /* Derivatives with 1st-order Low-Pass Filter (~28 Hz at 1 kHz) */
    const float d_lpf_alpha = 0.15f;
    float d_roll_raw  = s_gains.Kd_p * (err_p - s_prev_err_p) / dt;
    float d_pitch_raw = s_gains.Kd_q * (err_q - s_prev_err_q) / dt;
    float d_yaw_raw   = s_gains.Kd_r * (err_r - s_prev_err_r) / dt;

    s_d_roll_f  += d_lpf_alpha * (d_roll_raw  - s_d_roll_f);
    s_d_pitch_f += d_lpf_alpha * (d_pitch_raw - s_d_pitch_f);
    s_d_yaw_f   += d_lpf_alpha * (d_yaw_raw   - s_d_yaw_f);

    s_prev_err_p = err_p;
    s_prev_err_q = err_q;
    s_prev_err_r = err_r;

    /* Total control torque effort (scaled to PWM delta) */
    float u_roll  = ((s_gains.Kp_p * err_p) + s_i_roll  + s_d_roll_f)  * pwm_scale;
    float u_pitch = ((s_gains.Kp_q * err_q) + s_i_pitch + s_d_pitch_f) * pwm_scale;
    float u_yaw   = ((s_gains.Kp_r * err_r) + s_i_yaw   + s_d_yaw_f)   * pwm_scale;

    /* 5. Quad-X Motor Mixing */
    float base_thrust = MOTOR_IDLE_PWM_US + rc->throttle * (MOTOR_MAX_PWM_US - MOTOR_IDLE_PWM_US);

    /* M1: Rear-Right (CCW) */
    motors_out->m1 = Clamp(base_thrust - u_roll - u_pitch - u_yaw, MOTOR_IDLE_PWM_US, MOTOR_MAX_PWM_US);
    /* M2: Front-Right (CW) */
    motors_out->m2 = Clamp(base_thrust - u_roll + u_pitch + u_yaw, MOTOR_IDLE_PWM_US, MOTOR_MAX_PWM_US);
    /* M3: Rear-Left (CW) */
    motors_out->m3 = Clamp(base_thrust + u_roll - u_pitch + u_yaw, MOTOR_IDLE_PWM_US, MOTOR_MAX_PWM_US);
    /* M4: Front-Left (CCW) */
    motors_out->m4 = Clamp(base_thrust + u_roll + u_pitch - u_yaw, MOTOR_IDLE_PWM_US, MOTOR_MAX_PWM_US);
}
