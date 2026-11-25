#include "flight_controller.h"
#include "attitude_mahony.h"
#include "biquad.h"
#include "config.h"
#include <math.h>
#include <string.h>

#define PI_F 3.14159265358979323846f

/* Persistent AHRS & Filter Instances */
static Mahony_AHRS_t   s_ahrs;
static Biquad_Filter_t s_gyro_lpf[3];
static Biquad_Filter_t s_gyro_notch[3];
static Biquad_Filter_t s_dterm_lpf[3];

/* Persistent PID Controller States */
static float s_i_roll  = 0.0f;
static float s_i_pitch = 0.0f;
static float s_i_yaw   = 0.0f;

static float s_prev_gyro_p = 0.0f;
static float s_prev_gyro_q = 0.0f;
static float s_prev_gyro_r = 0.0f;

static float s_prev_sp_p = 0.0f;
static float s_prev_sp_q = 0.0f;
static float s_prev_sp_r = 0.0f;

static float s_sp_dot_p = 0.0f;
static float s_sp_dot_q = 0.0f;
static float s_sp_dot_r = 0.0f;

/* Default PID gains (synchronized with MATLAB init_drone_parameters.m) */
static PID_Gains_t s_gains = {
    .Kp_angle = 6.0f,
    .Kp_p = 0.13f, .Ki_p = 0.05f, .Kd_p = 0.0014f, .Kf_p = 0.0020f,
    .Kp_q = 0.13f, .Ki_q = 0.05f, .Kd_q = 0.0014f, .Kf_q = 0.0020f,
    .Kp_r = 0.50f, .Ki_r = 0.20f, .Kd_r = 0.0000f, .Kf_r = 0.0020f
};

static inline float Clamp(float val, float min_v, float max_v) {
    if (val < min_v) return min_v;
    if (val > max_v) return max_v;
    return val;
}

void Flight_Controller_Init(void) {
    Mahony_Init(&s_ahrs, MAHONY_KP, MAHONY_KI);

    for (int i = 0; i < 3; i++) {
        Biquad_Init(&s_gyro_lpf[i],   BIQUAD_LPF,   GYRO_LPF_CUTOFF_HZ,   LOOP_FREQUENCY_HZ, BUTTERWORTH_Q);
        Biquad_Init(&s_gyro_notch[i], BIQUAD_NOTCH, GYRO_NOTCH_CENTER_HZ, LOOP_FREQUENCY_HZ, GYRO_NOTCH_Q);
        Biquad_Init(&s_dterm_lpf[i],  BIQUAD_LPF,   DTERM_LPF_CUTOFF_HZ,  LOOP_FREQUENCY_HZ, BUTTERWORTH_Q);
    }

    s_i_roll = 0.0f;  s_i_pitch = 0.0f; s_i_yaw = 0.0f;
    s_prev_gyro_p = 0.0f; s_prev_gyro_q = 0.0f; s_prev_gyro_r = 0.0f;
    s_prev_sp_p = 0.0f;   s_prev_sp_q = 0.0f;   s_prev_sp_r = 0.0f;
    s_sp_dot_p = 0.0f;    s_sp_dot_q = 0.0f;    s_sp_dot_r = 0.0f;
}

void Flight_Controller_Set_Gains(const PID_Gains_t *new_gains) {
    if (new_gains) {
        s_gains = *new_gains;
    }
}

void Flight_Controller_Get_Gains(PID_Gains_t *out_gains) {
    if (out_gains) {
        *out_gains = s_gains;
    }
}

void Flight_Controller_Reset_Attitude(float ax, float ay, float az) {
    Mahony_Reset_From_Accel(&s_ahrs, ax, ay, az);
}

void Flight_Controller_Update(
    const RC_Channels_t *rc,
    const MPU6000_Data_t *imu,
    bool is_armed,
    bool airmode,
    float vbat_volts,
    float dt,
    Motor_Outputs_t *motors_out,
    Attitude_t *attitude_out
) {
    if (!rc || !imu || !motors_out || !attitude_out || dt <= 0.0f) return;

    /* 1. 2-Stage Gyro Filtering (90 Hz Butterworth LPF + 220 Hz Notch) */
    float gx_lpf = Biquad_Apply(&s_gyro_lpf[0], imu->gx);
    float gy_lpf = Biquad_Apply(&s_gyro_lpf[1], imu->gy);
    float gz_lpf = Biquad_Apply(&s_gyro_lpf[2], imu->gz);

    float p_meas = Biquad_Apply(&s_gyro_notch[0], gx_lpf);
    float q_meas = Biquad_Apply(&s_gyro_notch[1], gy_lpf);
    float r_meas = Biquad_Apply(&s_gyro_notch[2], gz_lpf);

    /* 2. Singularity-Free Quaternion Mahony AHRS (always active for live bench tilt) */
    float est_roll = 0.0f, est_pitch = 0.0f, est_yaw = 0.0f;
    Mahony_Update(&s_ahrs, p_meas, q_meas, r_meas, imu->ax, imu->ay, imu->az, dt, &est_roll, &est_pitch, &est_yaw);

    attitude_out->roll      = est_roll;
    attitude_out->pitch     = est_pitch;
    attitude_out->yaw       = est_yaw;
    attitude_out->roll_deg  = est_roll  * (180.0f / PI_F);
    attitude_out->pitch_deg = est_pitch * (180.0f / PI_F);
    attitude_out->yaw_deg   = est_yaw   * (180.0f / PI_F);
    attitude_out->q[0] = s_ahrs.q0;
    attitude_out->q[1] = s_ahrs.q1;
    attitude_out->q[2] = s_ahrs.q2;
    attitude_out->q[3] = s_ahrs.q3;
    attitude_out->gx_filt = p_meas;
    attitude_out->gy_filt = q_meas;
    attitude_out->gz_filt = r_meas;

    float throttle_in = Clamp(rc->throttle, 0.0f, 1.0f);

    /* 3. Disarmed / Low-Throttle Integrator Reset */
    if (!is_armed || (throttle_in < MIN_THROTTLE_FOR_ARM && !airmode)) {
        s_i_roll = 0.0f; s_i_pitch = 0.0f; s_i_yaw = 0.0f;
        s_prev_gyro_p = p_meas; s_prev_gyro_q = q_meas; s_prev_gyro_r = r_meas;
        s_prev_sp_p = 0.0f; s_prev_sp_q = 0.0f; s_prev_sp_r = 0.0f;
        s_sp_dot_p = 0.0f;  s_sp_dot_q = 0.0f;  s_sp_dot_r = 0.0f;
        for (int i = 0; i < 3; i++) {
            Biquad_Reset(&s_dterm_lpf[i], 0.0f);
        }
    }

    if (!is_armed) {
        motors_out->m1 = MOTOR_MIN_PWM_US;
        motors_out->m2 = MOTOR_MIN_PWM_US;
        motors_out->m3 = MOTOR_MIN_PWM_US;
        motors_out->m4 = MOTOR_MIN_PWM_US;
        return;
    }

    /* 4. Outer Loop (Acro / Angle / Horizon) */
    const float max_angle   = 0.7854f; /* +/- 45 deg */
    const float max_rate_rp = 4.5f;    /* rad/s */
    const float max_rate_y  = 3.5f;    /* rad/s */

    float p_des, q_des, r_des;
    if (rc->flight_mode == FLIGHT_MODE_ANGLE) {
        float target_roll  = Clamp(rc->roll_cmd,  -max_angle, max_angle);
        float target_pitch = Clamp(rc->pitch_cmd, -max_angle, max_angle);
        p_des = s_gains.Kp_angle * (target_roll  - est_roll);
        q_des = s_gains.Kp_angle * (target_pitch - est_pitch);
        r_des = rc->yaw_cmd;
    } else if (rc->flight_mode == FLIGHT_MODE_HORIZON) {
        float abs_r = fabsf(rc->roll_cmd);
        float abs_p = fabsf(rc->pitch_cmd);
        float blend = Clamp(((abs_r > abs_p) ? abs_r : abs_p) / max_angle, 0.0f, 1.0f);

        float p_angle = s_gains.Kp_angle * (Clamp(rc->roll_cmd,  -max_angle, max_angle) - est_roll);
        float q_angle = s_gains.Kp_angle * (Clamp(rc->pitch_cmd, -max_angle, max_angle) - est_pitch);
        float p_acro  = (rc->roll_cmd  / max_angle) * max_rate_rp;
        float q_acro  = (rc->pitch_cmd / max_angle) * max_rate_rp;

        p_des = (1.0f - blend) * p_angle + blend * p_acro;
        q_des = (1.0f - blend) * q_angle + blend * q_acro;
        r_des = rc->yaw_cmd;
    } else {
        /* FLIGHT_MODE_ACRO */
        p_des = rc->roll_cmd;
        q_des = rc->pitch_cmd;
        r_des = rc->yaw_cmd;
    }

    p_des = Clamp(p_des, -max_rate_rp, max_rate_rp);
    q_des = Clamp(q_des, -max_rate_rp, max_rate_rp);
    r_des = Clamp(r_des, -max_rate_y,  max_rate_y);

    /* 5. Inner Loop: Angular Rate PID (Derivative-on-Measurement + D-LPF + Feedforward) */
    float err_p = p_des - p_meas;
    float err_q = q_des - q_meas;
    float err_r = r_des - r_meas;

    const float pwm_scale  = 800.0f;
    const float max_i_term = 120.0f / pwm_scale;

    if (throttle_in >= MIN_THROTTLE_FOR_ARM || airmode) {
        s_i_roll  = Clamp(s_i_roll  + s_gains.Ki_p * err_p * dt, -max_i_term, max_i_term);
        s_i_pitch = Clamp(s_i_pitch + s_gains.Ki_q * err_q * dt, -max_i_term, max_i_term);
        s_i_yaw   = Clamp(s_i_yaw   + s_gains.Ki_r * err_r * dt, -max_i_term, max_i_term);
    }

    float d_p_raw = -s_gains.Kd_p * (p_meas - s_prev_gyro_p) / dt;
    float d_q_raw = -s_gains.Kd_q * (q_meas - s_prev_gyro_q) / dt;
    float d_r_raw = -s_gains.Kd_r * (r_meas - s_prev_gyro_r) / dt;

    s_prev_gyro_p = p_meas;
    s_prev_gyro_q = q_meas;
    s_prev_gyro_r = r_meas;

    float d_p = Biquad_Apply(&s_dterm_lpf[0], d_p_raw);
    float d_q = Biquad_Apply(&s_dterm_lpf[1], d_q_raw);
    float d_r = Biquad_Apply(&s_dterm_lpf[2], d_r_raw);

    /* Filtered Setpoint Feedforward */
    const float sp_alpha = 0.05f;
    s_sp_dot_p += sp_alpha * ((p_des - s_prev_sp_p) / dt - s_sp_dot_p);
    s_sp_dot_q += sp_alpha * ((q_des - s_prev_sp_q) / dt - s_sp_dot_q);
    s_sp_dot_r += sp_alpha * ((r_des - s_prev_sp_r) / dt - s_sp_dot_r);

    s_prev_sp_p = p_des;
    s_prev_sp_q = q_des;
    s_prev_sp_r = r_des;

    float ff_p = s_gains.Kf_p * s_sp_dot_p;
    float ff_q = s_gains.Kf_q * s_sp_dot_q;
    float ff_r = s_gains.Kf_r * s_sp_dot_r;

    /* 6. TPA & Battery Voltage Sag Compensation */
    float tpa_scale = 1.0f;
    if (throttle_in > TPA_BREAKPOINT) {
        tpa_scale = 1.0f - TPA_RATE * ((throttle_in - TPA_BREAKPOINT) / (1.0f - TPA_BREAKPOINT));
    }

    float vbat_scale = 1.0f;
    if (vbat_volts >= 12.0f && vbat_volts <= 17.2f) {
        vbat_scale = Clamp(VBAT_NOMINAL_4S / vbat_volts, 0.90f, 1.25f);
    }

    float pid_scale = pwm_scale * tpa_scale * vbat_scale;

    float u_roll  = (s_gains.Kp_p * err_p + d_p + ff_p) * pid_scale + (s_i_roll  * pwm_scale * vbat_scale);
    float u_pitch = (s_gains.Kp_q * err_q + d_q + ff_q) * pid_scale + (s_i_pitch * pwm_scale * vbat_scale);
    float u_yaw   = (s_gains.Kp_r * err_r + d_r + ff_r) * pid_scale + (s_i_yaw   * pwm_scale * vbat_scale);

    /* 7. Quad-X Motor Mixing with Airmode Desaturation */
    float base_thrust = MOTOR_IDLE_PWM_US + throttle_in * (MOTOR_MAX_PWM_US - MOTOR_IDLE_PWM_US);

    float m1 = base_thrust - u_roll - u_pitch - u_yaw;
    float m2 = base_thrust - u_roll + u_pitch + u_yaw;
    float m3 = base_thrust + u_roll - u_pitch + u_yaw;
    float m4 = base_thrust + u_roll + u_pitch - u_yaw;

    if (airmode) {
        float m_min = fminf(fminf(m1, m2), fminf(m3, m4));
        float m_max = fmaxf(fmaxf(m1, m2), fmaxf(m3, m4));
        float range_avail = MOTOR_MAX_PWM_US - MOTOR_IDLE_PWM_US;
        float span = m_max - m_min;

        if (span > range_avail && span > 1e-3f) {
            float scale_mix = range_avail / span;
            u_roll  *= scale_mix;
            u_pitch *= scale_mix;
            u_yaw   *= scale_mix;
            m1 = base_thrust - u_roll - u_pitch - u_yaw;
            m2 = base_thrust - u_roll + u_pitch + u_yaw;
            m3 = base_thrust + u_roll - u_pitch + u_yaw;
            m4 = base_thrust + u_roll + u_pitch - u_yaw;
            m_min = fminf(fminf(m1, m2), fminf(m3, m4));
            m_max = fmaxf(fmaxf(m1, m2), fmaxf(m3, m4));
        }

        float shift = 0.0f;
        if (m_max > MOTOR_MAX_PWM_US) {
            shift = MOTOR_MAX_PWM_US - m_max;
        } else if (m_min < MOTOR_IDLE_PWM_US) {
            shift = MOTOR_IDLE_PWM_US - m_min;
        }
        m1 += shift;
        m2 += shift;
        m3 += shift;
        m4 += shift;
    }

    motors_out->m1 = Clamp(m1, MOTOR_IDLE_PWM_US, MOTOR_MAX_PWM_US);
    motors_out->m2 = Clamp(m2, MOTOR_IDLE_PWM_US, MOTOR_MAX_PWM_US);
    motors_out->m3 = Clamp(m3, MOTOR_IDLE_PWM_US, MOTOR_MAX_PWM_US);
    motors_out->m4 = Clamp(m4, MOTOR_IDLE_PWM_US, MOTOR_MAX_PWM_US);
}
