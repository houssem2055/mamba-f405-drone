function [motors_pwm, telemetry] = flight_controller(rc_cmd, accel, gyro, armed, flight_mode, dt, vbat, airmode)
%#codegen
% FLIGHT_CONTROLLER (Pro Aerospace Architecture)
% Deterministic 1 kHz flight control loop featuring:
%   1. 2nd-Order Butterworth Gyro LPF (90 Hz) + 2nd-Order Notch (220 Hz)
%   2. Singularity-Free Quaternion Mahony AHRS (always active for bench/tilt checks)
%   3. Acro (0), Angle (1), and Horizon (2) Flight Modes + RC Expo
%   4. Cascaded P-PID with Derivative-on-Measurement + 70 Hz D-Term LPF + Feedforward
%   5. Throttle PID Attenuation (TPA) & Battery Voltage Sag Compensation
%   6. Airmode Dynamic Mixer Desaturation (preserves control authority at extremes)
%
% Inputs:
%   rc_cmd:      [throttle (0..1); roll_in; pitch_in; yaw_in]
%                (In Angle/Horizon mode: roll_in/pitch_in in rad or [-1..1] normalized;
%                 if |val| <= 1, treated directly as angle rad when called from simulator or scaled)
%   accel:       [ax; ay; az] in m/s^2
%   gyro:        [gx; gy; gz] in rad/s
%   armed:       uint8 (0 = disarmed, 1 = armed)
%   flight_mode: uint8 (0 = Acro, 1 = Angle, 2 = Horizon)
%   dt:          Sample time (s), typically 0.001
%   vbat:        (Optional) Battery voltage in Volts (default 16.0V)
%   airmode:     (Optional) uint8 (1 = Airmode active, 0 = standard mixer; default 1)
%
% Outputs:
%   motors_pwm:  [m1; m2; m3; m4] in microseconds (1000..2000 us)
%   telemetry:   [roll; pitch; yaw; gx_filt; gy_filt; gz_filt]

    if nargin < 7
        vbat = 16.0;
    end
    if nargin < 8
        airmode = uint8(1);
    end

    %% 1. Persistent States
    persistent i_roll i_pitch i_yaw;
    persistent prev_gyro_p prev_gyro_q prev_gyro_r;
    persistent prev_sp_p prev_sp_q prev_sp_r;
    persistent sp_dot_p sp_dot_q sp_dot_r;
    persistent gyro_lpf_st gyro_notch_st d_lpf_st;
    persistent initialized;

    if isempty(initialized)
        i_roll = 0.0; i_pitch = 0.0; i_yaw = 0.0;
        prev_gyro_p = 0.0; prev_gyro_q = 0.0; prev_gyro_r = 0.0;
        prev_sp_p = 0.0; prev_sp_q = 0.0; prev_sp_r = 0.0;
        sp_dot_p = 0.0; sp_dot_q = 0.0; sp_dot_r = 0.0;
        gyro_lpf_st   = zeros(2, 3);
        gyro_notch_st = zeros(2, 3);
        d_lpf_st      = zeros(2, 3);
        initialized = true;
    end

    %% 2. Controller & Filter Constants (Synced with C Firmware)
    fs_hz = 1.0 / max(dt, 1e-5);

    % Biquad Filter Cutoffs
    GYRO_LPF_HZ   = 90.0;
    GYRO_NOTCH_HZ = 220.0;
    GYRO_NOTCH_Q  = 2.5;
    DTERM_LPF_HZ  = 70.0;
    BUTTER_Q      = 0.70710678;

    % Outer Loop (Angle)
    Kp_angle = 6.0;
    MAX_TILT_RAD = 0.7854; % 45 degrees

    % Inner Loop (Rate PID + Feedforward)
    Kp_p = 0.13; Ki_p = 0.05; Kd_p = 0.0014; Kf_p = 0.0020;
    Kp_q = 0.13; Ki_q = 0.05; Kd_q = 0.0014; Kf_q = 0.0020;
    Kp_r = 0.50; Ki_r = 0.20; Kd_r = 0.0000; Kf_r = 0.0020;

    % Limits & Compensation
    PWM_SCALE      = 800.0;
    MAX_I_PWM      = 120.0;
    MAX_I_TERM     = MAX_I_PWM / PWM_SCALE;
    MIN_PWM        = 1000.0;
    IDLE_PWM       = 1060.0;
    MAX_PWM        = 2000.0;
    MAX_RATE_RP    = 4.5;
    MAX_RATE_YAW   = 3.5;
    TPA_BREAKPOINT = 0.50;
    TPA_RATE       = 0.25;
    VBAT_NOM       = 16.0;

    %% 3. Gyro Biquad Filtering (90 Hz LPF + 220 Hz Notch)
    [gx_lpf, gyro_lpf_st(:, 1)]   = biquad_filter(gyro(1), gyro_lpf_st(:, 1), 0, GYRO_LPF_HZ, fs_hz, BUTTER_Q);
    [gy_lpf, gyro_lpf_st(:, 2)]   = biquad_filter(gyro(2), gyro_lpf_st(:, 2), 0, GYRO_LPF_HZ, fs_hz, BUTTER_Q);
    [gz_lpf, gyro_lpf_st(:, 3)]   = biquad_filter(gyro(3), gyro_lpf_st(:, 3), 0, GYRO_LPF_HZ, fs_hz, BUTTER_Q);

    [p_meas, gyro_notch_st(:, 1)] = biquad_filter(gx_lpf, gyro_notch_st(:, 1), 1, GYRO_NOTCH_HZ, fs_hz, GYRO_NOTCH_Q);
    [q_meas, gyro_notch_st(:, 2)] = biquad_filter(gy_lpf, gyro_notch_st(:, 2), 1, GYRO_NOTCH_HZ, fs_hz, GYRO_NOTCH_Q);
    [r_meas, gyro_notch_st(:, 3)] = biquad_filter(gz_lpf, gyro_notch_st(:, 3), 1, GYRO_NOTCH_HZ, fs_hz, GYRO_NOTCH_Q);

    %% 4. Quaternion Mahony Attitude Estimation (always running, no forced zero while disarmed)
    [est_roll, est_pitch, est_yaw, ~] = attitude_estimator(accel, [p_meas; q_meas; r_meas], dt, false);

    throttle_in  = max(min(rc_cmd(1), 1.0), 0.0);
    roll_cmd_in  = rc_cmd(2);
    pitch_cmd_in = rc_cmd(3);
    yaw_cmd_in   = rc_cmd(4);

    %% 5. Disarmed / Low-Throttle Integrator Reset
    if armed == 0 || (throttle_in < 0.05 && airmode == 0)
        i_roll = 0.0; i_pitch = 0.0; i_yaw = 0.0;
        prev_gyro_p = p_meas; prev_gyro_q = q_meas; prev_gyro_r = r_meas;
        prev_sp_p = 0.0; prev_sp_q = 0.0; prev_sp_r = 0.0;
        sp_dot_p = 0.0; sp_dot_q = 0.0; sp_dot_r = 0.0;
        d_lpf_st = zeros(2, 3);
    end

    if armed == 0
        motors_pwm = [MIN_PWM; MIN_PWM; MIN_PWM; MIN_PWM];
        telemetry  = [est_roll; est_pitch; est_yaw; p_meas; q_meas; r_meas];
        return;
    end

    %% 6. Outer Loop (Acro / Angle / Horizon)
    if flight_mode == 1 % Angle (Self-Leveling) Mode
        target_roll  = max(min(roll_cmd_in, MAX_TILT_RAD), -MAX_TILT_RAD);
        target_pitch = max(min(pitch_cmd_in, MAX_TILT_RAD), -MAX_TILT_RAD);
        p_des = Kp_angle * (target_roll  - est_roll);
        q_des = Kp_angle * (target_pitch - est_pitch);
        r_des = yaw_cmd_in;
    elseif flight_mode == 2 % Horizon Mode (blends Angle near center -> Acro at full stick)
        stick_deflection = max(abs(roll_cmd_in), abs(pitch_cmd_in)) / MAX_TILT_RAD;
        blend = max(min(stick_deflection, 1.0), 0.0);
        p_angle = Kp_angle * (max(min(roll_cmd_in, MAX_TILT_RAD), -MAX_TILT_RAD) - est_roll);
        q_angle = Kp_angle * (max(min(pitch_cmd_in, MAX_TILT_RAD), -MAX_TILT_RAD) - est_pitch);
        p_acro  = (roll_cmd_in  / MAX_TILT_RAD) * MAX_RATE_RP;
        q_acro  = (pitch_cmd_in / MAX_TILT_RAD) * MAX_RATE_RP;
        p_des   = (1.0 - blend) * p_angle + blend * p_acro;
        q_des   = (1.0 - blend) * q_angle + blend * q_acro;
        r_des   = yaw_cmd_in;
    else % Mode 0: Pure Acro (Rate) Mode (supports 360-deg flips)
        p_des = roll_cmd_in;
        q_des = pitch_cmd_in;
        r_des = yaw_cmd_in;
    end

    p_des = max(min(p_des, MAX_RATE_RP), -MAX_RATE_RP);
    q_des = max(min(q_des, MAX_RATE_RP), -MAX_RATE_RP);
    r_des = max(min(r_des, MAX_RATE_YAW), -MAX_RATE_YAW);

    %% 7. Inner Loop: Rate PID (Derivative-on-Measurement + 70 Hz Biquad D-LPF + Feedforward)
    err_p = p_des - p_meas;
    err_q = q_des - q_meas;
    err_r = r_des - r_meas;

    % Anti-windup integration (active when throttle >= 0.05 or airmode)
    if throttle_in >= 0.05 || airmode == 1
        i_roll  = max(min(i_roll  + Ki_p * err_p * dt, MAX_I_TERM), -MAX_I_TERM);
        i_pitch = max(min(i_pitch + Ki_q * err_q * dt, MAX_I_TERM), -MAX_I_TERM);
        i_yaw   = max(min(i_yaw   + Ki_r * err_r * dt, MAX_I_TERM), -MAX_I_TERM);
    end

    % Derivative-on-Measurement (-d(gyro)/dt eliminates derivative kick on step commands)
    d_p_raw = -Kd_p * (p_meas - prev_gyro_p) / dt;
    d_q_raw = -Kd_q * (q_meas - prev_gyro_q) / dt;
    d_r_raw = -Kd_r * (r_meas - prev_gyro_r) / dt;

    prev_gyro_p = p_meas;
    prev_gyro_q = q_meas;
    prev_gyro_r = r_meas;

    [d_p, d_lpf_st(:, 1)] = biquad_filter(d_p_raw, d_lpf_st(:, 1), 0, DTERM_LPF_HZ, fs_hz, BUTTER_Q);
    [d_q, d_lpf_st(:, 2)] = biquad_filter(d_q_raw, d_lpf_st(:, 2), 0, DTERM_LPF_HZ, fs_hz, BUTTER_Q);
    [d_r, d_lpf_st(:, 3)] = biquad_filter(d_r_raw, d_lpf_st(:, 3), 0, DTERM_LPF_HZ, fs_hz, BUTTER_Q);

    % Smooth Setpoint Feedforward (filtered derivative of setpoint)
    sp_alpha = 0.05;
    sp_dot_p = sp_dot_p + sp_alpha * ((p_des - prev_sp_p) / dt - sp_dot_p);
    sp_dot_q = sp_dot_q + sp_alpha * ((q_des - prev_sp_q) / dt - sp_dot_q);
    sp_dot_r = sp_dot_r + sp_alpha * ((r_des - prev_sp_r) / dt - sp_dot_r);

    prev_sp_p = p_des;
    prev_sp_q = q_des;
    prev_sp_r = r_des;

    ff_p = Kf_p * sp_dot_p;
    ff_q = Kf_q * sp_dot_q;
    ff_r = Kf_r * sp_dot_r;

    %% 8. TPA (Throttle PID Attenuation) & Battery Voltage Sag Compensation
    tpa_scale = 1.0;
    if throttle_in > TPA_BREAKPOINT
        tpa_scale = 1.0 - TPA_RATE * ((throttle_in - TPA_BREAKPOINT) / (1.0 - TPA_BREAKPOINT));
    end

    vbat_scale = 1.0;
    if vbat >= 12.0 && vbat <= 17.2
        vbat_scale = max(min(VBAT_NOM / vbat, 1.25), 0.90);
    end

    pid_scale = PWM_SCALE * tpa_scale * vbat_scale;

    u_roll  = (Kp_p * err_p + d_p + ff_p) * pid_scale + (i_roll  * PWM_SCALE * vbat_scale);
    u_pitch = (Kp_q * err_q + d_q + ff_q) * pid_scale + (i_pitch * PWM_SCALE * vbat_scale);
    u_yaw   = (Kp_r * err_r + d_r + ff_r) * pid_scale + (i_yaw   * PWM_SCALE * vbat_scale);

    %% 9. Quad-X Motor Mixing with Airmode Desaturation
    base_thrust = IDLE_PWM + throttle_in * (MAX_PWM - IDLE_PWM);

    m1 = base_thrust - u_roll - u_pitch - u_yaw;
    m2 = base_thrust - u_roll + u_pitch + u_yaw;
    m3 = base_thrust + u_roll - u_pitch + u_yaw;
    m4 = base_thrust + u_roll + u_pitch - u_yaw;

    if airmode == 1
        m_min = min([m1, m2, m3, m4]);
        m_max = max([m1, m2, m3, m4]);
        range_avail = MAX_PWM - IDLE_PWM;
        span = m_max - m_min;

        if span > range_avail && span > 1e-3
            % Scale differential authority proportionally if total span exceeds ESC range
            scale_mix = range_avail / span;
            u_roll  = u_roll  * scale_mix;
            u_pitch = u_pitch * scale_mix;
            u_yaw   = u_yaw   * scale_mix;
            m1 = base_thrust - u_roll - u_pitch - u_yaw;
            m2 = base_thrust - u_roll + u_pitch + u_yaw;
            m3 = base_thrust + u_roll - u_pitch + u_yaw;
            m4 = base_thrust + u_roll + u_pitch - u_yaw;
            m_min = min([m1, m2, m3, m4]);
            m_max = max([m1, m2, m3, m4]);
        end

        shift = 0.0;
        if m_max > MAX_PWM
            shift = MAX_PWM - m_max;
        elseif m_min < IDLE_PWM
            shift = IDLE_PWM - m_min;
        end
        m1 = m1 + shift;
        m2 = m2 + shift;
        m3 = m3 + shift;
        m4 = m4 + shift;
    end

    m1 = max(min(m1, MAX_PWM), IDLE_PWM);
    m2 = max(min(m2, MAX_PWM), IDLE_PWM);
    m3 = max(min(m3, MAX_PWM), IDLE_PWM);
    m4 = max(min(m4, MAX_PWM), IDLE_PWM);

    motors_pwm = [m1; m2; m3; m4];
    telemetry  = [est_roll; est_pitch; est_yaw; p_meas; q_meas; r_meas];
end
