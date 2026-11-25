function [motors_pwm, telemetry] = flight_controller(rc_cmd, accel, gyro, armed, flight_mode, dt)
%#codegen
% FLIGHT_CONTROLLER
% Core deterministic flight control algorithm for Mamba F405 MK2
%
% Inputs:
%   rc_cmd:      [throttle (0.0 to 1.0); roll_cmd; pitch_cmd; yaw_rate_cmd (rad/s)]
%   accel:       [ax; ay; az] in m/s^2 from MPU6000
%   gyro:        [gx; gy; gz] in rad/s from MPU6000
%   armed:       uint8 (0 = disarmed, 1 = armed)
%   flight_mode: uint8 (0 = Rate/Acro Mode, 1 = Angle/Level Mode)
%   dt:          Sample time (seconds, typically 0.001)
%
% Outputs:
%   motors_pwm:  [m1; m2; m3; m4] in microseconds (1000 to 2000 us)
%   telemetry:   [roll; pitch; yaw; p_err; q_err; r_err] for logging/ground station

    %% 1. Persistent Controller States (Integrals & Derivatives)
    persistent i_roll i_pitch i_yaw prev_rate_err_p prev_rate_err_q prev_rate_err_r d_roll_f d_pitch_f d_yaw_f;
    if isempty(i_roll)
        i_roll = 0.0;
        i_pitch = 0.0;
        i_yaw = 0.0;
        prev_rate_err_p = 0.0;
        prev_rate_err_q = 0.0;
        prev_rate_err_r = 0.0;
        d_roll_f = 0.0;
        d_pitch_f = 0.0;
        d_yaw_f = 0.0;
    end

    %% 2. Controller Tuning Parameters
    % Angle (Outer) Loop Proportional Gain
    Kp_angle = 6.0;

    % Rate (Inner) Loop PID Gains
    % Roll
    Kp_p = 0.13;
    Ki_p = 0.05;
    Kd_p = 0.0012;

    % Pitch
    Kp_q = 0.13;
    Ki_q = 0.05;
    Kd_q = 0.0012;

    % Yaw
    Kp_r = 0.50;
    Ki_r = 0.20;
    Kd_r = 0.0000;

    % Safety & Actuator Limits
    PWM_SCALE  = 800.0;
    MAX_I_PWM  = 120.0;                  % Max integral contribution in PWM us
    MAX_I_TERM = MAX_I_PWM / PWM_SCALE;  % Pre-scale anti-windup limit
    D_LPF_ALPHA = 0.15;                  % 1st-order D-term low-pass filter (~28 Hz at 1 kHz)
    MIN_PWM    = 1000.0;                 % ESC idle/off
    IDLE_PWM   = 1060.0;                 % ESC active idle
    MAX_PWM    = 2000.0;                 % Full throttle

    %% 3. Estimate Current Attitude
    reset_filter = (armed == 0);
    [est_roll, est_pitch, est_yaw] = attitude_estimator(accel, gyro, dt, reset_filter);

    throttle_in  = rc_cmd(1);
    roll_cmd_in  = rc_cmd(2);
    pitch_cmd_in = rc_cmd(3);
    yaw_cmd_in   = rc_cmd(4);

    % Reset I-terms when disarmed or throttle is at zero
    if armed == 0 || throttle_in < 0.05
        i_roll  = 0.0;
        i_pitch = 0.0;
        i_yaw   = 0.0;
        prev_rate_err_p = 0.0;
        prev_rate_err_q = 0.0;
        prev_rate_err_r = 0.0;
        d_roll_f = 0.0;
        d_pitch_f = 0.0;
        d_yaw_f = 0.0;
    end

    %% 4. Outer Loop: Attitude Angle -> Desired Angular Rates
    if flight_mode == 1 % Angle / Self-Leveling Mode
        % Limit max tilt angle to +/- 45 degrees (0.785 rad)
        max_tilt = 0.785;
        target_roll  = max(min(roll_cmd_in, max_tilt), -max_tilt);
        target_pitch = max(min(pitch_cmd_in, max_tilt), -max_tilt);

        % Proportional angle error to rate setpoints
        p_des = Kp_angle * (target_roll - est_roll);
        q_des = Kp_angle * (target_pitch - est_pitch);
        r_des = yaw_cmd_in; % Yaw is always rate controlled
    else % Rate / Acro Mode
        p_des = roll_cmd_in;
        q_des = pitch_cmd_in;
        r_des = yaw_cmd_in;
    end

    % Cap maximum angular rate setpoints (~250 deg/s = 4.36 rad/s)
    max_rate = 4.5;
    p_des = max(min(p_des, max_rate), -max_rate);
    q_des = max(min(q_des, max_rate), -max_rate);

    %% 5. Inner Loop: Angular Rate PID
    p_meas = gyro(1);
    q_meas = gyro(2);
    r_meas = gyro(3);

    % Rate Errors
    err_p = p_des - p_meas;
    err_q = q_des - q_meas;
    err_r = r_des - r_meas;

    % Integration with Anti-Windup
    if armed == 1 && throttle_in >= 0.05
        i_roll  = max(min(i_roll  + Ki_p * err_p * dt, MAX_I_TERM), -MAX_I_TERM);
        i_pitch = max(min(i_pitch + Ki_q * err_q * dt, MAX_I_TERM), -MAX_I_TERM);
        i_yaw   = max(min(i_yaw   + Ki_r * err_r * dt, MAX_I_TERM), -MAX_I_TERM);
    end

    % Derivative Term with 1st-order Low-Pass Filter (suppresses high-freq gyro noise)
    d_roll_raw  = Kd_p * (err_p - prev_rate_err_p) / dt;
    d_pitch_raw = Kd_q * (err_q - prev_rate_err_q) / dt;
    d_yaw_raw   = Kd_r * (err_r - prev_rate_err_r) / dt;

    d_roll_f  = d_roll_f  + D_LPF_ALPHA * (d_roll_raw  - d_roll_f);
    d_pitch_f = d_pitch_f + D_LPF_ALPHA * (d_pitch_raw - d_pitch_f);
    d_yaw_f   = d_yaw_f   + D_LPF_ALPHA * (d_yaw_raw   - d_yaw_f);

    prev_rate_err_p = err_p;
    prev_rate_err_q = err_q;
    prev_rate_err_r = err_r;

    % Total Control Torques (in PWM scale)
    u_roll  = (Kp_p * err_p) + i_roll  + d_roll_f;
    u_pitch = (Kp_q * err_q) + i_pitch + d_pitch_f;
    u_yaw   = (Kp_r * err_r) + i_yaw   + d_yaw_f;

    % Scale control efforts to motor PWM delta (scale factor = 800 us)
    u_roll  = u_roll  * PWM_SCALE;
    u_pitch = u_pitch * PWM_SCALE;
    u_yaw   = u_yaw   * PWM_SCALE;

    %% 6. Quad-X Motor Mixing
    % Base throttle maps 0.0 -> IDLE_PWM, 1.0 -> MAX_PWM
    base_thrust = IDLE_PWM + throttle_in * (MAX_PWM - IDLE_PWM);

    if armed == 0
        % Disarmed: Output zero / minimum pulse
        m1 = MIN_PWM;
        m2 = MIN_PWM;
        m3 = MIN_PWM;
        m4 = MIN_PWM;
    else
        % Quad-X Mixer (Betaflight standard numbering):
        % Motor 1: Rear-Right  (CCW) -> base - roll - pitch - yaw
        % Motor 2: Front-Right (CW)  -> base - roll + pitch + yaw
        % Motor 3: Rear-Left   (CW)  -> base + roll - pitch + yaw
        % Motor 4: Front-Left  (CCW) -> base + roll + pitch - yaw
        m1 = base_thrust - u_roll - u_pitch - u_yaw;
        m2 = base_thrust - u_roll + u_pitch + u_yaw;
        m3 = base_thrust + u_roll - u_pitch + u_yaw;
        m4 = base_thrust + u_roll + u_pitch - u_yaw;

        % Saturate to allowable PWM range
        m1 = max(min(m1, MAX_PWM), IDLE_PWM);
        m2 = max(min(m2, MAX_PWM), IDLE_PWM);
        m3 = max(min(m3, MAX_PWM), IDLE_PWM);
        m4 = max(min(m4, MAX_PWM), IDLE_PWM);
    end

    motors_pwm = [m1; m2; m3; m4];
    telemetry  = [est_roll; est_pitch; est_yaw; err_p; err_q; err_r];
end
