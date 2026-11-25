function [roll, pitch, yaw, q_out] = attitude_estimator(accel, gyro, dt, reset)
%#codegen
% ATTITUDE_ESTIMATOR (Quaternion Mahony AHRS)
% Singularity-free quaternion sensor fusion combining accelerometer (m/s^2)
% and gyroscope (rad/s) with adaptive acceleration gating and online gyro
% bias compensation. Matches firmware/src/attitude_mahony.c.
%
% Inputs:
%   accel: [ax; ay; az] in m/s^2
%   gyro:  [gx; gy; gz] in rad/s (body angular rates: p, q, r)
%   dt:    Sample time in seconds (e.g., 0.001 for 1 kHz)
%   reset: Boolean flag to re-initialize quaternion from accelerometer
%
% Outputs:
%   roll:  Estimated roll angle (rad)
%   pitch: Estimated pitch angle (rad)
%   yaw:   Estimated yaw angle (rad)
%   q_out: Unit quaternion [q0; q1; q2; q3]

    persistent q integral_err initialized;
    if isempty(initialized) || reset
        ax = accel(1); ay = accel(2); az = accel(3);
        norm_acc = sqrt(ax*ax + ay*ay + az*az);
        if norm_acc >= 3.0 && norm_acc <= 18.0
            roll0  = atan2(ay, az);
            pitch0 = atan2(-ax, sqrt(ay*ay + az*az));
            cr = cos(roll0 * 0.5); sr = sin(roll0 * 0.5);
            cp = cos(pitch0 * 0.5); sp = sin(pitch0 * 0.5);
            q = [cr * cp; sr * cp; cr * sp; -sr * sp];
        else
            q = [1.0; 0.0; 0.0; 0.0];
        end
        integral_err = [0.0; 0.0; 0.0];
        initialized = true;
    end

    Kp_mahony = 2.5;   % Proportional accel trust gain
    Ki_mahony = 0.05;  % Integral gyro bias tracking gain

    ax = accel(1); ay = accel(2); az = accel(3);
    gx = gyro(1);  gy = gyro(2);  gz = gyro(3);

    q0 = q(1); q1 = q(2); q2 = q(3); q3 = q(4);

    % 1. Adaptive Accelerometer Correction (rejects freefall/high-g linear accel)
    norm_acc = sqrt(ax*ax + ay*ay + az*az);
    if norm_acc > 3.0 && norm_acc < 18.0
        inv_norm = 1.0 / norm_acc;
        ax_n = ax * inv_norm;
        ay_n = ay * inv_norm;
        az_n = az * inv_norm;

        % Gravity direction estimated from quaternion
        vx = 2.0 * (q1*q3 - q0*q2);
        vy = 2.0 * (q0*q1 + q2*q3);
        vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

        % Cross product error: accel_norm x v_est
        ex = ay_n * vz - az_n * vy;
        ey = az_n * vx - ax_n * vz;
        ez = ax_n * vy - ay_n * vx;

        % Smoothly attenuate trust if specific force deviates from 1g
        g_err = abs(norm_acc - 9.81) / 9.81;
        weight = max(0.0, 1.0 - 1.5 * g_err);

        integral_err(1) = integral_err(1) + Ki_mahony * weight * ex * dt;
        integral_err(2) = integral_err(2) + Ki_mahony * weight * ey * dt;
        integral_err(3) = integral_err(3) + Ki_mahony * weight * ez * dt;

        gx = gx + Kp_mahony * weight * ex + integral_err(1);
        gy = gy + Kp_mahony * weight * ey + integral_err(2);
        gz = gz + Kp_mahony * weight * ez + integral_err(3);
    end

    % 2. Quaternion Kinematics Integration
    half_dt = 0.5 * dt;
    q0_new = q0 + half_dt * (-q1*gx - q2*gy - q3*gz);
    q1_new = q1 + half_dt * ( q0*gx + q2*gz - q3*gy);
    q2_new = q2 + half_dt * ( q0*gy - q1*gz + q3*gx);
    q3_new = q3 + half_dt * ( q0*gz + q1*gy - q2*gx);

    q_norm = sqrt(q0_new*q0_new + q1_new*q1_new + q2_new*q2_new + q3_new*q3_new);
    if q_norm > 1e-8
        inv_q = 1.0 / q_norm;
        q = [q0_new; q1_new; q2_new; q3_new] * inv_q;
    end

    q0 = q(1); q1 = q(2); q2 = q(3); q3 = q(4);

    % 3. Extract Z-Y-X Euler Angles
    sin_pitch = max(min(2.0 * (q0*q2 - q3*q1), 1.0), -1.0);
    roll  = atan2(2.0 * (q0*q1 + q2*q3), 1.0 - 2.0 * (q1*q1 + q2*q2));
    pitch = asin(sin_pitch);
    yaw   = atan2(2.0 * (q0*q3 + q1*q2), 1.0 - 2.0 * (q2*q2 + q3*q3));
    q_out = q;
end
