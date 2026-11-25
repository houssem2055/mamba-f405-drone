function [roll, pitch, yaw] = attitude_estimator(accel, gyro, dt, reset)
%#codegen
% ATTITUDE_ESTIMATOR
% Fuses Accelerometer (m/s^2) and Gyroscope (rad/s) data into Euler angles (rad).
% Designed for direct MATLAB Coder compilation into embedded C.
%
% Inputs:
%   accel: [ax; ay; az] in m/s^2
%   gyro:  [gx; gy; gz] in rad/s (body angular rates: p, q, r)
%   dt:    Sample time in seconds (e.g. 0.001 for 1kHz)
%   reset: Boolean flag to re-initialize attitude from accelerometer
%
% Outputs:
%   roll:  Estimated roll angle (rad)
%   pitch: Estimated pitch angle (rad)
%   yaw:   Estimated yaw angle (rad, integrated from gyro)

    % Persistent states across sample steps
    persistent est_roll est_pitch est_yaw initialized;
    if isempty(initialized) || reset
        est_roll  = 0.0;
        est_pitch = 0.0;
        est_yaw   = 0.0;
        initialized = true;
    end

    ax = accel(1);
    ay = accel(2);
    az = accel(3);

    gx = gyro(1);
    gy = gyro(2);
    gz = gyro(3);

    % 1. Compute Pitch and Roll from Accelerometer (gravity vector)
    % Roll: rotation around X-axis -> atan2(ay, az)
    % Pitch: rotation around Y-axis -> atan2(-ax, sqrt(ay^2 + az^2))
    norm_acc = sqrt(ax*ax + ay*ay + az*az);
    if norm_acc > 3.0 && norm_acc < 18.0 % Validate gravity is within sensible bounds (0.3g - 1.8g)
        acc_roll  = atan2(ay, az);
        acc_pitch = atan2(-ax, sqrt(ay*ay + az*az));
    else
        % Under high linear acceleration / freefall, trust only gyro
        acc_roll  = est_roll;
        acc_pitch = est_pitch;
    end

    % 2. Gyro Kinematic Transformation to Euler Angle Rates
    % [dphi/dt; dtheta/dt; dpsi/dt] = W * [p; q; r]
    sr = sin(est_roll);
    cr = cos(est_roll);
    tp = tan(est_pitch);
    cp = cos(est_pitch);

    if abs(cp) < 0.05
        cp = 0.05; % Prevent gimbal lock singularity
    end

    roll_dot  = gx + (gy * sr + gz * cr) * tp;
    pitch_dot = gy * cr - gz * sr;
    yaw_dot   = (gy * sr + gz * cr) / cp;

    % 3. Complementary Filter Fusion
    % Alpha factor: 0.995 gives high weight to low-noise gyro at 1 kHz,
    % while continuously correcting drift with gravity reference.
    alpha = 0.995;

    est_roll  = alpha * (est_roll + roll_dot * dt) + (1.0 - alpha) * acc_roll;
    est_pitch = alpha * (est_pitch + pitch_dot * dt) + (1.0 - alpha) * acc_pitch;
    est_yaw   = est_yaw + yaw_dot * dt; % Yaw has no magnetic reference, pure integration

    % Bound angles between -pi and +pi
    if est_roll > pi
        est_roll = est_roll - 2.0*pi;
    elseif est_roll < -pi
        est_roll = est_roll + 2.0*pi;
    end

    if est_pitch > pi
        est_pitch = est_pitch - 2.0*pi;
    elseif est_pitch < -pi
        est_pitch = est_pitch + 2.0*pi;
    end

    roll  = est_roll;
    pitch = est_pitch;
    yaw   = est_yaw;
end
