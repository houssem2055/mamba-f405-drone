%% init_drone_parameters.m
% Initialization of Quadcopter Physical Parameters and Controller Gains
% Configured for standard 5-inch / 3-inch FPV Quadcopter with Mamba F405 MK2
clear; clc;

fprintf('==============================================\n');
fprintf('  Initializing Drone Parameters (Mamba F405)  \n');
fprintf('==============================================\n');

%% 1. Physical Parameters
drone.m       = 0.650;           % Total mass (kg) including 4S LiPo battery
drone.g       = 9.81;            % Gravitational acceleration (m/s^2)
drone.arm_len = 0.125;           % Distance from center to motor axis (m) (250mm frame)
drone.d       = drone.arm_len / sqrt(2); % X-distance & Y-distance from COG (m)

% Inertia Matrix (kg*m^2)
drone.Ixx = 4.85e-3;
drone.Iyy = 4.85e-3;
drone.Izz = 8.80e-3;
drone.J   = diag([drone.Ixx, drone.Iyy, drone.Izz]);

% Aerodynamic Coefficients (5-inch prop on 4S LiPo)
drone.kt = 1.15e-6;              % Thrust coefficient (N / (rad/s)^2) (~5.5N max per motor)
drone.km = 1.90e-8;              % Drag torque coefficient (N*m / (rad/s)^2)
drone.cq = drone.km / drone.kt;  % Torque-to-thrust ratio

% Motor / ESC Characteristics
drone.motor_tau   = 0.020;       % First-order motor response time constant (seconds)
drone.pwm_min     = 1000;        % Minimum PWM pulse width (microseconds)
drone.pwm_idle    = 1060;        % Idle spinning PWM (armed idle)
drone.pwm_max     = 2000;        % Maximum PWM pulse width (microseconds)
drone.omega_idle  = 200.0;       % Idle rotor angular velocity (rad/s)
drone.omega_max   = 2200.0;      % Max rotor angular velocity at 2000us (rad/s)

%% 2. Execution Timing & Sampling
control.fs   = 1000.0;           % Flight controller loop frequency (Hz)
control.Ts   = 1.0 / control.fs; % Sample time = 0.001 s (1 ms)

%% 3. Controller Gains (Cascaded Architecture)

% --- Outer Loop: Angle / Attitude Controller (P-Controller) ---
% Converts angle error (rad) -> desired angular rate (rad/s)
control.Kp_angle_roll  = 6.0;
control.Kp_angle_pitch = 6.0;

% --- Inner Loop: Angular Rate Controller (PID Controller) ---
% Roll Rate Loop
control.Kp_rate_roll  = 0.13;
control.Ki_rate_roll  = 0.05;
control.Kd_rate_roll  = 0.0012;

% Pitch Rate Loop
control.Kp_rate_pitch = 0.13;
control.Ki_rate_pitch = 0.05;
control.Kd_rate_pitch = 0.0012;

% Yaw Rate Loop
control.Kp_rate_yaw   = 0.50;
control.Ki_rate_yaw   = 0.20;
control.Kd_rate_yaw   = 0.0000;

% Anti-windup and actuator limits
control.max_rate_roll_pitch = 4.5;  % Max allowable angular rate command (rad/s) ~ 258 deg/s
control.max_rate_yaw        = 3.5;  % Max yaw rate command (rad/s) ~ 200 deg/s
control.i_term_limit        = 120.0;% Max integral contribution (PWM units)

%% 4. Attitude Filter Parameters
filter.alpha_comp = 0.995;       % Complementary filter gyro weight: angle = alpha*(angle+gyro*dt) + (1-alpha)*accel
filter.gyro_lpf_cutoff = 80.0;   % Gyro 2nd order low-pass filter cutoff (Hz)

fprintf('Configuration loaded successfully:\n');
fprintf('  Mass:           %.3f kg\n', drone.m);
fprintf('  Loop Frequency: %.0f Hz (Ts = %.4f s)\n', control.fs, control.Ts);
fprintf('  Roll/Pitch PID: Kp=%.3f, Ki=%.3f, Kd=%.4f\n', ...
    control.Kp_rate_roll, control.Ki_rate_roll, control.Kd_rate_roll);
fprintf('==============================================\n');
