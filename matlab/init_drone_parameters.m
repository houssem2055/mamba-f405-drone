%% init_drone_parameters.m
% Master Initialization of Quadcopter Physical Parameters, Filters, and Controllers
% Configured for 5-inch / 250mm Quad-X FPV Drone with MAMBA F405 MK2 V2 (4S LiPo)
% (Does not call 'clear' so caller scripts keep their local variables.)

fprintf('============================================================\n');
fprintf('  Initializing MAMBA F405 Drone Parameters (Pro Aerospace)  \n');
fprintf('============================================================\n');

%% 1. Physical Airframe & Aerodynamic Parameters
drone.m       = 0.650;           % Total mass (kg) including 4S 1500mAh LiPo battery
drone.g       = 9.81;            % Gravitational acceleration (m/s^2)
drone.arm_len = 0.125;           % Arm radius center-to-motor (m) (250mm diagonal frame)
drone.d       = drone.arm_len / sqrt(2); % Moment arm on X/Y axes (m)

% Moments of Inertia (kg*m^2)
drone.Ixx = 4.85e-3;
drone.Iyy = 4.85e-3;
drone.Izz = 8.80e-3;
drone.J   = diag([drone.Ixx, drone.Iyy, drone.Izz]);

% Propeller Aerodynamic Constants (5-inch prop on 4S LiPo)
drone.kt = 1.15e-6;              % Thrust coefficient (N / (rad/s)^2) (~5.5N max per motor)
drone.km = 1.90e-8;              % Drag torque coefficient (N*m / (rad/s)^2)
drone.cq = drone.km / drone.kt;  % Torque-to-thrust ratio (m)
drone.drag_lin = 0.18;           % Translational aerodynamic drag coefficient (N/(m/s))
drone.drag_rot = 0.0025;         % Rotational aerodynamic damping (N*m/(rad/s))

% ESC & Brushless Motor Dynamics
drone.motor_tau   = 0.020;       % First-order motor response time constant (20 ms)
drone.pwm_min     = 1000;        % Disarmed ESC pulse width (us)
drone.pwm_idle    = 1060;        % Armed idle spin pulse width (us)
drone.pwm_max     = 2000;        % Full throttle pulse width (us)
drone.omega_idle  = 200.0;       % Idle rotor speed (rad/s)
drone.omega_max   = 2200.0;      % Max rotor speed at 2000us (rad/s)
drone.vbat_nom    = 16.0;        % Nominal 4S LiPo reference voltage (V)

%% 2. Execution Timing & Sampling
control.fs   = 1000.0;           % Main flight control loop frequency (Hz)
control.Ts   = 1.0 / control.fs; % Sample period = 0.001 s (1 ms)

%% 3. Digital Filter Parameters (Biquad LPF & Notch)
filter.gyro_lpf_hz     = 90.0;   % 2nd-order Butterworth Gyro LPF cutoff (Hz)
filter.gyro_notch_hz   = 220.0;  % 2nd-order Gyro Notch center frequency (Hz)
filter.gyro_notch_q    = 2.5;    % Notch quality factor
filter.dterm_lpf_hz    = 70.0;   % 2nd-order Butterworth D-term LPF cutoff (Hz)
filter.mahony_kp       = 2.5;    % Quaternion Mahony AHRS proportional gain
filter.mahony_ki       = 0.05;   % Quaternion Mahony AHRS integral gyro-bias gain

%% 4. Cascaded Flight Controller Gains (Matched with firmware/src/flight_controller.c)
% --- Outer Loop: Angle / Attitude P-Controller ---
control.Kp_angle_roll  = 6.0;
control.Kp_angle_pitch = 6.0;
control.max_angle_rad  = 0.7854; % +/- 45 deg tilt limit in Angle Mode

% --- Inner Loop: Angular Rate PID + Feedforward ---
% Roll Rate
control.Kp_rate_roll   = 0.13;
control.Ki_rate_roll   = 0.05;
control.Kd_rate_roll   = 0.0014;
control.Kf_rate_roll   = 0.0020;

% Pitch Rate
control.Kp_rate_pitch  = 0.13;
control.Ki_rate_pitch  = 0.05;
control.Kd_rate_pitch  = 0.0014;
control.Kf_rate_pitch  = 0.0020;

% Yaw Rate
control.Kp_rate_yaw    = 0.50;
control.Ki_rate_yaw    = 0.20;
control.Kd_rate_yaw    = 0.0000;
control.Kf_rate_yaw    = 0.0020;

% Rate Limits, Expo, TPA, and Anti-Windup
control.max_rate_rp    = 4.5;    % Max Roll/Pitch rate command (rad/s) (~258 deg/s)
control.max_rate_yaw   = 3.5;    % Max Yaw rate command (rad/s) (~200 deg/s)
control.rc_expo        = 0.25;   % RC stick curve expo (0 = linear, 1 = cubic)
control.pwm_scale      = 800.0;  % Normalized effort -> PWM us delta scale
control.i_term_limit   = 120.0;  % Max integral contribution in PWM us
control.tpa_breakpoint = 0.50;   % Throttle point where TPA begins attenuating P/D
control.tpa_rate       = 0.25;   % Max P/D attenuation at 100% throttle (25%)

fprintf('Parameters Ready:\n');
fprintf('  Mass: %.3f kg | Loop: %.0f Hz | Mahony AHRS: Kp=%.1f, Ki=%.2f\n', ...
    drone.m, control.fs, filter.mahony_kp, filter.mahony_ki);
fprintf('  Filters: Gyro LPF=%.0f Hz, Notch=%.0f Hz (Q=%.1f), D-LPF=%.0f Hz\n', ...
    filter.gyro_lpf_hz, filter.gyro_notch_hz, filter.gyro_notch_q, filter.dterm_lpf_hz);
fprintf('  Roll/Pitch PID: Kp=%.3f, Ki=%.3f, Kd=%.4f, Kf=%.4f\n', ...
    control.Kp_rate_roll, control.Ki_rate_roll, control.Kd_rate_roll, control.Kf_rate_roll);
fprintf('============================================================\n');
