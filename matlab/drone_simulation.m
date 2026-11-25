%% drone_simulation.m
% Pro-Grade 6-DOF Quaternion Quadcopter Simulator (No Gimbal Lock)
% Stress-tests:
%   - 220 Hz motor vibration harmonics + IMU noise (Biquad LPF + Notch)
%   - External wind gust disturbance rejection
%   - Battery voltage sag (16.4V -> 14.6V) compensation
%   - Angle Mode steps + full 360-degree Acro barrel roll + self-level recovery
clear; close all; clc;

fprintf('============================================================\n');
fprintf('  Running Pro 6-DOF Quaternion Flight Simulation & Stress   \n');
fprintf('============================================================\n');

init_drone_parameters;

%% 1. Simulation Timing & Quaternion State Initialization
t_final = 10.0;
dt      = control.Ts;
time    = 0:dt:t_final;
N       = length(time);

% Rigid body states:
% pos = [x; y; z] (NED), vel = [vx; vy; vz], q = [q0; q1; q2; q3], omega = [p; q; r]
pos   = [0; 0; 0];
vel   = [0; 0; 0];
q     = [1; 0; 0; 0];
omega = [0; 0; 0];
rotor_omega = ones(4, 1) * drone.omega_idle;

% Exact hover throttle
omega_hover = sqrt((drone.m * drone.g) / (4.0 * drone.kt));
pwm_hover   = drone.pwm_min + ((omega_hover - drone.omega_idle) / (drone.omega_max - drone.omega_idle)) * (drone.pwm_max - drone.pwm_min);
hover_thr   = min(max((pwm_hover - drone.pwm_idle) / (drone.pwm_max - drone.pwm_idle), 0.25), 0.70);

%% 2. Multi-Phase Flight Mission Setup
rc_commands = zeros(4, N);
flight_modes = ones(1, N, 'uint8'); % Default: Angle Mode (1)
vbat_profile = linspace(16.4, 14.6, N); % Simulated 4S LiPo discharge sag

for k = 1:N
    t = time(k);
    thr = 0.0; roll_cmd = 0.0; pitch_cmd = 0.0; yaw_cmd = 0.0;
    mode_k = uint8(1);

    if t < 0.5
        thr = 0.0;                         % Armed idle on ground
    elseif t < 2.0
        thr = hover_thr + 0.14;            % Strong climb to ~14m headroom
    elseif t < 3.0
        thr = hover_thr + 0.01;            % Hover (+ wind gust at t=2.0..2.2s)
    elseif t < 4.5
        thr = hover_thr + 0.02;
        roll_cmd = deg2rad(15.0);          % +15 deg Roll step
    elseif t < 6.0
        thr = hover_thr + 0.04;
        pitch_cmd = deg2rad(-15.0);        % -15 deg Pitch step
    elseif t < 7.0
        mode_k = uint8(0);                 % Switch to ACRO mode for 360-deg Flip!
        thr = hover_thr + 0.10;
        roll_cmd = 2.0 * pi / 1.0 * 0.7;   % ~4.4 rad/s (252 deg/s)
        if t >= 6.82
            roll_cmd = 0.0;
        end
    elseif t < 8.5
        mode_k = uint8(1);                 % Snap back to ANGLE mode self-level recovery
        thr = hover_thr + 0.08;
    else
        thr = hover_thr + 0.02;
        yaw_cmd = 0.8;                     % 0.8 rad/s yaw rotation
    end

    rc_commands(:, k) = [thr; roll_cmd; pitch_cmd; yaw_cmd];
    flight_modes(k)   = mode_k;
end

%% 3. Logging Arrays
log_pos       = zeros(3, N);
log_euler     = zeros(3, N);
log_est_euler = zeros(3, N);
log_gyro_raw  = zeros(3, N);
log_gyro_filt = zeros(3, N);
log_motors    = zeros(4, N);
log_wind_tau  = zeros(3, N);

%% 4. Main 1 kHz Closed-Loop Simulation
fprintf('Simulating %d steps (%.1f s) with 220 Hz vibration & wind gust...\n', N, t_final);
armed   = uint8(1);
airmode = uint8(1);

% Reset controller persistent state cleanly before run
clear flight_controller attitude_estimator biquad_filter ai_neuro_compensator;
rng(42); % Reproducible noise seed

for k = 1:N
    t = time(k);
    q0 = q(1); q1 = q(2); q2 = q(3); q3 = q(4);

    % Rotation matrix Body -> Inertial (from unit quaternion)
    R_IB = [1 - 2*(q2^2 + q3^2), 2*(q1*q2 - q0*q3),     2*(q1*q3 + q0*q2);
            2*(q1*q2 + q0*q3),     1 - 2*(q1^2 + q3^2), 2*(q2*q3 - q0*q1);
            2*(q1*q3 - q0*q2),     2*(q2*q3 + q0*q1),     1 - 2*(q1^2 + q2^2)];
    R_BI = R_IB';

    % True Euler angles for logging
    true_roll  = atan2(2*(q0*q1 + q2*q3), 1 - 2*(q1^2 + q2^2));
    true_pitch = asin(max(min(2*(q0*q2 - q3*q1), 1.0), -1.0));
    true_yaw   = atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2^2 + q3^2));

    % --- Sensor Simulation (MPU6000 with 220 Hz Frame Resonance + Noise) ---
    vib_220hz = 0.08 * sin(2 * pi * 220.0 * t); % Strong 220 Hz prop-wash vibration
    accel_meas = R_BI * [0; 0; drone.g] + 0.12 * randn(3, 1);
    gyro_meas  = omega + [vib_220hz; -0.8*vib_220hz; 0.3*vib_220hz] + 0.008 * randn(3, 1);

    % --- Execute Flight Controller ---
    rc_in = rc_commands(:, k);
    [motors_pwm, telem] = flight_controller(rc_in, accel_meas, gyro_meas, armed, flight_modes(k), dt, vbat_profile(k), airmode);

    % --- Actuator Dynamics (Vbat scaling + 1st-Order Motor Lag) ---
    vbat_thrust_factor = vbat_profile(k) / drone.vbat_nom;
    pwm_norm = max(min((motors_pwm - drone.pwm_min) / (drone.pwm_max - drone.pwm_min), 1.0), 0.0);
    rotor_omega_cmd = (drone.omega_idle + pwm_norm * (drone.omega_max - drone.omega_idle)) * sqrt(vbat_thrust_factor);
    rotor_omega     = rotor_omega + (dt / drone.motor_tau) * (rotor_omega_cmd - rotor_omega);

    thrusts = drone.kt * (rotor_omega .^ 2);
    torques = drone.km * (rotor_omega .^ 2);
    T_total = sum(thrusts);

    % Wind gust disturbance torque impulse at t = 2.0 .. 2.2 s
    tau_wind = [0; 0; 0];
    if t >= 2.0 && t <= 2.2
        tau_wind = [0.045; -0.025; 0.0];
    end

    tau_phi   = drone.d * (-thrusts(1) - thrusts(2) + thrusts(3) + thrusts(4));
    tau_theta = drone.d * (-thrusts(1) + thrusts(2) - thrusts(3) + thrusts(4));
    tau_psi   = -torques(1) + torques(2) + torques(3) - torques(4);
    tau_body  = [tau_phi; tau_theta; tau_psi] + tau_wind - drone.drag_rot * omega;

    % --- 6-DOF Rigid-Body Dynamics Integration ---
    omega_dot = drone.J \ (tau_body - cross(omega, drone.J * omega));
    omega     = omega + omega_dot * dt;

    % Quaternion kinematics integration (singularity-free)
    p = omega(1); q_rate = omega(2); r = omega(3);
    q_dot = 0.5 * [-q1*p - q2*q_rate - q3*r;
                    q0*p + q2*r      - q3*q_rate;
                    q0*q_rate - q1*r + q3*p;
                    q0*r + q1*q_rate - q2*p];
    q = q + q_dot * dt;
    q = q / norm(q);

    % Translational dynamics
    acc_inertial = (R_IB * [0; 0; -T_total] + [0; 0; drone.m * drone.g] - drone.drag_lin * vel) / drone.m;
    if pos(3) >= 0 && acc_inertial(3) > 0 && T_total < (drone.m * drone.g)
        vel(3) = 0; acc_inertial(3) = 0; pos(3) = 0;
    end
    vel = vel + acc_inertial * dt;
    pos = pos + vel * dt;
    if pos(3) > 0
        pos(3) = 0; vel(3) = 0;
    end

    % Log
    log_pos(:, k)       = pos;
    log_euler(:, k)     = [true_roll; true_pitch; true_yaw];
    log_est_euler(:, k) = telem(1:3);
    log_gyro_raw(:, k)  = gyro_meas;
    log_gyro_filt(:, k) = telem(4:6);
    log_motors(:, k)    = motors_pwm;
    log_wind_tau(:, k)  = tau_wind;
end

%% 5. Export Comprehensive Multi-Panel Plot
out_dir = fullfile(fileparts(mfilename('fullpath')), '..', 'docs');

fig1 = figure('Name', 'MAMBA F405 Pro 6-DOF Simulation', 'Color', 'w', 'Position', [80, 60, 1400, 880], 'Visible', 'off');

subplot(3, 2, 1);
plot(time, rad2deg(log_euler(1, :)), 'r', 'LineWidth', 1.7); hold on;
plot(time, rad2deg(log_est_euler(1, :)), 'r:', 'LineWidth', 1.3);
plot(time, rad2deg(log_euler(2, :)), 'b', 'LineWidth', 1.7);
plot(time, rad2deg(log_euler(3, :)), 'k', 'LineWidth', 1.4);
xline(2.0, '--m', 'Wind Gust'); xline(6.0, '--g', 'Acro Flip'); xline(7.0, '--c', 'Angle Recov.');
grid on; xlabel('Time (s)'); ylabel('Angle (deg)');
title('Attitude Response (Quaternion AHRS + Wind Gust + Acro Flip)');
legend('\phi True', '\phi Mahony Est', '\theta True', '\psi True', 'Location', 'best');

subplot(3, 2, 2);
plot(time, rad2deg(log_gyro_raw(1, :)), 'Color', [0.85 0.65 0.65], 'LineWidth', 0.8); hold on;
plot(time, rad2deg(log_gyro_filt(1, :)), 'r', 'LineWidth', 1.6);
plot(time, rad2deg(log_gyro_filt(2, :)), 'b', 'LineWidth', 1.4);
grid on; xlabel('Time (s)'); ylabel('Rate (deg/s)');
title('Gyro Filtering: Raw (220Hz Vib + Noise) vs 90Hz LPF + 220Hz Notch');
legend('p Raw (Noisy)', 'p Filtered', 'q Filtered', 'Location', 'best');

subplot(3, 2, 3);
plot(time, log_motors(1, :), 'LineWidth', 1.2); hold on;
plot(time, log_motors(2, :), 'LineWidth', 1.2);
plot(time, log_motors(3, :), 'LineWidth', 1.2);
plot(time, log_motors(4, :), 'LineWidth', 1.2);
yline(1060, 'k:'); yline(2000, 'k:');
grid on; xlabel('Time (s)'); ylabel('PWM (\mus)');
title('ESC Motor Outputs (With Airmode Desaturation)');
legend('M1 (RR)', 'M2 (FR)', 'M3 (RL)', 'M4 (FL)', 'Location', 'best');

subplot(3, 2, 4);
plot(time, -log_pos(3, :), 'Color', [0 0.55 0.15], 'LineWidth', 1.8);
grid on; xlabel('Time (s)'); ylabel('Altitude (m)');
title('Altitude Trajectory');

subplot(3, 2, 5);
plot(time, log_pos(1, :), 'LineWidth', 1.5); hold on;
plot(time, log_pos(2, :), 'LineWidth', 1.5);
grid on; xlabel('Time (s)'); ylabel('Position (m)');
title('Horizontal Displacement (X North, Y East)');
legend('X (m)', 'Y (m)', 'Location', 'best');

subplot(3, 2, 6);
yyaxis left;
plot(time, vbat_profile, 'b', 'LineWidth', 1.6);
ylabel('Battery Voltage (V)');
yyaxis right;
plot(time, log_wind_tau(1, :), 'm', 'LineWidth', 1.6);
ylabel('Wind Disturbance Torque (N\cdotm)');
grid on; xlabel('Time (s)');
title('4S LiPo Voltage Sag & Wind Gust Injection');

exportgraphics(fig1, fullfile(out_dir, 'simulation_results.png'), 'Resolution', 150);
close(fig1);

%% 6. Export 3D Trajectory Plot
fig2 = figure('Name', '3D Quadcopter Flight Path', 'Color', 'w', 'Position', [120, 120, 900, 680], 'Visible', 'off');
plot3(log_pos(1, :), log_pos(2, :), -log_pos(3, :), 'b-', 'LineWidth', 2.0); hold on;
scatter3(log_pos(1, 1), log_pos(2, 1), -log_pos(3, 1), 80, 'g', 'filled');
scatter3(log_pos(1, end), log_pos(2, end), -log_pos(3, end), 80, 'r', 'filled');
grid on; axis equal; view(40, 28);
xlabel('X North (m)'); ylabel('Y East (m)'); zlabel('Altitude Z (m)');
title('3D Flight Trajectory (Takeoff, Wind Gust, Roll/Pitch Steps, 360^\circ Flip & Recovery)');
legend('Flight Path', 'Start', 'End', 'Location', 'best');
exportgraphics(fig2, fullfile(out_dir, '3d_flight_trajectory.png'), 'Resolution', 150);
close(fig2);

fprintf('Done! Exported simulation_results.png and 3d_flight_trajectory.png.\n');
