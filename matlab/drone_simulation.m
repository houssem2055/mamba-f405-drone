%% drone_simulation.m
% 6-DOF Non-Linear Quadcopter Flight Dynamic Simulator with Mamba F405 Controller
clear; close all; clc;

fprintf('====================================================\n');
fprintf('  Running Quadcopter 6-DOF Closed-Loop Simulation   \n');
fprintf('====================================================\n');


init_drone_parameters;

%% 1. Simulation Setup
t_final = 8.0;                    % Simulation duration in seconds
dt      = control.Ts;             % 1 ms sample time (1 kHz loop)
time    = 0:dt:t_final;
N       = length(time);

% State Vector: x = [pos(3); vel(3); euler(3); body_rates(3)] -> 12 states
% pos = [x, y, z]' (NED: z down)
% vel = [u, v, w]' (inertial frame)
% euler = [phi, theta, psi]' (roll, pitch, yaw)
% body_rates = [p, q, r]' (gyro measurements)
state = zeros(12, 1);
state(3) = 0.0; % Start on ground (z=0)

% Log vectors
log_state     = zeros(12, N);
log_motors    = zeros(4, N);
log_telemetry = zeros(6, N);
log_commands  = zeros(4, N);

%% 2. Generate Mission Trajectory Commands
% rc_cmd: [throttle (0..1); roll_cmd (rad); pitch_cmd (rad); yaw_rate_cmd (rad/s)]
rc_commands = zeros(4, N);

% Takeoff hover throttle (~0.45 throttle holds hover)
hover_thr = (drone.m * drone.g) / (4 * drone.kt * (drone.omega_max * 0.5)^2);
hover_thr = min(max(hover_thr, 0.35), 0.55);

for k = 1:N
    t = time(k);
    thr = 0.0;
    roll_cmd = 0.0;
    pitch_cmd = 0.0;
    yaw_cmd = 0.0;
    
    if t < 0.5
        thr = 0.1; % Idle on ground
    elseif t < 3.0
        thr = hover_thr; % Takeoff & hover
    elseif t < 5.0
        thr = hover_thr;
        roll_cmd = 0.15; % 8.6 degrees roll step
    elseif t < 7.0
        thr = hover_thr;
        roll_cmd = 0.0;
        pitch_cmd = -0.15; % Forward pitch step
    else
        thr = hover_thr;
        yaw_cmd = 0.5; % 0.5 rad/s yaw rotation
    end
    
    rc_commands(:, k) = [thr; roll_cmd; pitch_cmd; yaw_cmd];
end

%% 3. Main Simulation Loop
fprintf('Simulating %d steps (%.1f seconds)...\n', N, t_final);
armed = uint8(1);
flight_mode = uint8(1); % Angle Mode

for k = 1:N
    % Current states
    pos   = state(1:3);
    vel   = state(4:6);
    euler = state(7:9);
    omega = state(10:12);
    
    phi   = euler(1);
    theta = euler(2);
    psi   = euler(3);
    p = omega(1); q = omega(2); r = omega(3);
    
    % --- Sensor Emulation (MPU6000 on Mamba F405) ---
    % Accelerometer: senses gravity in body frame + linear acceleration + noise
    R_IB = [cos(theta)*cos(psi), cos(theta)*sin(psi), -sin(theta);
            sin(phi)*sin(theta)*cos(psi) - cos(phi)*sin(psi), sin(phi)*sin(theta)*sin(psi) + cos(phi)*cos(psi), sin(phi)*cos(theta);
            cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi), cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi), cos(phi)*cos(theta)];
    R_BI = R_IB';
    
    % Gravity vector in body frame: R_BI * [0; 0; -g] (NED coordinates)
    g_body = R_BI * [0; 0; -drone.g];
    
    % Add realistic IMU noise
    acc_noise  = 0.08 * randn(3, 1);
    gyro_noise = 0.005 * randn(3, 1);
    
    accel_meas = -g_body + acc_noise;
    gyro_meas  = omega + gyro_noise;
    
    % --- Call Embedded Flight Controller ---
    rc_in = rc_commands(:, k);
    [motors_pwm, telem] = flight_controller(rc_in, accel_meas, gyro_meas, armed, flight_mode, dt);
    
    % --- Actuator Model (PWM -> Rotor Speed -> Thrust & Torques) ---
    % Map PWM (1000..2000 us) to rotor speed (rad/s)
    pwm_norm = (motors_pwm - drone.pwm_min) / (drone.pwm_max - drone.pwm_min);
    pwm_norm = max(min(pwm_norm, 1.0), 0.0);
    rotor_omega = drone.omega_idle + pwm_norm * (drone.omega_max - drone.omega_idle);
    
    % Rotor thrusts & drag torques: T = kt * w^2, Q = km * w^2
    thrusts = drone.kt * (rotor_omega .^ 2);
    torques = drone.km * (rotor_omega .^ 2);
    
    T_total = sum(thrusts);
    
    % Torques in body frame (Quad-X configuration):
    % Motor 1 (Rear-Right, CCW):  [-d, -d]
    % Motor 2 (Front-Right, CW):  [+d, -d]
    % Motor 3 (Rear-Left, CW):    [-d, +d]
    % Motor 4 (Front-Left, CCW):  [+d, +d]
    tau_phi   = drone.d * (-thrusts(1) - thrusts(2) + thrusts(3) + thrusts(4));
    tau_theta = drone.d * (-thrusts(1) + thrusts(2) - thrusts(3) + thrusts(4));
    tau_psi   = -torques(1) + torques(2) + torques(3) - torques(4);
    
    tau_body = [tau_phi; tau_theta; tau_psi];
    
    % --- 6-DOF Equations of Motion Integration ---
    % 1. Rotational Accelerations (Euler's Equations)
    % J * domega/dt + omega x (J * omega) = tau
    omega_dot = drone.J \ (tau_body - cross(omega, drone.J * omega));
    
    % 2. Kinematic Equations (Body rates -> Euler rates)
    W_inv = [1, sin(phi)*tan(theta), cos(phi)*tan(theta);
             0, cos(phi),            -sin(phi);
             0, sin(phi)/cos(theta), cos(phi)/cos(theta)];
    euler_dot = W_inv * omega;
    
    % 3. Translational Accelerations (Newton's 2nd Law)
    thrust_inertial = R_IB * [0; 0; -T_total];
    gravity_inertial = [0; 0; drone.m * drone.g];
    drag_inertial    = -0.15 * vel; % Linear aerodynamic drag
    
    acc_inertial = (thrust_inertial + gravity_inertial + drag_inertial) / drone.m;
    
    % Ground contact model: cannot penetrate below z=0
    if state(3) >= 0 && acc_inertial(3) > 0 && T_total < (drone.m * drone.g)
        vel(3) = 0;
        acc_inertial(3) = 0;
        state(3) = 0;
    end
    
    % Euler integration
    state(1:3)   = state(1:3) + vel * dt;
    state(4:6)   = state(4:6) + acc_inertial * dt;
    state(7:9)   = state(7:9) + euler_dot * dt;
    state(10:12) = state(10:12) + omega_dot * dt;
    
    % Enforce ground plane
    if state(3) > 0
        state(3) = 0;
        state(6) = 0;
    end
    
    % Save logs
    log_state(:, k)     = state;
    log_motors(:, k)    = motors_pwm;
    log_telemetry(:, k) = telem;
    log_commands(:, k)  = rc_in;
end

fprintf('Simulation complete. Generating response plots...\n');

%% 4. Plotting Results
figure('Name', 'Mamba F405 Drone Closed-Loop Flight Simulation', 'Color', 'w', 'Position', [100, 100, 1200, 800]);

% 1. Attitude Angles (Roll, Pitch, Yaw)
subplot(2, 2, 1);
plot(time, rad2deg(log_state(7, :)), 'r', 'LineWidth', 1.8); hold on;
plot(time, rad2deg(log_commands(2, :)), 'r--', 'LineWidth', 1.2);
plot(time, rad2deg(log_state(8, :)), 'b', 'LineWidth', 1.8);
plot(time, rad2deg(log_commands(3, :)), 'b--', 'LineWidth', 1.2);
plot(time, rad2deg(log_state(9, :)), 'k', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)'); ylabel('Angle (deg)');
title('Euler Angles (Solid: Actual, Dashed: Commanded)');
legend('\phi (Roll)', '\phi_{cmd}', '\theta (Pitch)', '\theta_{cmd}', '\psi (Yaw)', 'Location', 'best');

% 2. Body Angular Rates
subplot(2, 2, 2);
plot(time, rad2deg(log_state(10, :)), 'r', 'LineWidth', 1.5); hold on;
plot(time, rad2deg(log_state(11, :)), 'b', 'LineWidth', 1.5);
plot(time, rad2deg(log_state(12, :)), 'k', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)'); ylabel('Rate (deg/s)');
title('Body Angular Rates (Gyro)');
legend('p (Roll rate)', 'q (Pitch rate)', 'r (Yaw rate)', 'Location', 'best');

% 3. Motor PWM Outputs
subplot(2, 2, 3);
plot(time, log_motors(1, :), 'LineWidth', 1.3); hold on;
plot(time, log_motors(2, :), 'LineWidth', 1.3);
plot(time, log_motors(3, :), 'LineWidth', 1.3);
plot(time, log_motors(4, :), 'LineWidth', 1.3);
yline(1000, 'k:'); yline(2000, 'k:');
grid on;
xlabel('Time (s)'); ylabel('Pulse Width (\mus)');
title('ESC Motor Outputs (1000 - 2000 \mus)');
legend('M1 (Rear-R)', 'M2 (Front-R)', 'M3 (Rear-L)', 'M4 (Front-L)', 'Location', 'best');

% 4. Altitude (Position Z)
subplot(2, 2, 4);
plot(time, -log_state(3, :), 'Color', [0 0.5 0], 'LineWidth', 1.8);
grid on;
xlabel('Time (s)'); ylabel('Height (m)');
title('Drone Altitude');

fprintf('Done! All results successfully plotted.\n');
