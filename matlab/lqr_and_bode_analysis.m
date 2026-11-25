%% lqr_and_bode_analysis.m
% Linearized State-Space Modeling, Optimal LQR Synthesis, and Bode Stability Margins
% Uses MATLAB Control System Toolbox to analytically verify Roll/Pitch loop margins
clear; close all; clc;

fprintf('============================================================\n');
fprintf('  Linear State-Space, LQR Synthesis & Bode Margin Analysis  \n');
fprintf('============================================================\n');

init_drone_parameters;

%% 1. Linearize Rotational Dynamics around Hover
omega_hover = sqrt((drone.m * drone.g) / (4.0 * drone.kt));
dT_dPWM = 2.0 * drone.kt * omega_hover * ((drone.omega_max - drone.omega_idle) / (drone.pwm_max - drone.pwm_min));
G_tau_pwm = 4.0 * drone.d * dT_dPWM; % Roll/Pitch torque per unit differential PWM (N*m / us)

% Plant transfer function from PWM delta (u_roll) -> angular rate (p)
% Includes 1st-order motor lag (tau = 20 ms) and rotational inertia Ixx
s = tf('s');
G_motor = 1.0 / (drone.motor_tau * s + 1.0);
G_body  = G_tau_pwm / (drone.Ixx * s + drone.drag_rot);
G_rate_plant = G_motor * G_body;
G_angle_plant = G_rate_plant / s;

%% 2. Frequency Domain Stability Margins of Current Cascaded PID
Kp_eff = control.Kp_rate_roll * control.pwm_scale;
Ki_eff = control.Ki_rate_roll * control.pwm_scale;
Kd_eff = control.Kd_rate_roll * control.pwm_scale;

% Rate PID controller with 70 Hz D-term filter
w_d = 2 * pi * filter.dterm_lpf_hz;
C_rate = Kp_eff + Ki_eff / s + (Kd_eff * s) / (s / w_d + 1.0);

L_rate = C_rate * G_rate_plant;
[Gm, Pm, Wcg, Wcp] = margin(L_rate);
Gm_dB = 20 * log10(Gm);

fprintf('Current Rate Loop Stability Margins:\n');
fprintf('  Gain Margin:  %.2f dB (at %.2f rad/s = %.2f Hz)\n', Gm_dB, Wcg, Wcg/(2*pi));
fprintf('  Phase Margin: %.2f deg (at %.2f rad/s = %.2f Hz)\n', Pm, Wcp, Wcp/(2*pi));

%% 3. Optimal LQR Controller Synthesis (State vector x = [phi; p; motor_state])
% Continuous state-space:
%   d(phi)/dt = p
%   d(p)/dt   = (-drag_rot*p + G_tau_pwm * u_m) / Ixx
%   d(u_m)/dt = (-u_m + u_cmd) / motor_tau
A = [0, 1, 0;
     0, -drone.drag_rot / drone.Ixx, G_tau_pwm / drone.Ixx;
     0, 0, -1 / drone.motor_tau];
B = [0; 0; 1 / drone.motor_tau];
C = [1, 0, 0];
D = 0;

sys_ss = ss(A, B, C, D);

% LQR Weighting Matrices (Bryson's Rule tuning)
Q_lqr = diag([2500.0, 25.0, 0.01]); % Prioritize tight angle & rate tracking
R_lqr = 0.08;                       % Penalize excessive PWM effort
[K_lqr, ~, eigs_cl] = lqr(sys_ss, Q_lqr, R_lqr);

% Feedforward DC gain Nbar for zero steady-state tracking error on angle step
Nbar = -1.0 / (C * ((A - B * K_lqr) \ B));
sys_lqr_cl = ss(A - B * K_lqr, B * Nbar, C, 0);

fprintf('\nSynthesized Optimal LQR Gains (x = [angle(rad); rate(rad/s); motor_us]):\n');
fprintf('  K_lqr = [%.3f, %.3f, %.4f]  |  Nbar = %.3f\n', K_lqr(1), K_lqr(2), K_lqr(3), Nbar);
fprintf('  Closed-loop poles:\n');
disp(eigs_cl);

% Compare Cascaded PID closed-loop angle step vs LQR
T_rate_cl = feedback(L_rate, 1);
L_angle   = control.Kp_angle_roll * T_rate_cl / s;
T_pid_cl  = feedback(L_angle, 1);

info_pid = stepinfo(T_pid_cl);
info_lqr = stepinfo(sys_lqr_cl);

fprintf('Step Response Comparison (15 deg Roll Step):\n');
fprintf('  Cascaded PID -> Rise: %.3f s | Settling: %.3f s | Overshoot: %.2f%%\n', ...
    info_pid.RiseTime, info_pid.SettlingTime, info_pid.Overshoot);
fprintf('  Optimal LQR  -> Rise: %.3f s | Settling: %.3f s | Overshoot: %.2f%%\n', ...
    info_lqr.RiseTime, info_lqr.SettlingTime, info_lqr.Overshoot);

%% 4. Save Bode Margin & Step Comparison Plot
out_dir = fullfile(fileparts(mfilename('fullpath')), '..', 'docs');
fig = figure('Name', 'LQR & Bode Margin Analysis', 'Color', 'w', 'Position', [100, 100, 1200, 620], 'Visible', 'off');

subplot(1, 2, 1);
margin(L_rate);
grid on;
title(sprintf('Open-Loop Rate Bode (GM=%.1fdB, PM=%.1fdeg)', Gm_dB, Pm));

subplot(1, 2, 2);
t_step = 0:0.001:1.2;
[y_pid, ~] = step(15.0 * T_pid_cl, t_step);
[y_lqr, ~] = step(15.0 * sys_lqr_cl, t_step);
plot(t_step, y_pid, 'b-', 'LineWidth', 2.0); hold on;
plot(t_step, y_lqr, 'r--', 'LineWidth', 2.0);
yline(15.0, 'k:');
grid on;
xlabel('Time (s)'); ylabel('Roll Angle (deg)');
title('15^\circ Roll Step: Cascaded PID vs Optimal LQR');
legend('Cascaded PID (Deployed)', 'Optimal LQR (Theoretical)', 'Target 15^\circ', 'Location', 'best');

exportgraphics(fig, fullfile(out_dir, 'bode_lqr_analysis.png'), 'Resolution', 150);
close(fig);

fprintf('Saved docs/bode_lqr_analysis.png.\n');
