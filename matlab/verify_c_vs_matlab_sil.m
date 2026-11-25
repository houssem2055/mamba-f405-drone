%% verify_c_vs_matlab_sil.m
% Cross-verifies the Compiled C Firmware SIL trace (tests/sil_c_results.csv)
% against the MATLAB flight_controller.m implementation on the identical mission.
clear; close all; clc;

fprintf('============================================================\n');
fprintf('  Cross-Verifying C Firmware SIL vs MATLAB Reference Model  \n');
fprintf('============================================================\n');

script_dir = fileparts(mfilename('fullpath'));
csv_path   = fullfile(script_dir, '..', 'tests', 'sil_c_results.csv');
if ~exist(csv_path, 'file')
    error('Run tests/test_firmware.exe first to generate tests/sil_c_results.csv');
end

c_data = readtable(csv_path);

init_drone_parameters;
dt = 0.001;
omega_hover = sqrt((drone.m * drone.g) / (4.0 * drone.kt));
pwm_hover   = drone.pwm_min + ((omega_hover - drone.omega_idle) / (drone.omega_max - drone.omega_idle)) * (drone.pwm_max - drone.pwm_min);
hover_thr   = (pwm_hover - drone.pwm_idle) / (drone.pwm_max - drone.pwm_idle);

q = [1; 0; 0; 0];
omega = [0; 0; 0];
rw = ones(4, 1) * drone.omega_idle;
clear flight_controller attitude_estimator biquad_filter;

m_roll = zeros(5000, 1);
m_motors = zeros(5000, 4);

for k = 1:5000
    t = (k - 1) * dt;
    thr = 0.0;
    if t >= 0.5 && t < 1.5, thr = hover_thr + 0.10; elseif t >= 1.5, thr = hover_thr + 0.02; end
    rc_roll = 0.0;
    if t >= 2.5, rc_roll = 15.0 * pi / 180.0; end

    q0 = q(1); q1 = q(2); q2 = q(3); q3 = q(4);
    r31 = 2*(q1*q3 - q0*q2); r32 = 2*(q2*q3 + q0*q1); r33 = 1 - 2*(q1^2 + q2^2);
    vib = 0.08 * sin(2 * pi * 220.0 * t);

    accel = [r31; r32; r33] * drone.g;
    gyro  = omega + [vib; -0.8*vib; 0.3*vib];

    [mpwm, ~] = flight_controller([thr; rc_roll; 0; 0], accel, gyro, uint8(1), uint8(1), dt, 16.0, uint8(1));
    m_motors(k, :) = mpwm';

    w_cmd = drone.omega_idle + ((mpwm - 1000.0) / 1000.0) * 2000.0;
    rw = rw + (dt / drone.motor_tau) * (w_cmd - rw);
    T = drone.kt * (rw.^2); Q = drone.km * (rw.^2);

    tau = [drone.d*(-T(1)-T(2)+T(3)+T(4)); drone.d*(-T(1)+T(2)-T(3)+T(4)); -Q(1)+Q(2)+Q(3)-Q(4)] - drone.drag_rot*omega;
    omega = omega + (drone.J \ (tau - cross(omega, drone.J * omega))) * dt;
    p = omega(1); qr = omega(2); r = omega(3);
    q = q + 0.5*[-q1*p-q2*qr-q3*r; q0*p+q2*r-q3*qr; q0*qr-q1*r+q3*p; q0*r+q1*qr-q2*p]*dt;
    q = q / norm(q);

    m_roll(k) = rad2deg(atan2(2*(q(1)*q(2) + q(3)*q(4)), 1 - 2*(q(2)^2 + q(3)^2)));
end

idx = 1:5:5000;
max_roll_diff_deg = max(abs(c_data.roll_deg - m_roll(idx)));
max_pwm_diff_us   = max(abs(c_data.m1 - m_motors(idx, 1)));

fprintf('C vs MATLAB SIL Comparison (5000 steps @ 1 kHz):\n');
fprintf('  Max Roll Angle Difference: %.6f deg\n', max_roll_diff_deg);
fprintf('  Max Motor 1 PWM Diff:      %.6f us\n',  max_pwm_diff_us);

if max_roll_diff_deg < 0.05 && max_pwm_diff_us < 0.5
    fprintf('[VERIFIED] C Firmware and MATLAB Model match within single-precision tolerance!\n');
else
    error('Divergence exceeded tolerance!');
end
