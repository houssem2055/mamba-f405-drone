%% run_verification.m
% Automated verification script for MATLAB simulation & algorithms
clear; clc;
fprintf('====================================================\n');
fprintf('  Running MATLAB Quadcopter Model Verification      \n');
fprintf('====================================================\n');

%% 1. Initialize parameters
init_drone_parameters;
assert(drone.m > 0, 'Drone mass must be positive');
assert(drone.g > 9.0 && drone.g < 10.0, 'Gravity constant valid');
fprintf('  [PASS] Physical drone parameters initialized\n');

%% 2. Test Attitude Estimator
accel_flat = [0; 0; 9.81];
gyro_zero = [0; 0; 0];
dt = 0.001;

% Run for 500 steps
for i = 1:500
    [roll, pitch, yaw] = attitude_estimator(accel_flat, gyro_zero, dt, false);
end
assert(abs(roll) < 0.01, 'Level roll must be near 0');
assert(abs(pitch) < 0.01, 'Level pitch must be near 0');
fprintf('  [PASS] Attitude estimator level convergence verified\n');

%% 3. Test Flight Controller
rc_hover = [0.5; 0.0; 0.0; 0.0]; % 50% throttle, 0 angles
[motors_pwm, telem] = flight_controller(rc_hover, accel_flat, gyro_zero, uint8(1), uint8(1), dt);
assert(all(motors_pwm >= 1000 & motors_pwm <= 2000), 'Motor PWMs within [1000, 2000] us');
assert(abs(motors_pwm(1) - motors_pwm(2)) < 1.0, 'Hover symmetric M1 == M2');
assert(abs(motors_pwm(3) - motors_pwm(4)) < 1.0, 'Hover symmetric M3 == M4');
fprintf('  [PASS] Flight controller hover output symmetric and bounded\n');

%% 4. Disarmed safety test
[motors_disarmed, ~] = flight_controller(rc_hover, accel_flat, gyro_zero, uint8(0), uint8(1), dt);
assert(all(motors_disarmed == 1000), 'Disarmed motors must be 1000 us');
fprintf('  [PASS] Flight controller disarmed safety output verified\n');

fprintf('====================================================\n');
fprintf('  All MATLAB Verification Tests Passed Successfully! \n');
fprintf('====================================================\n');
