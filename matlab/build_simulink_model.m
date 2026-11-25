%% build_simulink_model.m
% Programmatically constructs and simulates the Simulink Model:
%   matlab/mamba_f405_quadcopter.slx
% Integrates the exact flight_controller.m block with a 6-DOF Rigid-Body Plant.
clear; close all; clc;

fprintf('============================================================\n');
fprintf('  Building & Verifying Simulink Model (mamba_f405_quadcopter)\n');
fprintf('============================================================\n');

init_drone_parameters;

script_dir = fileparts(mfilename('fullpath'));
cd(script_dir);

model_name = 'mamba_f405_quadcopter';
slx_path   = fullfile(script_dir, [model_name '.slx']);

if bdIsLoaded(model_name)
    close_system(model_name, 0);
end
if exist(slx_path, 'file')
    delete(slx_path);
end

assignin('base', 'drone', drone);
assignin('base', 'control', control);

% Compute hover throttle
omega_hover = sqrt((drone.m * drone.g) / (4.0 * drone.kt));
pwm_hover   = drone.pwm_min + ((omega_hover - drone.omega_idle) / (drone.omega_max - drone.omega_idle)) * (drone.pwm_max - drone.pwm_min);
hover_thr   = min(max((pwm_hover - drone.pwm_idle) / (drone.pwm_max - drone.pwm_idle), 0.25), 0.70);

%% 1. Create Simulink Model & Solver Configuration
sys = new_system(model_name);
open_system(sys);

set_param(model_name, 'SolverType', 'Fixed-step');
set_param(model_name, 'Solver', 'ode1');
set_param(model_name, 'FixedStep', '0.001');
set_param(model_name, 'StopTime', '6.0');

%% 2. Add Blocks
% Command Inputs
add_block('simulink/Sources/Constant', [model_name '/Throttle_Hover'], 'Value', num2str(hover_thr + 0.01), 'Position', [40, 40, 100, 70]);
add_block('simulink/Sources/Step',     [model_name '/Roll_Step_Rad'],  'Time', '1.5', 'Before', '0', 'After', '0.2618', 'SampleTime', '0.001', 'Position', [40, 100, 100, 130]);
add_block('simulink/Sources/Step',     [model_name '/Pitch_Step_Rad'], 'Time', '3.5', 'Before', '0', 'After', '-0.2618', 'SampleTime', '0.001', 'Position', [40, 160, 100, 190]);
add_block('simulink/Sources/Constant', [model_name '/Yaw_Rate_Cmd'],   'Value', '0', 'Position', [40, 220, 100, 250]);
add_block('simulink/Signal Routing/Mux', [model_name '/RC_Mux'], 'Inputs', '4', 'Position', [160, 45, 165, 245]);

add_block('simulink/Sources/Constant', [model_name '/Armed'],       'Value', 'uint8(1)', 'Position', [110, 280, 165, 305]);
add_block('simulink/Sources/Constant', [model_name '/Flight_Mode'], 'Value', 'uint8(1)', 'Position', [110, 320, 165, 345]);
add_block('simulink/Sources/Constant', [model_name '/dt'],          'Value', '0.001',    'Position', [110, 360, 165, 385]);
add_block('simulink/Sources/Constant', [model_name '/Vbat'],        'Value', '16.0',     'Position', [110, 400, 165, 425]);
add_block('simulink/Sources/Constant', [model_name '/Airmode'],     'Value', 'uint8(1)', 'Position', [110, 440, 165, 465]);

% MATLAB Function Block: Flight Controller
fc_blk = add_block('simulink/User-Defined Functions/MATLAB Function', [model_name '/Flight_Controller_Block'], 'Position', [260, 120, 430, 340]);
rt = sfroot;
fc_chart = rt.find('-isa', 'Stateflow.EMChart', 'Path', [model_name '/Flight_Controller_Block']);
fc_chart.Script = sprintf([ ...
    'function [motors_pwm, telemetry] = fcn(rc_cmd, accel, gyro, armed, flight_mode, dt, vbat, airmode)\n' ...
    'coder.extrinsic(''flight_controller'');\n' ...
    'motors_pwm = zeros(4,1);\n' ...
    'telemetry = zeros(6,1);\n' ...
    '[motors_pwm, telemetry] = flight_controller(rc_cmd, accel, gyro, armed, flight_mode, dt, vbat, airmode);\n' ...
]);

% MATLAB Function Block: 6-DOF Plant + Sensors (Discrete 1ms Integration)
plant_blk = add_block('simulink/User-Defined Functions/MATLAB Function', [model_name '/Quad_6DOF_Plant'], 'Position', [520, 120, 700, 340]);
pl_chart = rt.find('-isa', 'Stateflow.EMChart', 'Path', [model_name '/Quad_6DOF_Plant']);
pl_chart.Script = sprintf([ ...
    'function [accel_out, gyro_out, euler_deg, alt_m] = plant(motors_pwm)\n' ...
    'persistent pos vel q omega rotor_omega;\n' ...
    'if isempty(pos)\n' ...
    '    pos = [0;0;0]; vel = [0;0;0]; q = [1;0;0;0]; omega = [0;0;0];\n' ...
    '    rotor_omega = [200;200;200;200];\n' ...
    'end\n' ...
    'dt = 0.001; m = 0.650; g = 9.81; d = 0.125/sqrt(2);\n' ...
    'J = diag([4.85e-3, 4.85e-3, 8.80e-3]); kt = 1.15e-6; km = 1.90e-8;\n' ...
    'pwm_norm = max(min((motors_pwm - 1000.0) / 1000.0, 1.0), 0.0);\n' ...
    'w_cmd = 200.0 + pwm_norm * 2000.0;\n' ...
    'rotor_omega = rotor_omega + (dt / 0.020) * (w_cmd - rotor_omega);\n' ...
    'T = kt * (rotor_omega.^2); Q = km * (rotor_omega.^2);\n' ...
    'tau = [d*(-T(1)-T(2)+T(3)+T(4)); d*(-T(1)+T(2)-T(3)+T(4)); -Q(1)+Q(2)+Q(3)-Q(4)] - 0.0025*omega;\n' ...
    'omega = omega + (J \\ (tau - cross(omega, J*omega))) * dt;\n' ...
    'q0=q(1); q1=q(2); q2=q(3); q3=q(4); p=omega(1); qr=omega(2); r=omega(3);\n' ...
    'q = q + 0.5*[-q1*p-q2*qr-q3*r; q0*p+q2*r-q3*qr; q0*qr-q1*r+q3*p; q0*r+q1*qr-q2*p]*dt;\n' ...
    'q = q / norm(q);\n' ...
    'q0=q(1); q1=q(2); q2=q(3); q3=q(4);\n' ...
    'R_IB = [1-2*(q2^2+q3^2), 2*(q1*q2-q0*q3), 2*(q1*q3+q0*q2);\n' ...
    '        2*(q1*q2+q0*q3), 1-2*(q1^2+q3^2), 2*(q2*q3-q0*q1);\n' ...
    '        2*(q1*q3-q0*q2), 2*(q2*q3+q0*q1), 1-2*(q1^2+q2^2)];\n' ...
    'acc = (R_IB*[0;0;-sum(T)] + [0;0;m*g] - 0.18*vel) / m;\n' ...
    'if pos(3)>=0 && acc(3)>0 && sum(T)<m*g, vel(3)=0; acc(3)=0; pos(3)=0; end\n' ...
    'vel = vel + acc*dt; pos = pos + vel*dt;\n' ...
    'if pos(3)>0, pos(3)=0; vel(3)=0; end\n' ...
    'accel_out = R_IB'' * [0;0;g];\n' ...
    'gyro_out  = omega;\n' ...
    'roll  = atan2(2*(q0*q1+q2*q3), 1-2*(q1^2+q2^2));\n' ...
    'pitch = asin(max(min(2*(q0*q2-q3*q1),1),-1));\n' ...
    'yaw   = atan2(2*(q0*q3+q1*q2), 1-2*(q2^2+q3^2));\n' ...
    'euler_deg = [roll; pitch; yaw] * (180/pi);\n' ...
    'alt_m = -pos(3);\n' ...
]);

% Feedback Unit Delays (to break algebraic loop in discrete 1kHz simulation)
add_block('simulink/Discrete/Unit Delay', [model_name '/Accel_Delay'], 'InitialCondition', '[0; 0; 9.81]', 'SampleTime', '0.001', 'Position', [480, 400, 520, 430]);
add_block('simulink/Discrete/Unit Delay', [model_name '/Gyro_Delay'],  'InitialCondition', '[0; 0; 0]',    'SampleTime', '0.001', 'Position', [480, 460, 520, 490]);

% Outputs (To Workspace + Scope)
add_block('simulink/Sinks/To Workspace', [model_name '/Log_Euler_Deg'], 'VariableName', 'sim_euler_deg', 'SaveFormat', 'Timeseries', 'Position', [780, 215, 860, 245]);
add_block('simulink/Sinks/To Workspace', [model_name '/Log_Motors'],    'VariableName', 'sim_motors',    'SaveFormat', 'Timeseries', 'Position', [780, 60,  860, 90]);
add_block('simulink/Sinks/Scope',        [model_name '/Euler_Scope'],   'Position', [780, 280, 820, 320]);

%% 3. Wire Connections
add_line(model_name, 'Throttle_Hover/1', 'RC_Mux/1');
add_line(model_name, 'Roll_Step_Rad/1',  'RC_Mux/2');
add_line(model_name, 'Pitch_Step_Rad/1', 'RC_Mux/3');
add_line(model_name, 'Yaw_Rate_Cmd/1',   'RC_Mux/4');

add_line(model_name, 'RC_Mux/1',       'Flight_Controller_Block/1');
add_line(model_name, 'Accel_Delay/1',  'Flight_Controller_Block/2');
add_line(model_name, 'Gyro_Delay/1',   'Flight_Controller_Block/3');
add_line(model_name, 'Armed/1',        'Flight_Controller_Block/4');
add_line(model_name, 'Flight_Mode/1',  'Flight_Controller_Block/5');
add_line(model_name, 'dt/1',           'Flight_Controller_Block/6');
add_line(model_name, 'Vbat/1',         'Flight_Controller_Block/7');
add_line(model_name, 'Airmode/1',      'Flight_Controller_Block/8');

add_line(model_name, 'Flight_Controller_Block/1', 'Quad_6DOF_Plant/1');
add_line(model_name, 'Flight_Controller_Block/1', 'Log_Motors/1');

add_line(model_name, 'Quad_6DOF_Plant/1', 'Accel_Delay/1');
add_line(model_name, 'Quad_6DOF_Plant/2', 'Gyro_Delay/1');
add_line(model_name, 'Quad_6DOF_Plant/3', 'Log_Euler_Deg/1');
add_line(model_name, 'Quad_6DOF_Plant/3', 'Euler_Scope/1');

save_system(model_name, slx_path);
fprintf('[SUCCESS] Saved Simulink model to %s\n', slx_path);

%% 4. Execute Simulink Simulation to Verify
clear flight_controller attitude_estimator biquad_filter ai_neuro_compensator;
simOut = sim(model_name, 'ReturnWorkspaceOutputs', 'on');
ed = squeeze(simOut.sim_euler_deg.Data);
if size(ed, 1) == 3
    euler_final = ed(:, end);
else
    euler_final = ed(end, :)';
end
fprintf('[VERIFIED] Simulink 6.0s run complete. Final Euler (deg): Roll=%.2f, Pitch=%.2f, Yaw=%.2f\n', ...
    euler_final(1), euler_final(2), euler_final(3));

close_system(model_name, 0);
