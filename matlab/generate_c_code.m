%% generate_c_code.m
% Generates embedded C code from flight_controller.m (including Mahony AHRS
% and Biquad LPF/Notch filters) using MATLAB Embedded Coder / MATLAB Coder.
clear; clc;

fprintf('============================================================\n');
fprintf('  Generating Embedded C Code from Upgraded MATLAB Stack     \n');
fprintf('============================================================\n');

script_dir = fileparts(mfilename('fullpath'));
cd(script_dir);

has_coder = ~isempty(which('codegen'));

if has_coder
    fprintf('[INFO] MATLAB Coder detected. Configuring ARM Cortex-M4 C library...\n');

    cfg = coder.config('lib', 'ecoder', license('test', 'RTW_Embedded_Coder') == 1);
    cfg.TargetLang = 'C';
    cfg.GenerateReport = true;
    cfg.GenCodeOnly = true;
    cfg.HardwareImplementation.ProdHWDeviceType = 'ARM Compatible->ARM Cortex';
    cfg.SupportNonFinite = false;
    cfg.InlineThreshold = 10;

    % Fixed-size input specification for flight_controller(rc_cmd, accel, gyro, armed, flight_mode, dt, vbat, airmode)
    args_example = {
        coder.typeof(zeros(4, 1)), ...
        coder.typeof(zeros(3, 1)), ...
        coder.typeof(zeros(3, 1)), ...
        uint8(0), ...
        uint8(1), ...
        0.001, ...
        16.0, ...
        uint8(1) ...
    };

    fprintf('[INFO] Generating C code for flight_controller...\n');
    codegen -config cfg flight_controller -args args_example -o flight_controller_codegen
    fprintf('[SUCCESS] Generated C code into matlab/codegen/lib/flight_controller/\n');
else
    fprintf('[NOTICE] MATLAB Coder license not active in current session.\n');
    fprintf('[INFO] Hand-optimized C matching these algorithms is in firmware/src/\n');
end

fprintf('Done!\n');
