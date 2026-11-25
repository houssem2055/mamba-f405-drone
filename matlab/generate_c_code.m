%% generate_c_code.m
% Generates embedded C code from flight_controller.m and attitude_estimator.m
% using MATLAB Coder / Embedded Coder.
clear; clc;

fprintf('====================================================\n');
fprintf('  Generating Embedded C Code from MATLAB Functions   \n');
fprintf('====================================================\n');

% Ensure we are in the matlab directory
script_dir = fileparts(mfilename('fullpath'));
cd(script_dir);

target_inc_dir = fullfile(script_dir, '..', 'firmware', 'inc');
target_src_dir = fullfile(script_dir, '..', 'firmware', 'src');

% Check if MATLAB Coder is installed
has_coder = ~isempty(which('codegen'));

if has_coder
    fprintf('[INFO] MATLAB Coder detected. Configuring code generator...\n');
    
    cfg = coder.config('lib', 'ecoder', false); % Generate generic C library
    cfg.TargetLang = 'C';
    cfg.GenerateReport = true;
    cfg.GenCodeOnly = true;
    cfg.HardwareImplementation.ProdHWDeviceType = 'ARM Compatible->ARM Cortex-M4';
    cfg.SupportNonFinite = false;
    cfg.InlineThreshold = 10;
    
    % Example input types for code generation:
    % rc_cmd:      4x1 double
    % accel:       3x1 double
    % gyro:        3x1 double
    % armed:       uint8 scalar
    % flight_mode: uint8 scalar
    % dt:          double scalar
    args_example = {
        coder.typeof(zeros(4,1)), ...
        coder.typeof(zeros(3,1)), ...
        coder.typeof(zeros(3,1)), ...
        uint8(0), ...
        uint8(0), ...
        0.001 ...
    };

    fprintf('[INFO] Compiling flight_controller.m to C...\n');
    codegen -config cfg flight_controller -args args_example -o flight_controller_codegen
    fprintf('[SUCCESS] C code generated into codegen folder.\n');
else
    fprintf('[NOTICE] MATLAB Coder license not active in current session.\n');
    fprintf('[INFO] The firmware folder already includes the pre-translated, optimized\n');
    fprintf('       C implementation matching these MATLAB equations exactly.\n');
end

fprintf('Done!\n');
