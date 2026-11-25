%% telemetry_ground_station.m
% Real-Time MATLAB Ground Station for Mamba F405 Drone
% Connects over USB / UART (COM Port), receives live IMU & Motor telemetry,
% and displays live real-time scopes.
clear; close all; clc;

fprintf('====================================================\n');
fprintf('     Mamba F405 MATLAB Real-Time Ground Station     \n');
fprintf('====================================================\n');

%% 1. Serial Port Configuration
% List available serial ports
available_ports = serialportlist("available");
disp('Available Serial Ports:');
disp(available_ports);

if isempty(available_ports)
    warning('No COM port detected. Connect the Mamba F405 via USB cable.');
    port_name = 'COM3'; % Default placeholder
else
    port_name = available_ports(1);
    fprintf('Selecting detected port: %s\n', port_name);
end

baud_rate = 115200; % Standard telemetry baud rate

%% 2. Setup Real-time Figure
fig = figure('Name', 'Mamba F405 Live Telemetry', 'Color', 'w', 'Position', [150, 150, 1100, 700]);

window_size = 200; % Show last 200 samples
time_vec    = zeros(1, window_size);
roll_vec    = zeros(1, window_size);
pitch_vec   = zeros(1, window_size);
yaw_vec     = zeros(1, window_size);
m1_vec      = zeros(1, window_size);
m2_vec      = zeros(1, window_size);
m3_vec      = zeros(1, window_size);
m4_vec      = zeros(1, window_size);

subplot(2, 1, 1);
h_roll  = plot(time_vec, roll_vec, 'r', 'LineWidth', 1.8); hold on;
h_pitch = plot(time_vec, pitch_vec, 'b', 'LineWidth', 1.8);
h_yaw   = plot(time_vec, yaw_vec, 'k', 'LineWidth', 1.5);
grid on;
title('Live Drone Attitude Angles (Roll / Pitch / Yaw)');
xlabel('Sample'); ylabel('Degrees');
legend('Roll (\phi)', 'Pitch (\theta)', 'Yaw (\psi)');
ylim([-60, 60]);

subplot(2, 1, 2);
h_m1 = plot(time_vec, m1_vec, 'LineWidth', 1.4); hold on;
h_m2 = plot(time_vec, m2_vec, 'LineWidth', 1.4);
h_m3 = plot(time_vec, m3_vec, 'LineWidth', 1.4);
h_m4 = plot(time_vec, m4_vec, 'LineWidth', 1.4);
grid on;
title('Live Motor PWM Output Signals (\mus)');
xlabel('Sample'); ylabel('Pulse Width (\mus)');
legend('M1', 'M2', 'M3', 'M4');
ylim([950, 2050]);

%% 3. Connect to Drone & Stream Data
try
    s = serialport(port_name, baud_rate, 'Timeout', 2);
    flush(s);
    fprintf('Connected to %s at %d baud.\n', port_name, baud_rate);
    fprintf('Receiving live telemetry... Press Ctrl+C in Command Window to stop.\n');
    
    sample_idx = 0;
    while ishandle(fig)
        % Read a telemetry line (format: $TLM,roll,pitch,yaw,m1,m2,m3,m4,vbat\n)
        if s.NumBytesAvailable > 0
            line_str = readline(s);
            data = sscanf(line_str, "$TLM,%f,%f,%f,%f,%f,%f,%f");
            
            if length(data) >= 7
                sample_idx = sample_idx + 1;
                
                % Shift vectors
                roll_vec  = [roll_vec(2:end), data(1)];
                pitch_vec = [pitch_vec(2:end), data(2)];
                yaw_vec   = [yaw_vec(2:end), data(3)];
                m1_vec    = [m1_vec(2:end), data(4)];
                m2_vec    = [m2_vec(2:end), data(5)];
                m3_vec    = [m3_vec(2:end), data(6)];
                m4_vec    = [m4_vec(2:end), data(7)];
                time_vec  = [time_vec(2:end), sample_idx];
                
                % Update plots every 5 frames for smooth rendering
                if mod(sample_idx, 5) == 0
                    set(h_roll,  'YData', roll_vec,  'XData', time_vec);
                    set(h_pitch, 'YData', pitch_vec, 'XData', time_vec);
                    set(h_yaw,   'YData', yaw_vec,   'XData', time_vec);
                    set(h_m1,    'YData', m1_vec,    'XData', time_vec);
                    set(h_m2,    'YData', m2_vec,    'XData', time_vec);
                    set(h_m3,    'YData', m3_vec,    'XData', time_vec);
                    set(h_m4,    'YData', m4_vec,    'XData', time_vec);
                    drawnow limitrate;
                end
            end
        else
            pause(0.005);
        end
    end
    clear s;
catch ME
    warning('Serial connection error or aborted: %s', ME.message);
    if exist('s', 'var')
        clear s;
    end
end
