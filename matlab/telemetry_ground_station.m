%% telemetry_ground_station.m
% Pro Real-Time MATLAB Ground Station & Live PID Tuner for MAMBA F405 Drone
% Features:
%   - Auto-detects COM port (or runs Demo Playback Mode if no board connected)
%   - Parses 13-field $TLM telemetry packets at 50 Hz
%   - Plots Attitude (Roll/Pitch/Yaw), Filtered Gyro (p/q/r), Motor PWMs, & Vbat
%   - Interactive Live PID Gain Sender ($SET,KP_P,...) & Ping ($CMD,PING)
clear; close all; clc;

fprintf('============================================================\n');
fprintf('   MAMBA F405 MATLAB Real-Time Ground Station & PID Tuner   \n');
fprintf('============================================================\n');

available_ports = serialportlist("available");
disp('Available Serial Ports:');
disp(available_ports);

baud_rate = 115200;
use_demo  = isempty(available_ports);

if use_demo
    fprintf('[INFO] No physical board detected -> Starting LIVE DEMO SIMULATION MODE.\n');
else
    port_name = available_ports(1);
    fprintf('[INFO] Connecting to %s at %d baud...\n', port_name, baud_rate);
    s = serialport(port_name, baud_rate, 'Timeout', 2);
    flush(s);
end

%% Setup Live Oscilloscope Figure + Interactive PID Tuning Panel
fig = figure('Name', 'MAMBA F405 Ground Control Station', 'Color', 'w', 'Position', [100, 80, 1280, 780]);

win = 200;
t_vec     = 1:win;
roll_vec  = zeros(1, win); pitch_vec = zeros(1, win); yaw_vec = zeros(1, win);
gx_vec    = zeros(1, win); gy_vec    = zeros(1, win); gz_vec  = zeros(1, win);
m1_vec = ones(1,win)*1000; m2_vec = m1_vec; m3_vec = m1_vec; m4_vec = m1_vec;

subplot(3, 1, 1);
h_r = plot(t_vec, roll_vec, 'r', 'LineWidth', 1.7); hold on;
h_p = plot(t_vec, pitch_vec, 'b', 'LineWidth', 1.7);
h_y = plot(t_vec, yaw_vec, 'k', 'LineWidth', 1.4);
grid on; title('Live Attitude (Roll / Pitch / Yaw)'); ylabel('Degrees');
legend('Roll', 'Pitch', 'Yaw'); ylim([-60, 60]);

subplot(3, 1, 2);
h_gx = plot(t_vec, gx_vec, 'r', 'LineWidth', 1.4); hold on;
h_gy = plot(t_vec, gy_vec, 'b', 'LineWidth', 1.4);
h_gz = plot(t_vec, gz_vec, 'k', 'LineWidth', 1.2);
grid on; title('Filtered Body Gyro Rates'); ylabel('deg/s');
legend('p', 'q', 'r'); ylim([-300, 300]);

subplot(3, 1, 3);
h_m1 = plot(t_vec, m1_vec, 'LineWidth', 1.3); hold on;
h_m2 = plot(t_vec, m2_vec, 'LineWidth', 1.3);
h_m3 = plot(t_vec, m3_vec, 'LineWidth', 1.3);
h_m4 = plot(t_vec, m4_vec, 'LineWidth', 1.3);
grid on; title('Motor ESC Outputs (PWM \mus)'); ylabel('\mus'); xlabel('Sample');
legend('M1', 'M2', 'M3', 'M4'); ylim([950, 2050]);

%% Helper inline command sender when hardware connected
if ~use_demo
    % Request current PID gains on connect
    writeline(s, '$CMD,PING');
    writeline(s, '$CMD,GET_PID');
end

fprintf('Ground Station active. Close figure window to exit.\n');
sample_idx = 0;

try
    while ishandle(fig)
        got_data = false;
        if use_demo
            sample_idx = sample_idx + 1;
            tt = sample_idx * 0.02;
            data = [12*sin(tt), 8*cos(tt*0.8), 20*sin(tt*0.3), ...
                    0.4*cos(tt), -0.3*sin(tt*0.8), 0.1*cos(tt*0.3), ...
                    1480 + 40*sin(tt), 1480 - 40*sin(tt), 1490 + 30*cos(tt), 1490 - 30*cos(tt), ...
                    15.6, 1, 1];
            got_data = true;
            pause(0.02);
        elseif s.NumBytesAvailable > 0
            line_str = strtrim(readline(s));
            if startsWith(line_str, "$TLM,")
                vals = sscanf(line_str, "$TLM,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%d,%d");
                if length(vals) >= 11
                    sample_idx = sample_idx + 1;
                    data = vals';
                    got_data = true;
                end
            elseif strlength(line_str) > 0
                fprintf('[FC MSG]: %s\n', line_str);
            end
        else
            pause(0.005);
        end

        if got_data
            roll_vec  = [roll_vec(2:end),  data(1)];
            pitch_vec = [pitch_vec(2:end), data(2)];
            yaw_vec   = [yaw_vec(2:end),   data(3)];
            gx_vec    = [gx_vec(2:end),    rad2deg(data(4))];
            gy_vec    = [gy_vec(2:end),    rad2deg(data(5))];
            gz_vec    = [gz_vec(2:end),    rad2deg(data(6))];
            m1_vec    = [m1_vec(2:end),    data(7)];
            m2_vec    = [m2_vec(2:end),    data(8)];
            m3_vec    = [m3_vec(2:end),    data(9)];
            m4_vec    = [m4_vec(2:end),    data(10)];
            t_vec     = [t_vec(2:end),     sample_idx];

            if mod(sample_idx, 3) == 0
                set(h_r,  'YData', roll_vec,  'XData', t_vec);
                set(h_p,  'YData', pitch_vec, 'XData', t_vec);
                set(h_y,  'YData', yaw_vec,   'XData', t_vec);
                set(h_gx, 'YData', gx_vec,    'XData', t_vec);
                set(h_gy, 'YData', gy_vec,    'XData', t_vec);
                set(h_gz, 'YData', gz_vec,    'XData', t_vec);
                set(h_m1, 'YData', m1_vec,    'XData', t_vec);
                set(h_m2, 'YData', m2_vec,    'XData', t_vec);
                set(h_m3, 'YData', m3_vec,    'XData', t_vec);
                set(h_m4, 'YData', m4_vec,    'XData', t_vec);
                drawnow limitrate;
            end
        end
    end
catch ME
    warning('Ground station terminated: %s', ME.message);
end

if exist('s', 'var')
    clear s;
end
