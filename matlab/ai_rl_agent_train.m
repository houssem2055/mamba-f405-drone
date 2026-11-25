%% ai_rl_agent_train.m
% Trains a Deep Deterministic Policy Gradient (DDPG) Reinforcement Learning Agent
% using MATLAB Reinforcement Learning Toolbox on the nonlinear Quadcopter Roll
% stabilization + wind gust disturbance environment, and benchmarks:
%   1. Baseline Cascaded PID (AI OFF)
%   2. Embedded TinyMLP + Online Lyapunov Adaptation (AI ON)
%   3. Trained RL DDPG Actor Policy
clear; close all; clc;

fprintf('============================================================\n');
fprintf('  Training Reinforcement Learning (DDPG) Drone Policy Agent \n');
fprintf('============================================================\n');

script_dir = fileparts(mfilename('fullpath'));
cd(script_dir);
rng(2026);

init_drone_parameters;

%% 1. Define RL Environment (Observations: [roll_err, roll_rate, target_roll, integral_err])
obsInfo = rlNumericSpec([4 1], 'Name', 'DroneStates');
actInfo = rlNumericSpec([1 1], 'LowerLimit', -1.0, 'UpperLimit', 1.0, 'Name', 'RollTorquePWM');

env = rlFunctionEnv(obsInfo, actInfo, @quad_rl_step, @quad_rl_reset);

%% 2. Create & Configure DDPG Actor-Critic Agent
agentOpts = rlDDPGAgentOptions( ...
    'SampleTime', 0.01, ...
    'DiscountFactor', 0.98, ...
    'MiniBatchSize', 64, ...
    'ExperienceBufferLength', 20000);
agentOpts.NoiseOptions.StandardDeviation = 0.18;
agentOpts.NoiseOptions.StandardDeviationDecayRate = 1e-4;
agentOpts.ActorOptimizerOptions.LearnRate = 5e-4;
agentOpts.CriticOptimizerOptions.LearnRate = 1e-3;

agent = rlDDPGAgent(obsInfo, actInfo, agentOpts);

trainOpts = rlTrainingOptions( ...
    'MaxEpisodes', 60, ...
    'MaxStepsPerEpisode', 200, ...
    'ScoreAveragingWindowLength', 10, ...
    'Verbose', false, ...
    'Plots', 'none', ...
    'StopTrainingCriteria', 'AverageReward', ...
    'StopTrainingValue', -1.0);

fprintf('[RL] Training DDPG Agent for up to 60 episodes (100 Hz control)...\n');
trainingStats = train(agent, env, trainOpts);
save(fullfile(script_dir, 'rl_agent.mat'), 'agent', 'trainingStats');
fprintf('[RL] Training Complete! Final Episode Reward = %.2f\n', trainingStats.EpisodeReward(end));

%% 3. Benchmark: Pure PID vs AI Neuro-Adaptive PID under Heavy Crosswind + Motor Asymmetry
fprintf('\n[BENCHMARK] Comparing Pure PID vs Embedded AI (TinyMLP + Online Adapt)...\n');
dt = 0.001; N = 4000; t_vec = (0:N-1) * dt;
omega_hover = sqrt((drone.m * drone.g) / (4.0 * drone.kt));
pwm_hover   = drone.pwm_min + ((omega_hover - drone.omega_idle) / (drone.omega_max - drone.omega_idle)) * (drone.pwm_max - drone.pwm_min);
hover_thr   = (pwm_hover - drone.pwm_idle) / (drone.pwm_max - drone.pwm_idle);

[roll_pid, u_pid] = run_disturbance_test(drone, hover_thr, N, dt, uint8(0));
[roll_ai,  u_ai]  = run_disturbance_test(drone, hover_thr, N, dt, uint8(1));

% Compute tracking error during wind gust + motor asymmetry window (t = 1.0 .. 4.0 s)
target_deg = zeros(1, N);
target_deg(t_vec >= 2.0) = 15.0;
rmse_pid = sqrt(mean((roll_pid(t_vec >= 1.0) - target_deg(t_vec >= 1.0)).^2));
rmse_ai  = sqrt(mean((roll_ai(t_vec  >= 1.0) - target_deg(t_vec >= 1.0)).^2));
fprintf('Tracking RMSE under Sustained Wind + 12%% Motor Fault:\n');
fprintf('  Pure PID:              %.3f deg\n', rmse_pid);
fprintf('  AI Neuro-Adaptive PID: %.3f deg\n', rmse_ai);

%% 4. Export RL Training Plot & AI Comparison Plot
out_dir = fullfile(script_dir, '..', 'docs');

fig1 = figure('Name', 'RL DDPG Training Progress', 'Color', 'w', 'Position', [100, 100, 1000, 480], 'Visible', 'off');
subplot(1, 2, 1);
plot(trainingStats.EpisodeIndex, trainingStats.EpisodeReward, 'Color', [0.6 0.75 0.95], 'LineWidth', 1.0); hold on;
plot(trainingStats.EpisodeIndex, trainingStats.AverageReward, 'b-', 'LineWidth', 2.0);
grid on; xlabel('Episode'); ylabel('Cumulative Reward');
title('RL DDPG Agent Training Curve');
legend('Episode Reward', 'Moving Avg (10 ep)', 'Location', 'best');

subplot(1, 2, 2);
% Rollout trained DDPG agent on 2s evaluation episode (0 deg -> +15 deg step + wind gust)
[~, st] = quad_rl_reset();
st = [0.0; 0.0; 0.2618; 0.0; 1.0];
obs = [0.2618; 0.0; 0.2618; 0.0];
rl_roll = zeros(1, 200);
rl_ref  = zeros(1, 200);
for k = 1:200
    act = getAction(agent, obs);
    if iscell(act), act = act{1}; end
    [obs, ~, ~, st] = quad_rl_step(act, st);
    rl_roll(k) = rad2deg(st(1));
    rl_ref(k)  = rad2deg(st(3));
end
plot((0:199)*0.01, rl_ref, 'k--', 'LineWidth', 1.4); hold on;
plot((0:199)*0.01, rl_roll, 'r-', 'LineWidth', 1.8);
grid on; xlabel('Time (s)'); ylabel('Roll Angle (deg)');
title('Trained DDPG Policy Rollout (Step + Gust)');
legend('Target', 'DDPG RL Agent', 'Location', 'best');

exportgraphics(fig1, fullfile(out_dir, 'ai_rl_training.png'), 'Resolution', 150);
close(fig1);

fig2 = figure('Name', 'AI Neuro-Adaptive PID vs Pure PID', 'Color', 'w', 'Position', [100, 100, 1100, 540], 'Visible', 'off');
subplot(2, 1, 1);
plot(t_vec, target_deg, 'k--', 'LineWidth', 1.4); hold on;
plot(t_vec, roll_pid, 'b-', 'LineWidth', 1.6);
plot(t_vec, roll_ai,  'r-', 'LineWidth', 1.8);
xline(1.0, '--m', 'Sustained Gust + 12% Motor Fault');
grid on; ylabel('Roll Angle (deg)');
title(sprintf('Disturbance Rejection: Pure PID (RMSE=%.2f°) vs AI Neuro-Adaptive PID (RMSE=%.2f°)', rmse_pid, rmse_ai));
legend('Command', 'Pure PID (AI OFF)', 'TinyMLP + Online Adapt (AI ON)', 'Location', 'best');

subplot(2, 1, 2);
plot(t_vec, u_pid, 'b-', 'LineWidth', 1.3); hold on;
plot(t_vec, u_ai,  'r-', 'LineWidth', 1.3);
grid on; xlabel('Time (s)'); ylabel('M3 - M1 Differential (\mus)');
title('Control Effort Comparison');
legend('Pure PID', 'AI Neuro-Adaptive PID', 'Location', 'best');

exportgraphics(fig2, fullfile(out_dir, 'ai_comparison_results.png'), 'Resolution', 150);
close(fig2);

fprintf('[PLOTTED] Saved docs/ai_rl_training.png & docs/ai_comparison_results.png\n');

%% ========================================================================
%  HELPER FUNCTIONS FOR RL ENVIRONMENT & BENCHMARK
%% ========================================================================
function [initObs, loggedSignals] = quad_rl_reset()
    roll0 = (rand()*2 - 1) * 0.25;
    target = 0.2618; % +15 deg step target
    loggedSignals = [roll0; 0.0; target; 0.0; 1.0]; % [roll, rate, target, int_err, step]
    initObs = [target - roll0; 0.0; target; 0.0];
end

function [nextObs, reward, isDone, loggedSignals] = quad_rl_step(action, loggedSignals)
    dt = 0.01;
    Ixx = 4.85e-3;
    G_tau_pwm = 4 * (0.125/sqrt(2)) * (2 * 1.15e-6 * 1177 * 2.0);
    roll   = loggedSignals(1);
    rate   = loggedSignals(2);
    target = loggedSignals(3);
    int_e  = loggedSignals(4);
    step_k = loggedSignals(5);

    % Residual RL: Base PD stabilizer + DDPG RL residual torque [-120..+120] us
    err_ang = target - roll;
    rate_des = 6.0 * err_ang;
    pwm_base = 104.0 * (rate_des - rate) + 1.0 * (-rate);

    act_val = max(min(double(action(1)), 1.0), -1.0);
    pwm_diff = pwm_base + act_val * 120.0;

    tau_gust = 0.0;
    if step_k >= 60 && step_k <= 140
        tau_gust = 0.045; % Strong wind disturbance
    end

    tau = G_tau_pwm * pwm_diff + tau_gust - 0.0025 * rate;
    rate = rate + (tau / Ixx) * dt;
    roll = roll + rate * dt;

    err = target - roll;
    int_e = max(min(int_e + err * dt, 0.5), -0.5);
    step_k = step_k + 1;

    nextObs = [err; rate; target; int_e];
    reward  = -(20.0 * err^2 + 0.5 * rate^2 + 0.02 * act_val^2) + 0.1 * (abs(err) < 0.02);
    isDone  = (abs(roll) > 1.3) || (step_k > 200);
    loggedSignals = [roll; rate; target; int_e; step_k];
end

function [roll_hist, diff_pwm] = run_disturbance_test(drone, hover_thr, N, dt, ai_on)
    clear flight_controller attitude_estimator biquad_filter ai_neuro_compensator;
    q = [1;0;0;0]; omega = [0;0;0]; rw = ones(4,1)*drone.omega_idle;
    roll_hist = zeros(1, N);
    diff_pwm  = zeros(1, N);
    for k = 1:N
        t = (k - 1) * dt;
        rc_roll = 0.0;
        if t >= 2.0, rc_roll = deg2rad(15.0); end
        q0=q(1); q1=q(2); q2=q(3); q3=q(4);
        r31 = 2*(q1*q3 - q0*q2); r32 = 2*(q2*q3 + q0*q1); r33 = 1 - 2*(q1^2 + q2^2);
        accel = [r31; r32; r33] * drone.g;
        [mpwm, ~] = flight_controller([hover_thr + 0.02; rc_roll; 0; 0], accel, omega, uint8(1), uint8(1), dt, 15.5, uint8(1), ai_on);
        diff_pwm(k) = mpwm(3) - mpwm(1);

        w_cmd = drone.omega_idle + ((mpwm - 1000.0)/1000.0) * 2000.0;
        % Inject 12% Motor 1 thrust degradation + sustained crosswind after t = 1.0 s
        if t >= 1.0
            w_cmd(1) = w_cmd(1) * 0.94; % ~12% thrust loss
        end
        rw = rw + (dt / drone.motor_tau) * (w_cmd - rw);
        T = drone.kt * (rw.^2); Q = drone.km * (rw.^2);
        tau_gust = 0.0;
        if t >= 1.0, tau_gust = -0.045; end
        tau = [drone.d*(-T(1)-T(2)+T(3)+T(4)) + tau_gust; drone.d*(-T(1)+T(2)-T(3)+T(4)); -Q(1)+Q(2)+Q(3)-Q(4)] - drone.drag_rot*omega;
        omega = omega + (drone.J \ (tau - cross(omega, drone.J * omega))) * dt;
        p = omega(1); qr = omega(2); r = omega(3);
        q = q + 0.5*[-q1*p-q2*qr-q3*r; q0*p+q2*r-q3*qr; q0*qr-q1*r+q3*p; q0*r+q1*qr-q2*p]*dt;
        q = q / norm(q);
        roll_hist(k) = rad2deg(atan2(2*(q(1)*q(2) + q(3)*q(4)), 1 - 2*(q(2)^2 + q(3)^2)));
    end
end
