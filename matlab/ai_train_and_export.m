%% ai_train_and_export.m
% Trains Two Embedded TinyML Neural Networks using Deep Learning Toolbox:
%   1. Neural Residual Disturbance Compensator (6 -> 12 -> 8 -> 3 TinyMLP)
%   2. Real-Time 4-Class Flight Anomaly & Fault Classifier (5 -> 10 -> 4 Softmax)
% Evaluates accuracy, generates training plots (docs/ai_training_summary.png),
% and exports static float weights to:
%   - matlab/ai_models.mat
%   - firmware/inc/ai_weights.h
clear; close all; clc;

fprintf('============================================================\n');
fprintf('  Training Embedded AI Models (TinyMLP + Fault Classifier)  \n');
fprintf('============================================================\n');

script_dir = fileparts(mfilename('fullpath'));
cd(script_dir);
rng(2026);

%% ========================================================================
%  PART 1: NEURAL RESIDUAL DISTURBANCE COMPENSATOR (6 -> 12 -> 8 -> 3)
%  Inputs:  [roll_err, pitch_err, p_err, q_err, p_meas, q_meas]
%  Outputs: [du_roll, du_pitch, du_yaw] (normalized residual control [-0.18..0.18])
%% ========================================================================
N_samples = 8000;
X_comp = zeros(6, N_samples);
Y_comp = zeros(3, N_samples);

% Generate physics-grounded training samples across disturbances, CG offset & cross-coupling
Ixx = 4.85e-3; Iyy = 4.85e-3; Izz = 8.80e-3;
d   = 0.125 / sqrt(2);
kt  = 1.15e-6;
w_hov = sqrt((0.650 * 9.81) / (4 * kt));
G_tau_pwm = 4 * d * (2 * kt * w_hov * 2.0); % ~0.00191 N*m / us

for i = 1:N_samples
    roll_err  = (rand()*2 - 1) * 0.45;  % rad
    pitch_err = (rand()*2 - 1) * 0.45;
    p_err     = (rand()*2 - 1) * 2.50;  % rad/s
    q_err     = (rand()*2 - 1) * 2.50;
    p_meas    = (rand()*2 - 1) * 3.50;
    q_meas    = (rand()*2 - 1) * 3.50;

    % Nonlinear gyroscopic cross-coupling & aerodynamic drag disturbance torque
    r_est = 0.3 * (p_meas - q_meas);
    tau_cross_x = (Izz - Iyy) * q_meas * r_est + 0.0025 * p_meas;
    tau_cross_y = (Ixx - Izz) * p_meas * r_est + 0.0025 * q_meas;

    % Optimal residual compensation (normalized units, scaled by 800 in controller)
    % Combines nonlinear rate-error boost (tanh soft gain) + inverse cross-coupling
    du_r = (0.045 * tanh(2.0 * p_err) + 0.020 * tanh(3.0 * roll_err)  + tau_cross_x / (G_tau_pwm * 800.0));
    du_p = (0.045 * tanh(2.0 * q_err) + 0.020 * tanh(3.0 * pitch_err) + tau_cross_y / (G_tau_pwm * 800.0));
    du_y = (0.015 * tanh(1.5 * (p_err - q_err)));

    du_r = max(min(du_r, 0.18), -0.18);
    du_p = max(min(du_p, 0.18), -0.18);
    du_y = max(min(du_y, 0.12), -0.12);

    X_comp(:, i) = [roll_err; pitch_err; p_err; q_err; p_meas; q_meas];
    Y_comp(:, i) = [du_r; du_p; du_y];
end

comp_mean    = mean(X_comp, 2);
comp_std     = max(std(X_comp, 0, 2), 1e-4);
comp_inv_std = 1.0 ./ comp_std;
X_comp_norm  = (X_comp - comp_mean) .* comp_inv_std;

% Build and train 6 -> 12 -> 8 -> 3 network via Deep Learning Toolbox (trainlm)
net_comp = feedforwardnet([12, 8], 'trainlm');
net_comp.inputs{1}.processFcns  = {};
net_comp.outputs{3}.processFcns = {};
net_comp.layers{1}.transferFcn  = 'tansig';
net_comp.layers{2}.transferFcn  = 'tansig';
net_comp.layers{3}.transferFcn  = 'purelin';
net_comp.trainParam.epochs      = 60;
net_comp.trainParam.showWindow  = false;
net_comp.divideParam.trainRatio = 0.8;
net_comp.divideParam.valRatio   = 0.1;
net_comp.divideParam.testRatio  = 0.1;

[net_comp, tr_comp] = train(net_comp, X_comp_norm, Y_comp);
Y_comp_pred = net_comp(X_comp_norm(:, tr_comp.testInd));
Y_comp_true = Y_comp(:, tr_comp.testInd);
comp_rmse   = sqrt(mean((Y_comp_pred(:) - Y_comp_true(:)).^2));
fprintf('[PART 1] TinyMLP Compensator trained! Test RMSE = %.6f (norm units)\n', comp_rmse);

% Extract exact weights: W1 [12x6], b1 [12x1], W2 [8x12], b2 [8x1], W3 [3x8], b3 [3x1]
comp_W1 = net_comp.IW{1,1}; comp_b1 = net_comp.b{1};
comp_W2 = net_comp.LW{2,1}; comp_b2 = net_comp.b{2};
comp_W3 = net_comp.LW{3,2}; comp_b3 = net_comp.b{3};

%% ========================================================================
%  PART 2: 4-CLASS REAL-TIME ANOMALY & FAULT CLASSIFIER (5 -> 10 -> 4)
%  Features: [gyro_jitter_rms, accel_dev, motor_asym, rate_err_rms, peak_g]
%  Classes:  0=NORMAL, 1=PROP_VIBRATION, 2=MOTOR_LOSS, 3=COLLISION
%% ========================================================================
N_per_class = 1000;
X_fault = zeros(5, N_per_class * 4);
Y_fault = zeros(4, N_per_class * 4);

idx = 1;
for c = 0:3
    for k = 1:N_per_class
        if c == 0 % NORMAL flight
            f = [0.005 + 0.035*rand(); 0.05 + 0.65*rand(); 2.0 + 45.0*rand(); 0.01 + 0.22*rand(); 0.95 + 0.30*rand()];
        elseif c == 1 % PROP_VIBRATION (bent/cracked blade -> massive gyro jitter)
            f = [0.14 + 0.60*rand(); 0.40 + 2.20*rand(); 15.0 + 95.0*rand(); 0.05 + 0.40*rand(); 1.05 + 0.60*rand()];
        elseif c == 2 % MOTOR_LOSS (ESC desync / dead motor -> massive mixer asymmetry + rate error)
            f = [0.03 + 0.15*rand(); 1.0 + 4.0*rand(); 180.0 + 420.0*rand(); 0.80 + 3.00*rand(); 0.60 + 1.00*rand()];
        else % COLLISION / CRASH (extreme accel shock & peak G)
            f = [0.08 + 0.45*rand(); 5.5 + 18.0*rand(); 40.0 + 200.0*rand(); 0.50 + 2.50*rand(); 2.30 + 3.80*rand()];
        end
        X_fault(:, idx) = f;
        Y_fault(c + 1, idx) = 1.0;
        idx = idx + 1;
    end
end

fault_mean    = mean(X_fault, 2);
fault_std     = max(std(X_fault, 0, 2), 1e-4);
fault_inv_std = 1.0 ./ fault_std;
X_fault_norm  = (X_fault - fault_mean) .* fault_inv_std;

net_fault = patternnet(10, 'trainscg');
net_fault.inputs{1}.processFcns  = {};
net_fault.outputs{2}.processFcns = {};
net_fault.layers{1}.transferFcn  = 'tansig';
net_fault.layers{2}.transferFcn  = 'softmax';
net_fault.trainParam.epochs      = 150;
net_fault.trainParam.showWindow  = false;
net_fault.divideParam.trainRatio = 0.75;
net_fault.divideParam.valRatio   = 0.10;
net_fault.divideParam.testRatio  = 0.15;

[net_fault, tr_fault] = train(net_fault, X_fault_norm, Y_fault);
Y_fault_pred = net_fault(X_fault_norm(:, tr_fault.testInd));
[~, cls_pred] = max(Y_fault_pred, [], 1);
[~, cls_true] = max(Y_fault(:, tr_fault.testInd), [], 1);
fault_acc = mean(cls_pred == cls_true) * 100.0;
fprintf('[PART 2] TinyFaultClassifier trained! Test Accuracy = %.2f%%\n', fault_acc);

fault_W1 = net_fault.IW{1,1}; fault_b1 = net_fault.b{1};
fault_W2 = net_fault.LW{2,1}; fault_b2 = net_fault.b{2};

%% ========================================================================
%  PART 3: SAVE MAT WEIGHTS & EXPORT C HEADER (firmware/inc/ai_weights.h)
%% ========================================================================
save(fullfile(script_dir, 'ai_models.mat'), ...
    'comp_mean', 'comp_inv_std', 'comp_W1', 'comp_b1', 'comp_W2', 'comp_b2', 'comp_W3', 'comp_b3', ...
    'fault_mean', 'fault_inv_std', 'fault_W1', 'fault_b1', 'fault_W2', 'fault_b2', ...
    'comp_rmse', 'fault_acc');

hdr_path = fullfile(script_dir, '..', 'firmware', 'inc', 'ai_weights.h');
fid = fopen(hdr_path, 'w');
fprintf(fid, '#ifndef AI_WEIGHTS_H\n#define AI_WEIGHTS_H\n\n');
fprintf(fid, '/**\n * AUTO-GENERATED BY matlab/ai_train_and_export.m\n');
fprintf(fid, ' * Do not edit manually. Retrain in MATLAB to update TinyML weights.\n */\n\n');

fprintf(fid, '/* 1. Neural Residual Compensator (6 -> 12 -> 8 -> 3) */\n');
fprintf(fid, '#define AI_COMP_IN_DIM   6U\n');
fprintf(fid, '#define AI_COMP_H1_DIM   12U\n');
fprintf(fid, '#define AI_COMP_H2_DIM   8U\n');
fprintf(fid, '#define AI_COMP_OUT_DIM  3U\n\n');

write_c_array(fid, 'AI_COMP_IN_MEAN',    comp_mean(:));
write_c_array(fid, 'AI_COMP_IN_INV_STD', comp_inv_std(:));
write_c_array(fid, 'AI_COMP_W1', reshape(comp_W1', [], 1));
write_c_array(fid, 'AI_COMP_B1', comp_b1(:));
write_c_array(fid, 'AI_COMP_W2', reshape(comp_W2', [], 1));
write_c_array(fid, 'AI_COMP_B2', comp_b2(:));
write_c_array(fid, 'AI_COMP_W3', reshape(comp_W3', [], 1));
write_c_array(fid, 'AI_COMP_B3', comp_b3(:));

fprintf(fid, '\n/* 2. Real-Time Fault Classifier (5 -> 10 -> 4) */\n');
fprintf(fid, '#define AI_FAULT_IN_DIM  5U\n');
fprintf(fid, '#define AI_FAULT_H1_DIM  10U\n');
fprintf(fid, '#define AI_FAULT_OUT_DIM 4U\n\n');

write_c_array(fid, 'AI_FAULT_IN_MEAN',    fault_mean(:));
write_c_array(fid, 'AI_FAULT_IN_INV_STD', fault_inv_std(:));
write_c_array(fid, 'AI_FAULT_W1', reshape(fault_W1', [], 1));
write_c_array(fid, 'AI_FAULT_B1', fault_b1(:));
write_c_array(fid, 'AI_FAULT_W2', reshape(fault_W2', [], 1));
write_c_array(fid, 'AI_FAULT_B2', fault_b2(:));

fprintf(fid, '\n#endif /* AI_WEIGHTS_H */\n');
fclose(fid);
fprintf('[EXPORTED] Saved weights to firmware/inc/ai_weights.h\n');

%% Export matching static MATLAB function (#codegen compatible) -> matlab/ai_weights_data.m
m_path = fullfile(script_dir, 'ai_weights_data.m');
mfid = fopen(m_path, 'w');
fprintf(mfid, 'function [in_mean, in_inv_std, W1, b1, W2, b2, W3, b3] = ai_weights_data()\n');
fprintf(mfid, '%%#codegen\n');
fprintf(mfid, '%% AUTO-GENERATED BY ai_train_and_export.m — matches firmware/inc/ai_weights.h\n\n');
write_m_matrix(mfid, 'in_mean',    comp_mean);
write_m_matrix(mfid, 'in_inv_std', comp_inv_std);
write_m_matrix(mfid, 'W1', comp_W1);
write_m_matrix(mfid, 'b1', comp_b1);
write_m_matrix(mfid, 'W2', comp_W2);
write_m_matrix(mfid, 'b2', comp_b2);
write_m_matrix(mfid, 'W3', comp_W3);
write_m_matrix(mfid, 'b3', comp_b3);
fprintf(mfid, 'end\n');
fclose(mfid);
fprintf('[EXPORTED] Saved static weights function to matlab/ai_weights_data.m\n');

%% ========================================================================
%  PART 4: PLOT TRAINING & EVALUATION DIAGNOSTICS
%% ========================================================================
out_dir = fullfile(script_dir, '..', 'docs');
fig = figure('Name', 'AI TinyML Training Summary', 'Color', 'w', 'Position', [100, 100, 1280, 560], 'Visible', 'off');

subplot(1, 3, 1);
semilogy(tr_comp.epoch, tr_comp.perf, 'b-', 'LineWidth', 1.8); hold on;
semilogy(tr_comp.epoch, tr_comp.vperf, 'r--', 'LineWidth', 1.5);
grid on; xlabel('Epoch'); ylabel('MSE Loss');
title(sprintf('TinyMLP Training (RMSE=%.5f)', comp_rmse));
legend('Train MSE', 'Val MSE', 'Location', 'best');

subplot(1, 3, 2);
scatter(Y_comp_true(1, 1:400)*800, Y_comp_pred(1, 1:400)*800, 14, 'b', 'filled', 'MarkerFaceAlpha', 0.5); hold on;
lims = [-150, 150];
plot(lims, lims, 'r--', 'LineWidth', 1.6);
grid on; xlim(lims); ylim(lims);
xlabel('Target Compensation (\mus)'); ylabel('TinyMLP Prediction (\mus)');
title('Roll Residual Compensation Fit');

subplot(1, 3, 3);
cm = confusionmat(cls_true, cls_pred);
imagesc(cm); colormap('parula'); colorbar;
set(gca, 'XTick', 1:4, 'XTickLabel', {'Normal', 'PropVib', 'MotorLoss', 'Crash'});
set(gca, 'YTick', 1:4, 'YTickLabel', {'Normal', 'PropVib', 'MotorLoss', 'Crash'});
for r = 1:4
    for col = 1:4
        text(col, r, num2str(cm(r, col)), 'HorizontalAlignment', 'center', 'Color', 'w', 'FontWeight', 'bold');
    end
end
xlabel('Predicted Class'); ylabel('True Class');
title(sprintf('TinyML Fault Classifier (Acc=%.1f%%)', fault_acc));

exportgraphics(fig, fullfile(out_dir, 'ai_training_summary.png'), 'Resolution', 150);
close(fig);
fprintf('[PLOTTED] Saved docs/ai_training_summary.png\n');

function write_c_array(fid, name, vec)
    fprintf(fid, 'static const float %s[%d] = {\n    ', name, length(vec));
    for i = 1:length(vec)
        fprintf(fid, '%.7ef', vec(i));
        if i < length(vec)
            fprintf(fid, ', ');
        end
        if mod(i, 6) == 0 && i < length(vec)
            fprintf(fid, '\n    ');
        end
    end
    fprintf(fid, '\n};\n');
end

function write_m_matrix(fid, name, mat)
    [r, c] = size(mat);
    fprintf(fid, '    %s = [', name);
    for i = 1:r
        for j = 1:c
            fprintf(fid, '%.7e', mat(i, j));
            if j < c, fprintf(fid, ', '); end
        end
        if i < r, fprintf(fid, '; ...\n        '); end
    end
    fprintf(fid, '];\n');
end
