function [du, w_out_mag] = ai_neuro_compensator(nn_in, rate_err, adapt_active, reset_weights, dt)
%#codegen
% AI_NEURO_COMPENSATOR
% Embedded TinyMLP (6 -> 12 -> 8 -> 3) with Online Lyapunov-Safe Output-Layer
% Weight Adaptation (sigma-modification). Bit-accurate match with
% firmware/src/tiny_nn.c + firmware/inc/ai_weights.h.

    persistent w_out_live initialized;
    [in_mean, in_inv_std, W1, b1, W2, b2, W3_base, b3] = ai_weights_data();

    if isempty(initialized) || reset_weights
        w_out_live = W3_base;
        initialized = true;
    end

    % 1. Input normalization & clamping
    x_norm = max(min((nn_in - in_mean) .* in_inv_std, 6.0), -6.0);

    % 2. Forward Pass (tanh hidden, linear output)
    h1 = tanh(W1 * x_norm + b1);
    h2 = tanh(W2 * h1 + b2);
    du_raw = w_out_live * h2 + b3;

    % 3. Bounded residual compensation (safety guarantee)
    du = [max(min(du_raw(1), 0.18), -0.18); ...
          max(min(du_raw(2), 0.18), -0.18); ...
          max(min(du_raw(3), 0.12), -0.12)];

    % 4. Online Lyapunov-Safe Weight Adaptation (MIT Rule + Sigma Leakage)
    if adapt_active && dt > 0.0
        lr     = 0.15;
        sigma  = 0.08;
        max_dw = 0.35;
        e_clamped = max(min(rate_err, 4.0), -4.0);

        dw = (lr * (e_clamped * h2') - sigma * (w_out_live - W3_base)) * dt;
        w_out_live = max(min(w_out_live + dw, W3_base + max_dw), W3_base - max_dw);
    end

    w_out_mag = norm(w_out_live - W3_base, 'fro');
end
