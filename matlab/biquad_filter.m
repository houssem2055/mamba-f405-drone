function [y, state_out] = biquad_filter(x, state_in, type_id, cutoff_hz, fs_hz, q_factor)
%#codegen
% BIQUAD_FILTER
% 2nd-order Butterworth Low-Pass (type_id = 0) or Notch (type_id = 1) filter
% using Transposed Direct Form II (bit-accurate architecture with firmware/src/biquad.c).

    nyquist = 0.48 * fs_hz;
    if cutoff_hz > nyquist
        cutoff_hz = nyquist;
    end
    if q_factor < 0.1
        q_factor = 0.70710678;
    end

    omega = (2.0 * pi * cutoff_hz) / fs_hz;
    sn = sin(omega);
    cs = cos(omega);
    alpha = sn / (2.0 * q_factor);

    if type_id == 1
        % Notch Filter
        b0 = 1.0;
        b1 = -2.0 * cs;
        b2 = 1.0;
        a0 = 1.0 + alpha;
        a1 = -2.0 * cs;
        a2 = 1.0 - alpha;
    else
        % 2nd-order Butterworth LPF
        b0 = (1.0 - cs) * 0.5;
        b1 = 1.0 - cs;
        b2 = (1.0 - cs) * 0.5;
        a0 = 1.0 + alpha;
        a1 = -2.0 * cs;
        a2 = 1.0 - alpha;
    end

    inv_a0 = 1.0 / a0;
    b0 = b0 * inv_a0;
    b1 = b1 * inv_a0;
    b2 = b2 * inv_a0;
    a1 = a1 * inv_a0;
    a2 = a2 * inv_a0;

    z1 = state_in(1);
    z2 = state_in(2);

    y = b0 * x + z1;
    z1_new = b1 * x - a1 * y + z2;
    z2_new = b2 * x - a2 * y;

    state_out = [z1_new; z2_new];
end
