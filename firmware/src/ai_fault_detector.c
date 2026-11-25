#include "ai_fault_detector.h"
#include "tiny_nn.h"
#include "ai_weights.h"
#include <math.h>
#include <string.h>

static TinyNN_Model_t s_fault_net;

/* Rolling window buffers (32 ms @ 1 kHz) */
static float s_vib_buf[AI_WINDOW_SIZE];
static float s_acc_dev_buf[AI_WINDOW_SIZE];
static float s_asym_buf[AI_WINDOW_SIZE];
static float s_rerr_buf[AI_WINDOW_SIZE];
static float s_acc_mag_buf[AI_WINDOW_SIZE];
static uint8_t s_win_idx = 0;
static uint8_t s_win_count = 0;
static AI_Diagnostics_t s_last_diag;

void AI_Fault_Detector_Init(void) {
    TinyNN_Init(
        &s_fault_net,
        AI_FAULT_IN_DIM, AI_FAULT_H1_DIM, 0U, AI_FAULT_OUT_DIM,
        AI_FAULT_IN_MEAN, AI_FAULT_IN_INV_STD,
        AI_FAULT_W1, AI_FAULT_B1,
        NULL, NULL,
        AI_FAULT_W2, AI_FAULT_B2,
        false
    );

    memset(s_vib_buf, 0, sizeof(s_vib_buf));
    memset(s_acc_dev_buf, 0, sizeof(s_acc_dev_buf));
    memset(s_asym_buf, 0, sizeof(s_asym_buf));
    memset(s_rerr_buf, 0, sizeof(s_rerr_buf));
    memset(s_acc_mag_buf, 0, sizeof(s_acc_mag_buf));
    s_win_idx = 0;
    s_win_count = 0;

    memset(&s_last_diag, 0, sizeof(s_last_diag));
    s_last_diag.fault_class = AI_FAULT_NORMAL;
    s_last_diag.health_score = 1.0f;
    s_last_diag.class_probs[0] = 1.0f;
}

void AI_Fault_Detector_Update(
    float gx_raw, float gy_raw,
    float gx_filt, float gy_filt,
    float ax, float ay, float az,
    float err_p, float err_q,
    const Motor_Outputs_t *motors,
    bool is_armed,
    AI_Diagnostics_t *out_diag
) {
    if (!motors || !out_diag) return;

    /* Instantaneous residual features */
    float dvx = gx_raw - gx_filt;
    float dvy = gy_raw - gy_filt;
    float vib_sq = dvx * dvx + dvy * dvy;

    float acc_mag = sqrtf(ax * ax + ay * ay + az * az);
    float acc_dev = fabsf(acc_mag - 9.81f);

    float asym = 0.0f;
    if (is_armed) {
        float roll_asym  = fabsf((motors->m1 + motors->m2) - (motors->m3 + motors->m4));
        float pitch_asym = fabsf((motors->m1 + motors->m3) - (motors->m2 + motors->m4));
        asym = 0.5f * (roll_asym + pitch_asym);
    }

    float rerr_sq = err_p * err_p + err_q * err_q;

    s_vib_buf[s_win_idx]     = vib_sq;
    s_acc_dev_buf[s_win_idx] = acc_dev;
    s_asym_buf[s_win_idx]    = asym;
    s_rerr_buf[s_win_idx]    = rerr_sq;
    s_acc_mag_buf[s_win_idx] = acc_mag;

    s_win_idx = (uint8_t)((s_win_idx + 1U) % AI_WINDOW_SIZE);
    if (s_win_count < AI_WINDOW_SIZE) s_win_count++;

    /* Run neural inference every 10 samples (100 Hz) once window filled */
    if (s_win_count == AI_WINDOW_SIZE && (s_win_idx % 10U) == 0U) {
        float sum_vib = 0.0f, sum_adev = 0.0f, sum_asym = 0.0f, sum_rerr = 0.0f;
        float peak_g = 0.0f;
        for (uint8_t i = 0; i < AI_WINDOW_SIZE; i++) {
            sum_vib  += s_vib_buf[i];
            sum_adev += s_acc_dev_buf[i];
            sum_asym += s_asym_buf[i];
            sum_rerr += s_rerr_buf[i];
            if (s_acc_mag_buf[i] > peak_g) peak_g = s_acc_mag_buf[i];
        }

        float inv_n = 1.0f / (float)AI_WINDOW_SIZE;
        float features[AI_FAULT_IN_DIM] = {
            sqrtf(sum_vib  * inv_n),
            sum_adev * inv_n,
            sum_asym * inv_n,
            sqrtf(sum_rerr * inv_n),
            peak_g / 9.81f
        };

        float logits[AI_FAULT_OUT_DIM];
        float probs[AI_FAULT_OUT_DIM];
        TinyNN_Forward(&s_fault_net, features, logits);
        TinyNN_Softmax(logits, probs, AI_FAULT_OUT_DIM);

        uint8_t best_cls = 0U;
        for (uint8_t c = 1; c < AI_FAULT_OUT_DIM; c++) {
            if (probs[c] > probs[best_cls]) best_cls = c;
        }

        s_last_diag.fault_class      = (AI_Fault_Class_t)best_cls;
        s_last_diag.health_score     = probs[0];
        s_last_diag.feat_gyro_jitter = features[0];
        s_last_diag.feat_accel_dev   = features[1];
        s_last_diag.feat_motor_asym  = features[2];
        s_last_diag.feat_rate_err    = features[3];
        s_last_diag.feat_peak_g      = features[4];
        for (uint8_t c = 0; c < AI_FAULT_OUT_DIM; c++) {
            s_last_diag.class_probs[c] = probs[c];
        }
    }

    *out_diag = s_last_diag;
}
