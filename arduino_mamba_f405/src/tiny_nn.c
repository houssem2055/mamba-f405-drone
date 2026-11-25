#include "tiny_nn.h"
#include <math.h>
#include <string.h>

static inline float Fast_Clamp(float x, float min_v, float max_v) {
    if (x < min_v) return min_v;
    if (x > max_v) return max_v;
    return x;
}

void TinyNN_Init(
    TinyNN_Model_t *model,
    uint8_t n_in, uint8_t n_h1, uint8_t n_h2, uint8_t n_out,
    const float *in_mean, const float *in_inv_std,
    const float *w1, const float *b1,
    const float *w2, const float *b2,
    const float *w_out_base, const float *b_out,
    bool enable_online_adapt
) {
    if (!model) return;
    memset(model, 0, sizeof(*model));

    model->n_in  = (n_in  <= TINYNN_MAX_IN)  ? n_in  : TINYNN_MAX_IN;
    model->n_h1  = (n_h1  <= TINYNN_MAX_H1)  ? n_h1  : TINYNN_MAX_H1;
    model->n_h2  = (n_h2  <= TINYNN_MAX_H2)  ? n_h2  : TINYNN_MAX_H2;
    model->n_out = (n_out <= TINYNN_MAX_OUT) ? n_out : TINYNN_MAX_OUT;

    model->in_mean     = in_mean;
    model->in_inv_std  = in_inv_std;
    model->w1          = w1;
    model->b1          = b1;
    model->w2          = w2;
    model->b2          = b2;
    model->w_out_base  = w_out_base;
    model->b_out       = b_out;
    model->online_adapt_enabled = enable_online_adapt;

    TinyNN_Reset_Live_Weights(model);
}

void TinyNN_Reset_Live_Weights(TinyNN_Model_t *model) {
    if (!model || !model->w_out_base) return;
    uint8_t feat_dim = (model->n_h2 > 0U) ? model->n_h2 : model->n_h1;
    uint16_t total = (uint16_t)model->n_out * (uint16_t)feat_dim;
    for (uint16_t i = 0; i < total; i++) {
        model->w_out_live[i] = model->w_out_base[i];
    }
    memset(model->last_features, 0, sizeof(model->last_features));
}

void TinyNN_Forward(TinyNN_Model_t *model, const float *inputs, float *outputs) {
    if (!model || !inputs || !outputs) return;

    float x_norm[TINYNN_MAX_IN];
    for (uint8_t i = 0; i < model->n_in; i++) {
        float mean = model->in_mean    ? model->in_mean[i]    : 0.0f;
        float invs = model->in_inv_std ? model->in_inv_std[i] : 1.0f;
        x_norm[i] = Fast_Clamp((inputs[i] - mean) * invs, -6.0f, 6.0f);
    }

    /* Hidden Layer 1 (tanh) */
    float h1[TINYNN_MAX_H1];
    for (uint8_t j = 0; j < model->n_h1; j++) {
        float sum = model->b1 ? model->b1[j] : 0.0f;
        const float *row = &model->w1[(uint16_t)j * model->n_in];
        for (uint8_t i = 0; i < model->n_in; i++) {
            sum += row[i] * x_norm[i];
        }
        h1[j] = tanhf(sum);
    }

    const float *last_h = h1;
    uint8_t last_dim = model->n_h1;

    /* Optional Hidden Layer 2 (tanh) */
    float h2[TINYNN_MAX_H2];
    if (model->n_h2 > 0U && model->w2) {
        for (uint8_t j = 0; j < model->n_h2; j++) {
            float sum = model->b2 ? model->b2[j] : 0.0f;
            const float *row = &model->w2[(uint16_t)j * model->n_h1];
            for (uint8_t i = 0; i < model->n_h1; i++) {
                sum += row[i] * h1[i];
            }
            h2[j] = tanhf(sum);
        }
        last_h = h2;
        last_dim = model->n_h2;
    }

    /* Store features for potential online weight adaptation */
    for (uint8_t i = 0; i < last_dim; i++) {
        model->last_features[i] = last_h[i];
    }

    /* Output Layer (uses w_out_live if online adaptation enabled, else w_out_base) */
    const float *w_out = model->online_adapt_enabled ? model->w_out_live : model->w_out_base;
    for (uint8_t k = 0; k < model->n_out; k++) {
        float sum = model->b_out ? model->b_out[k] : 0.0f;
        const float *row = &w_out[(uint16_t)k * last_dim];
        for (uint8_t i = 0; i < last_dim; i++) {
            sum += row[i] * last_h[i];
        }
        outputs[k] = sum;
    }
}

void TinyNN_Adapt_Output(
    TinyNN_Model_t *model,
    const float *rate_error,
    float learning_rate,
    float leakage_sigma,
    float max_delta_weight,
    float dt
) {
    if (!model || !model->online_adapt_enabled || !rate_error || !model->w_out_base || dt <= 0.0f) {
        return;
    }

    uint8_t feat_dim = (model->n_h2 > 0U) ? model->n_h2 : model->n_h1;

    for (uint8_t k = 0; k < model->n_out; k++) {
        float err = Fast_Clamp(rate_error[k], -4.0f, 4.0f);
        uint16_t offset = (uint16_t)k * feat_dim;
        for (uint8_t i = 0; i < feat_dim; i++) {
            float w_base = model->w_out_base[offset + i];
            float w_curr = model->w_out_live[offset + i];
            float feat   = model->last_features[i];

            /* Sigma-modification guarantees bounded adaptive weights */
            float dw = (learning_rate * err * feat - leakage_sigma * (w_curr - w_base)) * dt;
            float w_next = Fast_Clamp(w_curr + dw, w_base - max_delta_weight, w_base + max_delta_weight);
            model->w_out_live[offset + i] = w_next;
        }
    }
}

void TinyNN_Softmax(const float *logits, float *probs, uint8_t len) {
    if (!logits || !probs || len == 0U) return;

    float max_val = logits[0];
    for (uint8_t i = 1; i < len; i++) {
        if (logits[i] > max_val) max_val = logits[i];
    }

    float sum_exp = 0.0f;
    for (uint8_t i = 0; i < len; i++) {
        float e = expf(logits[i] - max_val);
        probs[i] = e;
        sum_exp += e;
    }

    if (sum_exp > 1e-12f) {
        float inv = 1.0f / sum_exp;
        for (uint8_t i = 0; i < len; i++) {
            probs[i] *= inv;
        }
    }
}
