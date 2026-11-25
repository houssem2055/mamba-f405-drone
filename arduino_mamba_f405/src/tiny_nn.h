#ifndef TINY_NN_H
#define TINY_NN_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @file    tiny_nn.h
 * @brief   Deterministic, Zero-Heap TinyML Neural Network Inference &
 *          Online Lyapunov-Safe Weight Adaptation Engine for STM32F405 Cortex-M4F.
 */

#define TINYNN_MAX_IN      8U
#define TINYNN_MAX_H1      16U
#define TINYNN_MAX_H2      16U
#define TINYNN_MAX_OUT     4U

typedef struct {
    uint8_t n_in;
    uint8_t n_h1;
    uint8_t n_h2;   /* 0 for single-hidden-layer network */
    uint8_t n_out;

    /* Normalization: x_norm[i] = (x[i] - in_mean[i]) * in_inv_std[i] */
    const float *in_mean;
    const float *in_inv_std;

    /* Layer 1: [n_h1 x n_in], bias [n_h1] */
    const float *w1;
    const float *b1;

    /* Layer 2 (optional if n_h2 > 0): [n_h2 x n_h1], bias [n_h2] */
    const float *w2;
    const float *b2;

    /* Output Layer base weights: [n_out x last_hidden], bias [n_out] */
    const float *w_out_base;
    const float *b_out;

    /* Optional mutable online-adapted output weights [n_out * TINYNN_MAX_H2] */
    float w_out_live[TINYNN_MAX_OUT * TINYNN_MAX_H2];
    float last_features[TINYNN_MAX_H2];
    bool  online_adapt_enabled;
} TinyNN_Model_t;

void TinyNN_Init(
    TinyNN_Model_t *model,
    uint8_t n_in, uint8_t n_h1, uint8_t n_h2, uint8_t n_out,
    const float *in_mean, const float *in_inv_std,
    const float *w1, const float *b1,
    const float *w2, const float *b2,
    const float *w_out_base, const float *b_out,
    bool enable_online_adapt
);

void TinyNN_Reset_Live_Weights(TinyNN_Model_t *model);

/**
 * @brief Run forward pass (tanh hidden activations, linear output).
 */
void TinyNN_Forward(TinyNN_Model_t *model, const float *inputs, float *outputs);

/**
 * @brief Online Lyapunov-stable output weight update (MIT-rule with sigma leakage).
 *        dW = lr * error * features^T - sigma * (W_live - W_base)
 */
void TinyNN_Adapt_Output(
    TinyNN_Model_t *model,
    const float *rate_error,
    float learning_rate,
    float leakage_sigma,
    float max_delta_weight,
    float dt
);

/**
 * @brief Compute Softmax probabilities over raw logits.
 */
void TinyNN_Softmax(const float *logits, float *probs, uint8_t len);

#endif /* TINY_NN_H */
