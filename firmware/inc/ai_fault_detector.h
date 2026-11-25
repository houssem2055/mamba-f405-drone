#ifndef AI_FAULT_DETECTOR_H
#define AI_FAULT_DETECTOR_H

#include <stdint.h>
#include <stdbool.h>
#include "motors.h"

#define AI_WINDOW_SIZE 32U

typedef enum {
    AI_FAULT_NORMAL         = 0,
    AI_FAULT_PROP_VIBRATION = 1,
    AI_FAULT_MOTOR_LOSS     = 2,
    AI_FAULT_COLLISION      = 3
} AI_Fault_Class_t;

typedef struct {
    AI_Fault_Class_t fault_class;
    float health_score;          /* 0.0 (critical failure) .. 1.0 (healthy) */
    float class_probs[4];        /* Softmax probabilities [Normal, PropVib, MotorLoss, Collision] */
    float feat_gyro_jitter;
    float feat_accel_dev;
    float feat_motor_asym;
    float feat_rate_err;
    float feat_peak_g;
} AI_Diagnostics_t;

void AI_Fault_Detector_Init(void);
void AI_Fault_Detector_Update(
    float gx_raw, float gy_raw,
    float gx_filt, float gy_filt,
    float ax, float ay, float az,
    float err_p, float err_q,
    const Motor_Outputs_t *motors,
    bool is_armed,
    AI_Diagnostics_t *out_diag
);

#endif /* AI_FAULT_DETECTOR_H */
