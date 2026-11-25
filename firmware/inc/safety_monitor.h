#ifndef SAFETY_MONITOR_H
#define SAFETY_MONITOR_H

#include <stdint.h>
#include <stdbool.h>
#include "ai_fault_detector.h"

/* Safety flag bitmask */
#define SAFETY_FLAG_NONE         0x00U
#define SAFETY_FLAG_LOW_BATTERY  0x01U
#define SAFETY_FLAG_CRIT_BATTERY 0x02U
#define SAFETY_FLAG_RC_LOST      0x04U
#define SAFETY_FLAG_AI_FAULT     0x08U

/* Default 4S LiPo voltage thresholds (Volts) */
#define SAFETY_VBAT_LOW_V        14.0f
#define SAFETY_VBAT_CRIT_V       13.2f
#define SAFETY_AI_HEALTH_MIN     0.50f

typedef struct {
    uint8_t flags;
    bool    is_safe_to_arm;
    bool    failsafe_active;
    float   vbat;
    float   ai_health;
} Safety_Status_t;

void Safety_Monitor_Init(void);
void Safety_Monitor_Update(
    float vbat,
    bool rc_ok,
    const AI_Diagnostics_t *ai,
    Safety_Status_t *out_status
);
const char* Safety_Monitor_Get_Flag_Text(uint8_t flags);

#endif /* SAFETY_MONITOR_H */
