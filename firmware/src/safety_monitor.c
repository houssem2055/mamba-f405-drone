#include "safety_monitor.h"
#include <stddef.h>

void Safety_Monitor_Init(void) {
    /* Stateless per-frame evaluator; reserved for future hysteresis counters */
}

void Safety_Monitor_Update(
    float vbat,
    bool rc_ok,
    const AI_Diagnostics_t *ai,
    Safety_Status_t *out_status
) {
    if (!out_status) return;

    uint8_t flags = SAFETY_FLAG_NONE;

    if (vbat > 0.5f) { /* Ignore 0V when USB-only without LiPo */
        if (vbat < SAFETY_VBAT_CRIT_V) {
            flags |= (SAFETY_FLAG_LOW_BATTERY | SAFETY_FLAG_CRIT_BATTERY);
        } else if (vbat < SAFETY_VBAT_LOW_V) {
            flags |= SAFETY_FLAG_LOW_BATTERY;
        }
    }

    if (!rc_ok) {
        flags |= SAFETY_FLAG_RC_LOST;
    }

    float health = 1.0f;
    if (ai) {
        health = ai->health_score;
        if (ai->fault_class != AI_FAULT_NORMAL || ai->health_score < SAFETY_AI_HEALTH_MIN) {
            flags |= SAFETY_FLAG_AI_FAULT;
        }
    }

    out_status->flags = flags;
    out_status->vbat = vbat;
    out_status->ai_health = health;
    out_status->failsafe_active = ((flags & (SAFETY_FLAG_CRIT_BATTERY | SAFETY_FLAG_RC_LOST)) != 0U);
    out_status->is_safe_to_arm = (flags == SAFETY_FLAG_NONE);
}

const char* Safety_Monitor_Get_Flag_Text(uint8_t flags) {
    if (flags == SAFETY_FLAG_NONE) return "OK";
    if (flags & SAFETY_FLAG_CRIT_BATTERY) return "CRITICAL_BATTERY";
    if (flags & SAFETY_FLAG_RC_LOST) return "RC_LOST";
    if (flags & SAFETY_FLAG_AI_FAULT) return "AI_FAULT";
    if (flags & SAFETY_FLAG_LOW_BATTERY) return "LOW_BATTERY";
    return "UNKNOWN";
}
