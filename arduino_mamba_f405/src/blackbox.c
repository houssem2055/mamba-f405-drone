#include "blackbox.h"
#include <string.h>

static Blackbox_Frame_t s_ring[BLACKBOX_BUFFER_CAPACITY];
static uint16_t s_head = 0;
static uint16_t s_tail = 0;
static uint16_t s_count = 0;

void Blackbox_Init(void) {
    s_head = 0;
    s_tail = 0;
    s_count = 0;
    memset(s_ring, 0, sizeof(s_ring));
}

static uint8_t Compute_Frame_XOR(const uint8_t *bytes, uint16_t len) {
    uint8_t x = 0;
    for (uint16_t i = 0; i < len; i++) {
        x ^= bytes[i];
    }
    return x;
}

bool Blackbox_Record_Frame(
    uint32_t time_ms,
    const Attitude_t *att,
    const Motor_Outputs_t *motors,
    float vbat,
    bool armed,
    uint8_t mode
) {
    if (!att || !motors) return false;
    if (s_count >= BLACKBOX_BUFFER_CAPACITY) {
        /* Drop oldest frame if SPI3 flash flush lags */
        s_tail = (uint16_t)((s_tail + 1U) % BLACKBOX_BUFFER_CAPACITY);
        s_count--;
    }

    Blackbox_Frame_t *f = &s_ring[s_head];
    f->magic        = 0x4D424258UL;
    f->timestamp_ms = time_ms;
    f->roll_cdeg    = (int16_t)(att->roll_deg * 100.0f);
    f->pitch_cdeg   = (int16_t)(att->pitch_deg * 100.0f);
    f->yaw_cdeg     = (int16_t)(att->yaw_deg * 100.0f);
    f->gx_mrad      = (int16_t)(att->gx_filt * 1000.0f);
    f->gy_mrad      = (int16_t)(att->gy_filt * 1000.0f);
    f->gz_mrad      = (int16_t)(att->gz_filt * 1000.0f);
    f->m1_us        = (uint16_t)motors->m1;
    f->m2_us        = (uint16_t)motors->m2;
    f->m3_us        = (uint16_t)motors->m3;
    f->m4_us        = (uint16_t)motors->m4;
    f->vbat_mv      = (uint16_t)(vbat * 1000.0f);
    f->flags        = (uint8_t)((armed ? 1U : 0U) | ((mode & 0x03U) << 1));
    f->checksum     = Compute_Frame_XOR((const uint8_t *)f, (uint16_t)(sizeof(Blackbox_Frame_t) - 1U));

    s_head = (uint16_t)((s_head + 1U) % BLACKBOX_BUFFER_CAPACITY);
    s_count++;
    return true;
}

bool Blackbox_Pop_Frame(Blackbox_Frame_t *out_frame) {
    if (!out_frame || s_count == 0U) return false;
    *out_frame = s_ring[s_tail];
    s_tail = (uint16_t)((s_tail + 1U) % BLACKBOX_BUFFER_CAPACITY);
    s_count--;
    return true;
}

uint16_t Blackbox_Pending_Count(void) {
    return s_count;
}
