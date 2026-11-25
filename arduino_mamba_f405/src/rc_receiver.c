#include "rc_receiver.h"
#include "config.h"
#include <math.h>

static RC_Channels_t s_rc_state = {
    .throttle = 0.0f,
    .roll_cmd = 0.0f,
    .pitch_cmd = 0.0f,
    .yaw_cmd = 0.0f,
    .arm_switch = false,
    .flight_mode = 1, /* Default: Angle mode */
    .airmode = true,
    .beeper = false,
    .failsafe = true,
    .last_packet_time_ms = 0,
    .valid_frames = 0,
    .crc_errors = 0
};

static uint8_t s_rx_buf[64];
static uint8_t s_rx_idx = 0;

void RC_Receiver_Init(void) {
    s_rc_state.failsafe = true;
    s_rc_state.arm_switch = false;
    s_rc_state.throttle = 0.0f;
    s_rc_state.roll_cmd = 0.0f;
    s_rc_state.pitch_cmd = 0.0f;
    s_rc_state.yaw_cmd = 0.0f;
    s_rc_state.airmode = true;
    s_rc_state.beeper = false;
    s_rc_state.valid_frames = 0;
    s_rc_state.crc_errors = 0;
    s_rx_idx = 0;
}

/* Standard CRSF CRC-8 DVB-S2 (polynomial 0xD5) */
uint8_t RC_Compute_CRSF_CRC8(const uint8_t *data, uint8_t len) {
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t b = 0; b < 8; b++) {
            if (crc & 0x80U) {
                crc = (uint8_t)((crc << 1) ^ 0xD5U);
            } else {
                crc = (uint8_t)(crc << 1);
            }
        }
    }
    return crc;
}

float RC_Apply_Expo(float stick_norm, float expo) {
    if (stick_norm > 1.0f) stick_norm = 1.0f;
    if (stick_norm < -1.0f) stick_norm = -1.0f;
    if (fabsf(stick_norm) < 0.015f) {
        return 0.0f; /* Center stick deadband (1.5%) */
    }
    return stick_norm * (1.0f - expo) + (stick_norm * stick_norm * stick_norm) * expo;
}

/* Parse CRSF RC Channels frame (Type 0x16: 22-byte payload for 16 x 11-bit channels) */
static void Parse_CRSF_Channels(const uint8_t *payload) {
    uint32_t ch[8] = {0};

    ch[0] = ((payload[0] | (payload[1] << 8)) & 0x07FFU);
    ch[1] = (((payload[1] >> 3) | (payload[2] << 5)) & 0x07FFU);
    ch[2] = (((payload[2] >> 6) | (payload[3] << 2) | (payload[4] << 10)) & 0x07FFU);
    ch[3] = (((payload[4] >> 1) | (payload[5] << 7)) & 0x07FFU);
    ch[4] = (((payload[5] >> 4) | (payload[6] << 4)) & 0x07FFU);
    ch[5] = (((payload[6] >> 7) | (payload[7] << 1) | (payload[8] << 9)) & 0x07FFU);
    ch[6] = (((payload[8] >> 2) | (payload[9] << 6)) & 0x07FFU);
    ch[7] = (((payload[9] >> 5) | (payload[10] << 3)) & 0x07FFU);

    /* Normalize sticks (-1.0 to +1.0) and apply RC Expo */
    float roll_norm  = ((float)ch[0] - 992.0f) / 820.0f;
    float pitch_norm = ((float)ch[1] - 992.0f) / 820.0f;
    float yaw_norm   = ((float)ch[3] - 992.0f) / 820.0f;

    float roll_curved  = RC_Apply_Expo(roll_norm,  RC_EXPO_FACTOR);
    float pitch_curved = RC_Apply_Expo(pitch_norm, RC_EXPO_FACTOR);
    float yaw_curved   = RC_Apply_Expo(yaw_norm,   RC_EXPO_FACTOR);

    /* Throttle (Ch 2): 0.0 to 1.0 */
    float thr = ((float)ch[2] - 172.0f) / 1640.0f;
    if (thr < 0.0f) thr = 0.0f;
    if (thr > 1.0f) thr = 1.0f;
    s_rc_state.throttle = thr;

    /* AUX1 (Ch 4): Arm Switch (> 1500us / > 1000 raw) */
    s_rc_state.arm_switch = (ch[4] > 1000U);

    /* AUX2 (Ch 5): 3-Position Flight Mode Switch */
    if (ch[5] < 700U) {
        s_rc_state.flight_mode = 0; /* Acro Mode */
    } else if (ch[5] < 1300U) {
        s_rc_state.flight_mode = 2; /* Horizon Mode */
    } else {
        s_rc_state.flight_mode = 1; /* Angle Mode */
    }

    /* AUX3 (Ch 6): Airmode enable (default true if unused/mid, false if pulled low) */
    s_rc_state.airmode = (ch[6] >= 600U);

    /* AUX4 (Ch 7): Beeper */
    s_rc_state.beeper = (ch[7] > 1300U);

    /* Scale roll/pitch/yaw according to mode */
    if (s_rc_state.flight_mode == 0) {
        /* Acro: rad/s */
        s_rc_state.roll_cmd  = roll_curved  * 4.5f;
        s_rc_state.pitch_cmd = pitch_curved * 4.5f;
    } else {
        /* Angle / Horizon: target tilt in rad (+/- 45 deg = 0.7854 rad) */
        s_rc_state.roll_cmd  = roll_curved  * 0.7854f;
        s_rc_state.pitch_cmd = pitch_curved * 0.7854f;
    }
    s_rc_state.yaw_cmd = yaw_curved * 3.5f;

    s_rc_state.failsafe = false;
    s_rc_state.valid_frames++;
}

void RC_Receiver_Process_Byte(uint8_t byte, uint32_t current_time_ms) {
    /* CRSF Sync byte: 0xC8 (Flight Controller) or 0xEE (Transmitter) */
    if (s_rx_idx == 0 && byte != 0xC8U && byte != 0xEEU) {
        return;
    }

    if (s_rx_idx >= sizeof(s_rx_buf)) {
        s_rx_idx = 0;
        return;
    }

    s_rx_buf[s_rx_idx++] = byte;

    if (s_rx_idx >= 2) {
        uint8_t frame_len = s_rx_buf[1];
        /* Valid CRSF frame length covers Type + Payload + CRC (minimum 2, max 60) */
        if (frame_len < 2U || frame_len > 60U) {
            s_rx_idx = 0;
            return;
        }

        if (s_rx_idx >= (uint8_t)(frame_len + 2U)) {
            uint8_t packet_type = s_rx_buf[2];
            uint8_t rx_crc      = s_rx_buf[frame_len + 1U];
            /* CRC covers Type + Payload (frame_len - 1 bytes starting at index 2) */
            uint8_t calc_crc    = RC_Compute_CRSF_CRC8(&s_rx_buf[2], (uint8_t)(frame_len - 1U));

            if (rx_crc == calc_crc) {
                if (packet_type == 0x16U && frame_len >= 24U) {
                    Parse_CRSF_Channels(&s_rx_buf[3]);
                    s_rc_state.last_packet_time_ms = current_time_ms;
                }
            } else {
                s_rc_state.crc_errors++;
            }
            s_rx_idx = 0;
        }
    }
}

void RC_Receiver_Check_Failsafe(RC_Channels_t *rc, uint32_t current_time_ms) {
    if (!rc) return;

    if ((current_time_ms - rc->last_packet_time_ms) > RC_FAILSAFE_TIMEOUT_MS) {
        rc->failsafe = true;
        rc->arm_switch = false;
        rc->throttle = 0.0f;
        rc->roll_cmd = 0.0f;
        rc->pitch_cmd = 0.0f;
        rc->yaw_cmd = 0.0f;

        s_rc_state.failsafe = true;
        s_rc_state.arm_switch = false;
        s_rc_state.throttle = 0.0f;
        s_rc_state.roll_cmd = 0.0f;
        s_rc_state.pitch_cmd = 0.0f;
        s_rc_state.yaw_cmd = 0.0f;
    }
}

void RC_Get_Channels(RC_Channels_t *out_channels) {
    if (out_channels) {
        *out_channels = s_rc_state;
    }
}
