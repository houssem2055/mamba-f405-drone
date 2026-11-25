#include "rc_receiver.h"
#include "config.h"

static RC_Channels_t s_rc_state = {
    .throttle = 0.0f,
    .roll_cmd = 0.0f,
    .pitch_cmd = 0.0f,
    .yaw_cmd = 0.0f,
    .arm_switch = false,
    .flight_mode = 1, /* Default to Angle/Self-leveling mode */
    .failsafe = true,
    .last_packet_time_ms = 0
};

/* Internal buffer for CRSF / SBUS frames */
static uint8_t s_rx_buf[64];
static uint8_t s_rx_idx = 0;

void RC_Receiver_Init(void) {
    s_rc_state.failsafe = true;
    s_rc_state.arm_switch = false;
    s_rc_state.throttle = 0.0f;
    s_rx_idx = 0;
}

/* Parse CRSF channel frame (Type 0x16: 16 channels packed into 22 bytes, 11 bits each) */
static void Parse_CRSF_Channels(const uint8_t *payload) {
    uint32_t channels[8] = {0};

    /* Decode channels from 11-bit packed words */
    channels[0] = ((payload[0] | (payload[1] << 8)) & 0x07FF);
    channels[1] = (((payload[1] >> 3) | (payload[2] << 5)) & 0x07FF);
    channels[2] = (((payload[2] >> 6) | (payload[3] << 2) | (payload[4] << 10)) & 0x07FF);
    channels[3] = (((payload[4] >> 1) | (payload[5] << 7)) & 0x07FF);
    channels[4] = (((payload[5] >> 4) | (payload[6] << 4)) & 0x07FF);
    channels[5] = (((payload[6] >> 7) | (payload[7] << 1) | (payload[8] << 9)) & 0x07FF);

    /* Normalize CRSF values: 172 is ~1000us, 992 is ~1500us, 1811 is ~2000us */
    /* Roll (Ch 0) */
    s_rc_state.roll_cmd = ((float)channels[0] - 992.0f) / 820.0f;
    /* Pitch (Ch 1) */
    s_rc_state.pitch_cmd = ((float)channels[1] - 992.0f) / 820.0f;
    /* Throttle (Ch 2): 0.0 to 1.0 */
    float thr = ((float)channels[2] - 172.0f) / 1640.0f;
    if (thr < 0.0f) thr = 0.0f;
    if (thr > 1.0f) thr = 1.0f;
    s_rc_state.throttle = thr;
    /* Yaw (Ch 3) */
    s_rc_state.yaw_cmd = ((float)channels[3] - 992.0f) / 820.0f;

    /* AUX1 (Ch 4): Arm Switch */
    s_rc_state.arm_switch = (channels[4] > 1000);

    /* AUX2 (Ch 5): Flight Mode (Acro vs Angle) */
    s_rc_state.flight_mode = (channels[5] > 1000) ? 1 : 0;

    s_rc_state.failsafe = false;
}

void RC_Receiver_Process_Byte(uint8_t byte, uint32_t current_time_ms) {
    /* CRSF Device Address Sync: 0xC8 (Flight Controller target) */
    if (s_rx_idx == 0 && byte != 0xC8) {
        return; /* Wait for sync byte */
    }

    if (s_rx_idx >= sizeof(s_rx_buf)) {
        s_rx_idx = 0;
        return;
    }

    s_rx_buf[s_rx_idx++] = byte;

    if (s_rx_idx >= 2) {
        uint8_t frame_len = s_rx_buf[1];
        if (frame_len > 60) {
            s_rx_idx = 0; /* Reset on corrupt frame length */
            return;
        }

        /* Full frame received: address (1) + length (1) + payload (frame_len) */
        if (s_rx_idx >= (frame_len + 2)) {
            uint8_t packet_type = s_rx_buf[2];
            if (packet_type == 0x16) { /* CRSF RC Channels Packed */
                Parse_CRSF_Channels(&s_rx_buf[3]);
                s_rc_state.last_packet_time_ms = current_time_ms;
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

        /* Keep internal state synchronized */
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
