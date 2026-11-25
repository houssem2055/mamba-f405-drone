#ifndef RC_RECEIVER_H
#define RC_RECEIVER_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    float throttle;     /* 0.0 to 1.0 */
    float roll_cmd;     /* Scaled to rad (Angle/Horizon) or rad/s (Acro) */
    float pitch_cmd;    /* Scaled to rad (Angle/Horizon) or rad/s (Acro) */
    float yaw_cmd;      /* rad/s */

    bool    arm_switch;   /* AUX1: true when armed switch is active */
    uint8_t flight_mode;  /* AUX2: 0 = Acro, 1 = Angle, 2 = Horizon */
    bool    airmode;      /* AUX3: true when Airmode is enabled */
    bool    beeper;       /* AUX4: true when Beeper switch is active */

    bool     failsafe;            /* true if signal lost > RC_FAILSAFE_TIMEOUT_MS */
    uint32_t last_packet_time_ms; /* Timestamp of last valid CRC8-verified frame */
    uint32_t valid_frames;        /* Total valid CRSF frames decoded */
    uint32_t crc_errors;          /* Total corrupted frames rejected by CRC8 */
} RC_Channels_t;

void RC_Receiver_Init(void);
void RC_Receiver_Process_Byte(uint8_t byte, uint32_t current_time_ms);
void RC_Receiver_Check_Failsafe(RC_Channels_t *rc, uint32_t current_time_ms);
void RC_Get_Channels(RC_Channels_t *out_channels);

/* Utility helpers (also used by unit tests) */
uint8_t RC_Compute_CRSF_CRC8(const uint8_t *data, uint8_t len);
float   RC_Apply_Expo(float stick_norm, float expo);

#endif /* RC_RECEIVER_H */
