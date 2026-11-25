#ifndef RC_RECEIVER_H
#define RC_RECEIVER_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    float throttle;     /* 0.0 to 1.0 */
    float roll_cmd;     /* -1.0 to +1.0 (or radians) */
    float pitch_cmd;    /* -1.0 to +1.0 (or radians) */
    float yaw_cmd;      /* -1.0 to +1.0 (or radians/sec) */
    
    bool  arm_switch;   /* true when armed switch is flipped */
    uint8_t flight_mode;/* 0 = Acro/Rate, 1 = Angle/Level */
    
    bool  failsafe;     /* true if connection lost */
    uint32_t last_packet_time_ms;
} RC_Channels_t;

void RC_Receiver_Init(void);
void RC_Receiver_Process_Byte(uint8_t byte, uint32_t current_time_ms);
void RC_Receiver_Check_Failsafe(RC_Channels_t *rc, uint32_t current_time_ms);
void RC_Get_Channels(RC_Channels_t *out_channels);

#endif /* RC_RECEIVER_H */
