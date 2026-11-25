#ifndef BLACKBOX_H
#define BLACKBOX_H

#include <stdint.h>
#include <stdbool.h>
#include "flight_controller.h"
#include "motors.h"

#define BLACKBOX_BUFFER_CAPACITY 64U

typedef struct __attribute__((packed)) {
    uint32_t magic;        /* 0x4D424258 ('MBBX') */
    uint32_t timestamp_ms;
    int16_t  roll_cdeg;    /* Centidegrees (deg * 100) */
    int16_t  pitch_cdeg;
    int16_t  yaw_cdeg;
    int16_t  gx_mrad;      /* millirad/s */
    int16_t  gy_mrad;
    int16_t  gz_mrad;
    uint16_t m1_us;
    uint16_t m2_us;
    uint16_t m3_us;
    uint16_t m4_us;
    uint16_t vbat_mv;      /* millivolts */
    uint8_t  flags;        /* bit 0: armed, bits 1-2: mode */
    uint8_t  checksum;     /* XOR checksum of bytes 0..30 */
} Blackbox_Frame_t;

void     Blackbox_Init(void);
bool     Blackbox_Record_Frame(uint32_t time_ms, const Attitude_t *att, const Motor_Outputs_t *motors, float vbat, bool armed, uint8_t mode);
bool     Blackbox_Pop_Frame(Blackbox_Frame_t *out_frame);
uint16_t Blackbox_Pending_Count(void);

#endif /* BLACKBOX_H */
