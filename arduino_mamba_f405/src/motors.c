#include "motors.h"
#include "config.h"

/* Platform timer compare register hook */
extern void Timer_Set_PWM_Pulse_Us(uint8_t channel, float pulse_us);

static float Clamp_Pwm(float val) {
    if (val < MOTOR_MIN_PWM_US) return MOTOR_MIN_PWM_US;
    if (val > MOTOR_MAX_PWM_US) return MOTOR_MAX_PWM_US;
    return val;
}

void Motors_Init(void) {
    Motors_Disarm();
}

void Motors_Set_All_Pwm(const Motor_Outputs_t *motors) {
    if (!motors) return;

    Timer_Set_PWM_Pulse_Us(1, Clamp_Pwm(motors->m1));
    Timer_Set_PWM_Pulse_Us(2, Clamp_Pwm(motors->m2));
    Timer_Set_PWM_Pulse_Us(3, Clamp_Pwm(motors->m3));
    Timer_Set_PWM_Pulse_Us(4, Clamp_Pwm(motors->m4));
}

void Motors_Disarm(void) {
    Timer_Set_PWM_Pulse_Us(1, MOTOR_MIN_PWM_US);
    Timer_Set_PWM_Pulse_Us(2, MOTOR_MIN_PWM_US);
    Timer_Set_PWM_Pulse_Us(3, MOTOR_MIN_PWM_US);
    Timer_Set_PWM_Pulse_Us(4, MOTOR_MIN_PWM_US);
}

uint16_t Motors_Pwm_To_DShot_Throttle(float pulse_us, bool armed) {
    if (!armed || pulse_us <= MOTOR_MIN_PWM_US) {
        return 0U; /* DShot command 0 = Disarm / Stop */
    }
    float clamped = Clamp_Pwm(pulse_us);
    /* DShot active throttle range is 48 .. 2047 (2000 discrete steps) */
    float norm = (clamped - MOTOR_MIN_PWM_US) / (MOTOR_MAX_PWM_US - MOTOR_MIN_PWM_US);
    uint16_t val = (uint16_t)(48.0f + norm * 1999.0f + 0.5f);
    if (val < 48U)   val = 48U;
    if (val > 2047U) val = 2047U;
    return val;
}

uint16_t Motors_Encode_DShot_Packet(uint16_t throttle_11bit, bool telemetry_req) {
    if (throttle_11bit > 2047U) throttle_11bit = 2047U;
    uint16_t packet = (uint16_t)((throttle_11bit << 1) | (telemetry_req ? 1U : 0U));
    /* Compute 4-bit XOR checksum across the three 4-bit nibbles */
    uint16_t crc = (packet ^ (packet >> 4) ^ (packet >> 8)) & 0x0FU;
    return (uint16_t)((packet << 4) | crc);
}
