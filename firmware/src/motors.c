#include "motors.h"
#include "config.h"

/* Platform timer compare register hooks */
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
