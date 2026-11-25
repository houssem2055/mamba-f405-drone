#ifndef MOTORS_H
#define MOTORS_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    float m1; /* Rear-Right  (1000 - 2000 us) */
    float m2; /* Front-Right (1000 - 2000 us) */
    float m3; /* Rear-Left   (1000 - 2000 us) */
    float m4; /* Front-Left  (1000 - 2000 us) */
} Motor_Outputs_t;

void Motors_Init(void);
void Motors_Set_All_Pwm(const Motor_Outputs_t *motors);
void Motors_Disarm(void);

#endif /* MOTORS_H */
