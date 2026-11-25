/**
 ******************************************************************************
 * @file    main.c
 * @brief   MAMBA F405 MK2 V2 Flight Controller Main Application
 *          Integrates MPU6000 IMU, CRSF RC Receiver, 4-channel PWM Motor Driver,
 *          and MATLAB-designed cascaded Attitude & Rate Flight Controller.
 ******************************************************************************
 */

#include "config.h"
#include "mpu6000.h"
#include "rc_receiver.h"
#include "motors.h"
#include "flight_controller.h"
#include "telemetry.h"
#include <math.h>

/* Global volatile tick flags */
static volatile uint32_t s_system_ticks_ms = 0;
static volatile bool s_loop_trigger_1khz = false;

/* Data structures */
static MPU6000_Data_t s_imu;
static RC_Channels_t  s_rc;
static Motor_Outputs_t s_motors;
static Attitude_t     s_attitude;
static bool           s_is_armed = false;

/* SysTick interrupt handler (called every 1 ms by hardware timer) */
void SysTick_Handler(void) {
    s_system_ticks_ms++;
    s_loop_trigger_1khz = true;
}

static void System_Init(void) {
    /* 1. Configure 168 MHz System Clocks, Flash Latency, and Power */
    /* 2. Initialize GPIOs for Status LED and Buzzer */
    /* 3. Initialize SPI1 for MPU6000 */
    /* 4. Initialize Timers (TIM3 / TIM2) for 4 Motor PWM outputs */
    /* 5. Initialize UART1 (CRSF RX) and UART6 (MATLAB Telemetry TX) */
    Motors_Init();
    RC_Receiver_Init();
    Telemetry_Init();
    Flight_Controller_Init();
}

static bool Check_Arming_Conditions(const RC_Channels_t *rc, const Attitude_t *att) {
    if (rc->failsafe) return false;
    if (!rc->arm_switch) return false;
    if (rc->throttle > MIN_THROTTLE_FOR_ARM) return false;
    
    /* Disallow arming if the drone is upside down or heavily tilted */
    if (fabsf(att->roll_deg) > MAX_ARM_TILT_DEG || fabsf(att->pitch_deg) > MAX_ARM_TILT_DEG) {
        return false;
    }
    return true;
}

int main(void) {
    System_Init();

    /* Initialize MPU6000 Gyro & Accel over SPI1 */
    bool imu_ok = MPU6000_Init();
    if (imu_ok) {
        /* Calibrate gyro while drone is stationary on flat surface */
        MPU6000_Calibrate_Gyro(&s_imu, 1000);
    }

    uint32_t telemetry_counter = 0;

    /* ====================================================================== */
    /*               DETERMINISTIC 1 kHz MAIN FLIGHT LOOP                     */
    /* ====================================================================== */
    while (1) {
        if (!s_loop_trigger_1khz) {
            continue; /* Wait for next 1ms SysTick interrupt */
        }
        s_loop_trigger_1khz = false;

        uint32_t now_ms = s_system_ticks_ms;

        /* 1. Read IMU Data (Accelerometer & Gyroscope) via SPI1 */
        MPU6000_Read_All(&s_imu);

        /* 2. Process RC Receiver & Check Failsafe */
        RC_Get_Channels(&s_rc);
        RC_Receiver_Check_Failsafe(&s_rc, now_ms);

        /* 3. Arming State Logic */
        if (!s_is_armed) {
            if (Check_Arming_Conditions(&s_rc, &s_attitude)) {
                s_is_armed = true;
            }
        } else {
            /* Immediate disarm condition */
            if (!s_rc.arm_switch || s_rc.failsafe) {
                s_is_armed = false;
            }
        }

        /* 4. Execute Flight Controller (Attitude Filter + Cascaded PID + Mixer) */
        Flight_Controller_Update(
            &s_rc,
            &s_imu,
            s_is_armed,
            LOOP_DT,
            &s_motors,
            &s_attitude
        );

        /* 5. Update ESC Motor PWM Outputs */
        if (s_is_armed) {
            Motors_Set_All_Pwm(&s_motors);
        } else {
            Motors_Disarm();
        }

        /* 6. Send Telemetry to MATLAB Ground Station at 50 Hz (every 20 ms) */
        telemetry_counter++;
        if (telemetry_counter >= 20) {
            telemetry_counter = 0;
            float vbat_mock = 15.2f; /* 4S LiPo voltage from ADC */
            Telemetry_Send_Frame(&s_attitude, &s_motors, vbat_mock);
        }
    }

    return 0;
}
