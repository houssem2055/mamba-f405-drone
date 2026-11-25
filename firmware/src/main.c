/**
 ******************************************************************************
 * @file    main.c
 * @brief   MAMBA F405 MK2 V2 Pro Flight Controller Main Application
 *          Integrates:
 *            - MPU6000 1 kHz SPI Burst Read + Startup Calibration
 *            - 2nd-Order Butterworth LPF (90 Hz) + 220 Hz Notch Filter
 *            - Singularity-Free Quaternion Mahony AHRS
 *            - Cascaded P-PID (D-on-Measurement, Feedforward, TPA, Vbat Comp)
 *            - Airmode Dynamic Mixer Desaturation
 *            - CRC8-Validated CRSF RC Receiver + 300ms Failsafe
 *            - 250 Hz Binary Blackbox Ring Logger + 50 Hz Bidirectional Telemetry
 ******************************************************************************
 */

#include "config.h"
#include "mpu6000.h"
#include "rc_receiver.h"
#include "motors.h"
#include "flight_controller.h"
#include "telemetry.h"
#include "blackbox.h"
#include "ai_fault_detector.h"
#include <math.h>

/* Hardware Platform Hooks (implemented in bsp/stm32f405_hal_bsp.c or SIL host) */
extern void  System_Hardware_Init(void);
extern float ADC_Read_Vbat_Volts(void);
extern void  Status_LED_Set(bool on);
extern void  Buzzer_Set(bool on);

/* Global volatile tick flags */
static volatile uint32_t s_system_ticks_ms = 0;
static volatile bool     s_loop_trigger_1khz = false;

/* Flight State Structures */
static MPU6000_Data_t   s_imu;
static RC_Channels_t    s_rc;
static Motor_Outputs_t  s_motors;
static Attitude_t       s_attitude;
static AI_Diagnostics_t s_ai_diag;
static bool             s_is_armed = false;

/* SysTick interrupt handler (1 ms period = 1000 Hz) */
void SysTick_Handler(void) {
    s_system_ticks_ms++;
    s_loop_trigger_1khz = true;
}

uint32_t System_Millis(void) {
    return s_system_ticks_ms;
}

static void System_Init(void) {
    System_Hardware_Init();
    Motors_Init();
    RC_Receiver_Init();
    Telemetry_Init();
    Blackbox_Init();
    Flight_Controller_Init();
    AI_Fault_Detector_Init();
}

static bool Check_Arming_Conditions(const RC_Channels_t *rc, const Attitude_t *att) {
    if (rc->failsafe) return false;
    if (!rc->arm_switch) return false;
    if (rc->throttle > MIN_THROTTLE_FOR_ARM) return false;

    /* Disallow arming if tilted > 25 degrees (Mahony AHRS runs live while disarmed) */
    if (fabsf(att->roll_deg) > MAX_ARM_TILT_DEG || fabsf(att->pitch_deg) > MAX_ARM_TILT_DEG) {
        return false;
    }
    return true;
}

int main(void) {
    System_Init();

    bool imu_ok = MPU6000_Init();
    if (imu_ok) {
        Status_LED_Set(true);
        MPU6000_Calibrate_Gyro(&s_imu, 1000);
        MPU6000_Read_All(&s_imu);
        Flight_Controller_Reset_Attitude(s_imu.ax, s_imu.ay, s_imu.az);
        Status_LED_Set(false);
    }

    uint32_t telemetry_counter = 0;
    uint32_t ai_tlm_counter    = 0;
    uint32_t blackbox_counter  = 0;
    uint32_t led_counter       = 0;
    bool     led_state         = false;

    /* ====================================================================== */
    /*               DETERMINISTIC 1 kHz MAIN FLIGHT LOOP                     */
    /* ====================================================================== */
    while (1) {
        if (!s_loop_trigger_1khz) {
            continue;
        }
        s_loop_trigger_1khz = false;

        uint32_t now_ms = s_system_ticks_ms;

        /* 1. Read IMU Data (Accelerometer & Gyroscope) via SPI1 */
        if (imu_ok) {
            MPU6000_Read_All(&s_imu);
        }

        /* 2. Read RC Channels & Check 300ms Failsafe */
        RC_Get_Channels(&s_rc);
        RC_Receiver_Check_Failsafe(&s_rc, now_ms);

        /* 3. Safety Arming State Machine */
        if (!s_is_armed) {
            if (imu_ok && Check_Arming_Conditions(&s_rc, &s_attitude)) {
                s_is_armed = true;
            }
        } else {
            if (!s_rc.arm_switch || s_rc.failsafe || !imu_ok) {
                s_is_armed = false;
            }
        }

        /* 4. Read Battery Voltage for Sag Compensation */
        float vbat = ADC_Read_Vbat_Volts();

        /* 5. Execute Flight Control Stack (Filters -> Mahony -> P-PID -> Airmode Mixer) */
        Flight_Controller_Update(
            &s_rc,
            &s_imu,
            s_is_armed,
            s_rc.airmode,
            vbat,
            LOOP_DT,
            &s_motors,
            &s_attitude
        );

        /* 6. Update ESC Motor Outputs */
        if (s_is_armed) {
            Motors_Set_All_Pwm(&s_motors);
        } else {
            Motors_Disarm();
        }

        /* 6.5 Real-Time TinyML Anomaly & Fault Classifier (100 Hz inference inside 1 kHz buffer) */
        AI_Fault_Detector_Update(
            s_imu.gx, s_imu.gy,
            s_attitude.gx_filt, s_attitude.gy_filt,
            s_imu.ax, s_imu.ay, s_imu.az,
            s_attitude.err_p, s_attitude.err_q,
            &s_motors,
            s_is_armed,
            &s_ai_diag
        );

        /* 7. Blackbox Recording at 250 Hz (every 4 ms while armed) */
        blackbox_counter++;
        if (blackbox_counter >= 4U) {
            blackbox_counter = 0U;
            if (s_is_armed) {
                Blackbox_Record_Frame(now_ms, &s_attitude, &s_motors, vbat, s_is_armed, s_rc.flight_mode);
            }
        }

        /* 8. Bidirectional Telemetry Stream to MATLAB at 50 Hz + AI Frame at 10 Hz */
        telemetry_counter++;
        if (telemetry_counter >= 20U) {
            telemetry_counter = 0U;
            Telemetry_Send_Frame(&s_attitude, &s_motors, vbat, s_is_armed, s_rc.flight_mode);
        }

        ai_tlm_counter++;
        if (ai_tlm_counter >= 100U) {
            ai_tlm_counter = 0U;
            Telemetry_Send_AI_Frame(&s_attitude, &s_ai_diag);
        }

        /* 9. Status LED & Beeper Feedback (alerts on low Vbat or AI Fault) */
        led_counter++;
        if (s_is_armed) {
            Status_LED_Set(true); /* Solid ON when armed */
        } else if (!imu_ok || s_rc.failsafe) {
            if (led_counter >= 100U) { /* Fast blink (10 Hz) on error/failsafe */
                led_counter = 0U;
                led_state = !led_state;
                Status_LED_Set(led_state);
            }
        } else {
            if (led_counter >= 500U) { /* Slow heartbeat (2 Hz) when ready */
                led_counter = 0U;
                led_state = !led_state;
                Status_LED_Set(led_state);
            }
        }

        bool ai_fault_alarm = (s_is_armed && s_ai_diag.fault_class != AI_FAULT_NORMAL && s_ai_diag.health_score < 0.35f);
        Buzzer_Set(s_rc.beeper || ai_fault_alarm || (s_is_armed && vbat > 6.0f && vbat < 13.6f));
    }

    return 0;
}
