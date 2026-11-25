/**
 * @file    test_firmware.c
 * @brief   Comprehensive Host Unit Test & 6-DOF Software-in-the-Loop (SIL)
 *          Verification Suite for the MAMBA F405 Drone Firmware.
 *
 * Compile & run with GCC:
 *   gcc -Wall -Wextra -Werror -O2 -Ifirmware/inc firmware/src/biquad.c \
 *       firmware/src/attitude_mahony.c firmware/src/flight_controller.c \
 *       firmware/src/motors.c firmware/src/rc_receiver.c \
 *       firmware/src/telemetry.c firmware/src/blackbox.c \
 *       tests/test_firmware.c -lm -o build/test_firmware.exe
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <assert.h>

#include "config.h"
#include "biquad.h"
#include "attitude_mahony.h"
#include "flight_controller.h"
#include "motors.h"
#include "rc_receiver.h"
#include "telemetry.h"
#include "blackbox.h"
#include "mpu6000.h"

#define PI_F 3.14159265358979323846f

/* ========================================================================== */
/*                    HOST HARDWARE MOCK STUBS                                */
/* ========================================================================== */
static float g_mock_pwm[4] = {1000.0f, 1000.0f, 1000.0f, 1000.0f};
static char  g_uart_last_tx[256] = {0};
static uint8_t g_mpu_regs[128] = {0};
static uint8_t g_spi_reg_addr = 0;
static bool    g_spi_is_read  = false;
static bool    g_spi_first    = true;

void SPI1_Select(bool select) {
    if (select) {
        g_spi_first = true;
    }
}

uint8_t SPI1_TransferByte(uint8_t byte) {
    if (g_spi_first) {
        g_spi_is_read = (byte & 0x80U) != 0;
        g_spi_reg_addr = byte & 0x7FU;
        g_spi_first = false;
        return 0x00U;
    }
    if (g_spi_is_read) {
        return g_mpu_regs[g_spi_reg_addr++];
    } else {
        g_mpu_regs[g_spi_reg_addr++] = byte;
        return 0x00U;
    }
}

void SPI1_TransferBuffer(const uint8_t *tx_buf, uint8_t *rx_buf, uint16_t length) {
    for (uint16_t i = 0; i < length; i++) {
        uint8_t tx = tx_buf ? tx_buf[i] : 0x00U;
        uint8_t rx = SPI1_TransferByte(tx);
        if (rx_buf) rx_buf[i] = rx;
    }
}

void Delay_Ms(uint32_t ms) {
    (void)ms;
}

void Timer_Set_PWM_Pulse_Us(uint8_t channel, float pulse_us) {
    if (channel >= 1 && channel <= 4) {
        g_mock_pwm[channel - 1] = pulse_us;
    }
}

void UART6_SendString(const char *str) {
    if (str) {
        snprintf(g_uart_last_tx, sizeof(g_uart_last_tx), "%s", str);
    }
}

/* ========================================================================== */
/*                              UNIT TESTS                                    */
/* ========================================================================== */
static void test_biquad_filters(void) {
    printf("[TEST 1] Biquad LPF (90Hz) & Notch (220Hz)... ");

    Biquad_Filter_t lpf, notch;
    Biquad_Init(&lpf,   BIQUAD_LPF,   90.0f,  1000.0f, 0.70710678f);
    Biquad_Init(&notch, BIQUAD_NOTCH, 220.0f, 1000.0f, 2.5f);

    /* 1. Verify DC unity gain */
    float dc_out = 0.0f;
    for (int i = 0; i < 500; i++) {
        dc_out = Biquad_Apply(&lpf, 1.0f);
    }
    assert(fabsf(dc_out - 1.0f) < 1e-4f);

    /* 2. Verify 220 Hz Notch attenuation (> 20 dB = factor > 10x reduction) */
    Biquad_Reset(&notch, 0.0f);
    float peak_220hz = 0.0f;
    for (int i = 0; i < 1000; i++) {
        float x = sinf(2.0f * PI_F * 220.0f * ((float)i / 1000.0f));
        float y = Biquad_Apply(&notch, x);
        if (i > 300 && fabsf(y) > peak_220hz) {
            peak_220hz = fabsf(y);
        }
    }
    assert(peak_220hz < 0.05f); /* > 26 dB attenuation at center frequency */

    /* 3. Verify 5 Hz signal passes through Notch cleanly */
    Biquad_Reset(&notch, 0.0f);
    float peak_5hz = 0.0f;
    for (int i = 0; i < 1000; i++) {
        float x = sinf(2.0f * PI_F * 5.0f * ((float)i / 1000.0f));
        float y = Biquad_Apply(&notch, x);
        if (i > 400 && fabsf(y) > peak_5hz) {
            peak_5hz = fabsf(y);
        }
    }
    assert(peak_5hz > 0.98f);
    printf("PASS (220Hz peak=%.4f, 5Hz peak=%.4f)\n", peak_220hz, peak_5hz);
}

static void test_mahony_ahrs_and_flip(void) {
    printf("[TEST 2] Quaternion Mahony AHRS & 360-deg Flip... ");

    Mahony_AHRS_t ahrs;
    Mahony_Init(&ahrs, 2.5f, 0.05f);

    /* 1. Tilted stationary convergence test: 20 deg roll, -10 deg pitch */
    float roll_true  = 20.0f * (PI_F / 180.0f);
    float pitch_true = -10.0f * (PI_F / 180.0f);
    float ax = -9.81f * sinf(pitch_true);
    float ay =  9.81f * sinf(roll_true) * cosf(pitch_true);
    float az =  9.81f * cosf(roll_true) * cosf(pitch_true);

    float r = 0, p = 0, y = 0;
    Mahony_Reset_From_Accel(&ahrs, ax, ay, az);
    for (int i = 0; i < 1000; i++) {
        Mahony_Update(&ahrs, 0.0f, 0.0f, 0.0f, ax, ay, az, 0.001f, &r, &p, &y);
    }
    assert(fabsf(r - roll_true)  < 0.005f);
    assert(fabsf(p - pitch_true) < 0.005f);

    /* 2. Full 360-deg roll flip under zero gravity (singularity-free check) */
    Mahony_Init(&ahrs, 2.5f, 0.05f);
    Mahony_Reset_From_Accel(&ahrs, 0.0f, 0.0f, 9.81f);
    for (int i = 0; i < 1000; i++) {
        Mahony_Update(&ahrs, 2.0f * PI_F, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.001f, &r, &p, &y);
    }
    float q_norm = sqrtf(ahrs.q0*ahrs.q0 + ahrs.q1*ahrs.q1 + ahrs.q2*ahrs.q2 + ahrs.q3*ahrs.q3);
    assert(fabsf(q_norm - 1.0f) < 1e-4f);
    assert(fabsf(r) < 0.02f); /* Returned cleanly to ~0 rad after 360 deg */
    printf("PASS (q_norm=%.6f, post-flip roll=%.4f rad)\n", q_norm, r);
}

static void pack_crsf_11bit(uint8_t *payload, const uint16_t ch[8]) {
    memset(payload, 0, 22);
    uint32_t bitbuf = 0;
    int bits_in = 0;
    int byte_idx = 0;
    for (int i = 0; i < 8; i++) {
        bitbuf |= ((uint32_t)(ch[i] & 0x07FFU)) << bits_in;
        bits_in += 11;
        while (bits_in >= 8) {
            payload[byte_idx++] = (uint8_t)(bitbuf & 0xFFU);
            bitbuf >>= 8;
            bits_in -= 8;
        }
    }
    if (bits_in > 0) {
        payload[byte_idx] = (uint8_t)(bitbuf & 0xFFU);
    }
}

static void test_crsf_receiver_and_failsafe(void) {
    printf("[TEST 3] CRSF 11-bit Decoding, CRC8 & Failsafe... ");

    RC_Receiver_Init();

    /* Build authentic CRSF frame: [0xC8, len=24, type=0x16, 22-byte payload, crc8] */
    uint8_t frame[26];
    frame[0] = 0xC8U;
    frame[1] = 24U;
    frame[2] = 0x16U;

    uint16_t ch[8] = {
        992U,  /* Roll center */
        992U,  /* Pitch center */
        992U,  /* Throttle ~50% */
        992U,  /* Yaw center */
        1800U, /* AUX1: Armed */
        1800U, /* AUX2: Angle Mode */
        1500U, /* AUX3: Airmode ON */
        200U   /* AUX4: Beeper OFF */
    };
    pack_crsf_11bit(&frame[3], ch);
    frame[25] = RC_Compute_CRSF_CRC8(&frame[2], 23U);

    for (int i = 0; i < 26; i++) {
        RC_Receiver_Process_Byte(frame[i], 100U);
    }

    RC_Channels_t rc;
    RC_Get_Channels(&rc);
    assert(!rc.failsafe);
    assert(rc.arm_switch);
    assert(rc.flight_mode == 1);
    assert(rc.airmode);
    assert(!rc.beeper);
    assert(rc.valid_frames == 1U);
    assert(fabsf(rc.throttle - 0.50f) < 0.02f);

    /* Corrupt 1 bit and verify CRC8 rejects it */
    frame[10] ^= 0x01U;
    for (int i = 0; i < 26; i++) {
        RC_Receiver_Process_Byte(frame[i], 120U);
    }
    RC_Get_Channels(&rc);
    assert(rc.crc_errors == 1U);
    assert(rc.valid_frames == 1U);

    /* Trigger failsafe after 450 ms (> 300 ms timeout) */
    RC_Receiver_Check_Failsafe(&rc, 450U);
    assert(rc.failsafe);
    assert(!rc.arm_switch);
    assert(rc.throttle == 0.0f);

    RC_Channels_t rc_sync;
    RC_Get_Channels(&rc_sync);
    assert(rc_sync.failsafe); /* Must remain synchronized! */
    printf("PASS (valid=%u, crc_rej=%u, failsafe=OK)\n", rc_sync.valid_frames, rc_sync.crc_errors);
}

static void test_dshot_and_motors(void) {
    printf("[TEST 4] Motors PWM Clamping & DShot600 Framing... ");

    assert(Motors_Pwm_To_DShot_Throttle(1000.0f, false) == 0U);
    assert(Motors_Pwm_To_DShot_Throttle(2000.0f, true)  == 2047U);

    uint16_t pkt = Motors_Encode_DShot_Packet(1047U, true);
    /* Verify 11-bit throttle and telemetry bit */
    assert(((pkt >> 5) & 0x07FFU) == 1047U);
    assert(((pkt >> 4) & 0x01U) == 1U);
    /* Verify 4-bit XOR checksum */
    uint16_t upper = pkt >> 4;
    uint16_t expected_crc = (upper ^ (upper >> 4) ^ (upper >> 8)) & 0x0FU;
    assert((pkt & 0x0FU) == expected_crc);

    Motor_Outputs_t m = {900.0f, 1500.0f, 2200.0f, 1060.0f};
    Motors_Set_All_Pwm(&m);
    assert(g_mock_pwm[0] == 1000.0f);
    assert(g_mock_pwm[1] == 1500.0f);
    assert(g_mock_pwm[2] == 2000.0f);
    assert(g_mock_pwm[3] == 1060.0f);
    printf("PASS (DShot pkt=0x%04X)\n", pkt);
}

static void test_airmode_mixer_and_telemetry_cli(void) {
    printf("[TEST 5] Airmode Desaturation & Bidirectional Telemetry CLI... ");

    Flight_Controller_Init();
    Telemetry_Init();

    /* 1. Test Airmode at 0% throttle with +30 deg roll command */
    RC_Channels_t rc = {0};
    rc.throttle = 0.0f;
    rc.roll_cmd = 0.5236f; /* +30 deg */
    rc.flight_mode = FLIGHT_MODE_ANGLE;
    rc.airmode = true;

    MPU6000_Data_t imu = {0};
    imu.az = 9.81f;

    Motor_Outputs_t m = {0};
    Attitude_t att = {0};

    for (int i = 0; i < 10; i++) {
        Flight_Controller_Update(&rc, &imu, true, true, 16.0f, 0.001f, &m, &att);
    }
    /* Airmode must boost M3/M4 above IDLE_PWM to create positive roll torque even at 0% throttle */
    assert(m.m3 > MOTOR_IDLE_PWM_US + 20.0f);
    assert(m.m4 > MOTOR_IDLE_PWM_US + 20.0f);
    assert(m.m1 >= MOTOR_IDLE_PWM_US && m.m2 >= MOTOR_IDLE_PWM_US);

    /* 2. Test Telemetry CLI live PID tuning */
    const char *cmd_ping = "$CMD,PING\n";
    for (size_t i = 0; i < strlen(cmd_ping); i++) {
        Telemetry_Process_Rx_Byte((uint8_t)cmd_ping[i]);
    }
    assert(strncmp(g_uart_last_tx, "$PONG,MAMBA_F405_V2", 19) == 0);

    const char *cmd_set = "$SET,KP_P,0.185\n";
    for (size_t i = 0; i < strlen(cmd_set); i++) {
        Telemetry_Process_Rx_Byte((uint8_t)cmd_set[i]);
    }
    assert(strncmp(g_uart_last_tx, "$ACK,SET_OK", 11) == 0);

    PID_Gains_t g;
    Flight_Controller_Get_Gains(&g);
    assert(fabsf(g.Kp_p - 0.185f) < 1e-5f);

    /* Restore default Kp_p */
    g.Kp_p = 0.13f;
    Flight_Controller_Set_Gains(&g);
    printf("PASS (Airmode M3=%.1f us, CLI PID set OK)\n", m.m3);
}

static void test_blackbox_logger(void) {
    printf("[TEST 6] Binary Blackbox Ring Buffer (32B packed frames)... ");

    assert(sizeof(Blackbox_Frame_t) == 32U);
    Blackbox_Init();

    Attitude_t att = {.roll_deg = 12.34f, .pitch_deg = -5.67f, .yaw_deg = 90.0f};
    Motor_Outputs_t m = {1450.0f, 1460.0f, 1510.0f, 1520.0f};

    for (uint32_t i = 0; i < 5; i++) {
        assert(Blackbox_Record_Frame(i * 4U, &att, &m, 15.8f, true, 1U));
    }
    assert(Blackbox_Pending_Count() == 5U);

    Blackbox_Frame_t out;
    assert(Blackbox_Pop_Frame(&out));
    assert(out.magic == 0x4D424258UL);
    assert(out.roll_cdeg == 1234);
    assert(out.pitch_cdeg == -567);
    assert(out.vbat_mv == 15800);
    assert(Blackbox_Pending_Count() == 4U);
    printf("PASS (frame_size=%zu bytes)\n", sizeof(Blackbox_Frame_t));
}

static void test_mpu6000_spi_driver(void) {
    printf("[TEST 7] MPU6000 SPI Register Init, Calibration & Burst Read... ");

    memset(g_mpu_regs, 0, sizeof(g_mpu_regs));
    g_mpu_regs[MPU_REG_WHO_AM_I] = MPU6000_WHO_AM_I_VAL; /* 0x68 */

    assert(MPU6000_Init());
    assert(g_mpu_regs[MPU_REG_GYRO_CONFIG] == 0x18U);
    assert(g_mpu_regs[MPU_REG_ACCEL_CONFIG] == 0x10U);

    /* Simulate stationary +1g on Z (4096 counts = 0x1000) & bias on Gyro X (164 counts = 10 deg/s) */
    g_mpu_regs[MPU_REG_ACCEL_XOUT_H + 4] = 0x10U; /* AZ high */
    g_mpu_regs[MPU_REG_ACCEL_XOUT_H + 5] = 0x00U; /* AZ low */
    g_mpu_regs[MPU_REG_GYRO_XOUT_H + 0]  = 0x00U;
    g_mpu_regs[MPU_REG_GYRO_XOUT_H + 1]  = 164U;

    MPU6000_Data_t imu = {0};
    assert(MPU6000_Calibrate_Gyro(&imu, 50U));
    assert(imu.is_calibrated);
    assert(MPU6000_Read_All(&imu));
    assert(fabsf(imu.az - 9.80665f) < 0.01f);
    assert(fabsf(imu.gx) < 1e-4f); /* Bias removed! */
    printf("PASS (az=%.3f m/s^2, calibrated gx=%.5f rad/s)\n", imu.az, imu.gx);
}

static void test_closed_loop_sil_6dof(void) {
    printf("[TEST 8] Closed-Loop C 6-DOF SIL Simulation & CSV Export... ");

    Flight_Controller_Init();

    FILE *csv = fopen("tests/sil_c_results.csv", "w");
    assert(csv != NULL);
    fprintf(csv, "t,roll_deg,pitch_deg,yaw_deg,m1,m2,m3,m4\n");

    const float dt = 0.001f;
    const float mass = 0.650f, g = 9.81f;
    const float d = 0.125f / sqrtf(2.0f);
    const float Ixx = 4.85e-3f, Iyy = 4.85e-3f, Izz = 8.80e-3f;
    const float kt = 1.15e-6f, km = 1.90e-8f;
    const float omega_hover = sqrtf((mass * g) / (4.0f * kt));
    const float pwm_hover = 1000.0f + ((omega_hover - 200.0f) / 2000.0f) * 1000.0f;
    const float hover_thr = (pwm_hover - 1060.0f) / (2000.0f - 1060.0f);

    float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
    float p = 0.0f, qr = 0.0f, r = 0.0f;
    float rw[4] = {200.0f, 200.0f, 200.0f, 200.0f};

    float final_roll_deg = 0.0f;

    for (int k = 0; k < 5000; k++) {
        float t = (float)k * dt;

        RC_Channels_t rc = {0};
        rc.flight_mode = FLIGHT_MODE_ANGLE;
        rc.airmode = true;
        rc.throttle = (t < 0.5f) ? 0.0f : (t < 1.5f ? hover_thr + 0.10f : hover_thr + 0.02f);
        rc.roll_cmd = (t >= 2.5f) ? (15.0f * PI_F / 180.0f) : 0.0f;

        /* Rotation matrix Body -> Inertial */
        float r31 = 2.0f * (q1*q3 - q0*q2);
        float r32 = 2.0f * (q2*q3 + q0*q1);
        float r33 = 1.0f - 2.0f * (q1*q1 + q2*q2);

        /* Sensor emulation with strong 220 Hz vibration */
        float vib = 0.08f * sinf(2.0f * PI_F * 220.0f * t);
        MPU6000_Data_t imu = {0};
        imu.ax = r31 * g;
        imu.ay = r32 * g;
        imu.az = r33 * g;
        imu.gx = p  + vib;
        imu.gy = qr - 0.8f * vib;
        imu.gz = r  + 0.3f * vib;

        Motor_Outputs_t m = {0};
        Attitude_t att = {0};
        Flight_Controller_Update(&rc, &imu, true, true, 16.0f, dt, &m, &att);

        float m_arr[4] = {m.m1, m.m2, m.m3, m.m4};
        float T[4], Q[4];
        for (int i = 0; i < 4; i++) {
            float pn = (m_arr[i] - 1000.0f) / 1000.0f;
            float w_cmd = 200.0f + pn * 2000.0f;
            rw[i] += (dt / 0.020f) * (w_cmd - rw[i]);
            T[i] = kt * rw[i] * rw[i];
            Q[i] = km * rw[i] * rw[i];
        }

        float tau_x = d * (-T[0] - T[1] + T[2] + T[3]) - 0.0025f * p;
        float tau_y = d * (-T[0] + T[1] - T[2] + T[3]) - 0.0025f * qr;
        float tau_z = (-Q[0] + Q[1] + Q[2] - Q[3]) - 0.0025f * r;

        /* Euler rotational equations */
        float p_dot = (tau_x - (Izz - Iyy) * qr * r) / Ixx;
        float q_dot = (tau_y - (Ixx - Izz) * p  * r) / Iyy;
        float r_dot = (tau_z - (Iyy - Ixx) * p  * qr) / Izz;

        p  += p_dot * dt;
        qr += q_dot * dt;
        r  += r_dot * dt;

        /* Quaternion kinematics */
        float nq0 = q0 + 0.5f * dt * (-q1*p - q2*qr - q3*r);
        float nq1 = q1 + 0.5f * dt * ( q0*p + q2*r  - q3*qr);
        float nq2 = q2 + 0.5f * dt * ( q0*qr - q1*r + q3*p);
        float nq3 = q3 + 0.5f * dt * ( q0*r + q1*qr - q2*p);
        float qn = sqrtf(nq0*nq0 + nq1*nq1 + nq2*nq2 + nq3*nq3);
        q0 = nq0 / qn; q1 = nq1 / qn; q2 = nq2 / qn; q3 = nq3 / qn;

        float true_roll_deg  = atan2f(2.0f*(q0*q1 + q2*q3), 1.0f - 2.0f*(q1*q1 + q2*q2)) * (180.0f / PI_F);
        float true_pitch_deg = asinf(fmaxf(fminf(2.0f*(q0*q2 - q3*q1), 1.0f), -1.0f)) * (180.0f / PI_F);
        float true_yaw_deg   = atan2f(2.0f*(q0*q3 + q1*q2), 1.0f - 2.0f*(q2*q2 + q3*q3)) * (180.0f / PI_F);
        final_roll_deg = true_roll_deg;

        if (k % 5 == 0) {
            fprintf(csv, "%.3f,%.4f,%.4f,%.4f,%.2f,%.2f,%.2f,%.2f\n",
                t, true_roll_deg, true_pitch_deg, true_yaw_deg, m.m1, m.m2, m.m3, m.m4);
        }
    }
    fclose(csv);

    /* After 2.5s of commanding +15.0 deg roll, steady state must be within +/- 0.5 deg */
    assert(fabsf(final_roll_deg - 15.0f) < 0.5f);
    printf("PASS (5000 steps, final roll=%.3f deg vs 15.000 deg target)\n", final_roll_deg);
}

int main(void) {
    printf("============================================================\n");
    printf("  MAMBA F405 Drone Firmware Unit & SIL Verification Suite   \n");
    printf("============================================================\n");

    test_biquad_filters();
    test_mahony_ahrs_and_flip();
    test_crsf_receiver_and_failsafe();
    test_dshot_and_motors();
    test_airmode_mixer_and_telemetry_cli();
    test_blackbox_logger();
    test_mpu6000_spi_driver();
    test_closed_loop_sil_6dof();

    printf("============================================================\n");
    printf("  ALL 8 TEST SUITES PASSED! Zero warnings, zero errors.     \n");
    printf("============================================================\n");
    return 0;
}
