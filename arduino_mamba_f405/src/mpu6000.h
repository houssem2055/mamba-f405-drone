#ifndef MPU6000_H
#define MPU6000_H

#include <stdint.h>
#include <stdbool.h>

/* MPU6000 SPI Register Map */
#define MPU_REG_SMPLRT_DIV       0x19
#define MPU_REG_CONFIG           0x1A
#define MPU_REG_GYRO_CONFIG      0x1B
#define MPU_REG_ACCEL_CONFIG     0x1C
#define MPU_REG_INT_PIN_CFG      0x37
#define MPU_REG_INT_ENABLE       0x38
#define MPU_REG_ACCEL_XOUT_H     0x3B
#define MPU_REG_TEMP_OUT_H       0x41
#define MPU_REG_GYRO_XOUT_H      0x43
#define MPU_REG_USER_CTRL        0x6A
#define MPU_REG_PWR_MGMT_1       0x6B
#define MPU_REG_WHO_AM_I         0x75

#define MPU6000_WHO_AM_I_VAL     0x68
#define SPI_READ_FLAG            0x80

/* Sensitivity scale factors */
/* Gyro +/- 2000 deg/s: 16.4 LSB / (deg/s) */
#define GYRO_SCALE_RAD_S         (1.0f / 16.4f * (3.14159265f / 180.0f))
/* Accel +/- 8g: 4096 LSB / g -> 1 g = 9.80665 m/s^2 */
#define ACCEL_SCALE_M_S2         (9.80665f / 4096.0f)

typedef struct {
    float ax, ay, az;      /* Accelerations in m/s^2 */
    float gx, gy, gz;      /* Angular rates in rad/s (p, q, r) */
    float temperature;     /* Board temperature in degrees C */
    
    /* Calibration offsets (calculated at startup) */
    float gx_bias, gy_bias, gz_bias;
    float ax_bias, ay_bias, az_bias;
    bool  is_calibrated;
} MPU6000_Data_t;

/* Public API */
bool MPU6000_Init(void);
bool MPU6000_Calibrate_Gyro(MPU6000_Data_t *imu, uint16_t sample_count);
bool MPU6000_Read_All(MPU6000_Data_t *imu);

#endif /* MPU6000_H */
