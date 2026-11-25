#include "mpu6000.h"
#include "config.h"

/* Platform abstraction wrappers for SPI transactions */
extern void SPI1_Select(bool select);
extern uint8_t SPI1_TransferByte(uint8_t byte);
extern void SPI1_TransferBuffer(const uint8_t *tx_buf, uint8_t *rx_buf, uint16_t length);
extern void Delay_Ms(uint32_t ms);

static uint8_t MPU_ReadReg(uint8_t reg) {
    uint8_t val;
    SPI1_Select(true);
    SPI1_TransferByte(reg | SPI_READ_FLAG);
    val = SPI1_TransferByte(0x00);
    SPI1_Select(false);
    return val;
}

static void MPU_WriteReg(uint8_t reg, uint8_t val) {
    SPI1_Select(true);
    SPI1_TransferByte(reg & ~SPI_READ_FLAG);
    SPI1_TransferByte(val);
    SPI1_Select(false);
}

bool MPU6000_Init(void) {
    /* 1. Reset device */
    MPU_WriteReg(MPU_REG_PWR_MGMT_1, 0x80);
    Delay_Ms(100);

    /* 2. Check Device ID (WHO_AM_I) */
    uint8_t who_am_i = MPU_ReadReg(MPU_REG_WHO_AM_I);
    if (who_am_i != MPU6000_WHO_AM_I_VAL) {
        return false; /* Failed to detect MPU6000 */
    }

    /* 3. Wake up device, select Auto PLL clock source (Gyro X reference) */
    MPU_WriteReg(MPU_REG_PWR_MGMT_1, 0x01);
    Delay_Ms(10);

    /* 4. Disable I2C interface to lock SPI mode */
    MPU_WriteReg(MPU_REG_USER_CTRL, 0x10);

    /* 5. Set DLPF (Digital Low Pass Filter) to 42 Hz bandwidth */
    MPU_WriteReg(MPU_REG_CONFIG, 0x03);

    /* 6. Sample Rate Divider: 1 kHz / (1 + 0) = 1 kHz */
    MPU_WriteReg(MPU_REG_SMPLRT_DIV, 0x00);

    /* 7. Gyro Full Scale Range: +/- 2000 deg/s (FS_SEL = 3 -> 0x18) */
    MPU_WriteReg(MPU_REG_GYRO_CONFIG, 0x18);

    /* 8. Accelerometer Full Scale Range: +/- 8g (AFS_SEL = 2 -> 0x10) */
    MPU_WriteReg(MPU_REG_ACCEL_CONFIG, 0x10);

    return true;
}

bool MPU6000_Calibrate_Gyro(MPU6000_Data_t *imu, uint16_t sample_count) {
    if (!imu || sample_count == 0) return false;

    float sum_gx = 0.0f, sum_gy = 0.0f, sum_gz = 0.0f;
    for (uint16_t i = 0; i < sample_count; i++) {
        if (!MPU6000_Read_All(imu)) {
            return false;
        }
        sum_gx += imu->gx;
        sum_gy += imu->gy;
        sum_gz += imu->gz;
        Delay_Ms(2);
    }

    imu->gx_bias = sum_gx / (float)sample_count;
    imu->gy_bias = sum_gy / (float)sample_count;
    imu->gz_bias = sum_gz / (float)sample_count;
    imu->is_calibrated = true;
    return true;
}

bool MPU6000_Read_All(MPU6000_Data_t *imu) {
    if (!imu) return false;

    uint8_t tx_buf[15] = {0};
    uint8_t rx_buf[15] = {0};
    tx_buf[0] = MPU_REG_ACCEL_XOUT_H | SPI_READ_FLAG;

    SPI1_Select(true);
    SPI1_TransferBuffer(tx_buf, rx_buf, 15);
    SPI1_Select(false);

    /* Extract 16-bit raw signed values (Big Endian) */
    int16_t raw_ax = (int16_t)((rx_buf[1]  << 8) | rx_buf[2]);
    int16_t raw_ay = (int16_t)((rx_buf[3]  << 8) | rx_buf[4]);
    int16_t raw_az = (int16_t)((rx_buf[5]  << 8) | rx_buf[6]);
    int16_t raw_tp = (int16_t)((rx_buf[7]  << 8) | rx_buf[8]);
    int16_t raw_gx = (int16_t)((rx_buf[9]  << 8) | rx_buf[10]);
    int16_t raw_gy = (int16_t)((rx_buf[11] << 8) | rx_buf[12]);
    int16_t raw_gz = (int16_t)((rx_buf[13] << 8) | rx_buf[14]);

    /* Convert to physical units */
    imu->ax = (float)raw_ax * ACCEL_SCALE_M_S2;
    imu->ay = (float)raw_ay * ACCEL_SCALE_M_S2;
    imu->az = (float)raw_az * ACCEL_SCALE_M_S2;

    float raw_rate_x = (float)raw_gx * GYRO_SCALE_RAD_S;
    float raw_rate_y = (float)raw_gy * GYRO_SCALE_RAD_S;
    float raw_rate_z = (float)raw_gz * GYRO_SCALE_RAD_S;

    if (imu->is_calibrated) {
        imu->gx = raw_rate_x - imu->gx_bias;
        imu->gy = raw_rate_y - imu->gy_bias;
        imu->gz = raw_rate_z - imu->gz_bias;
    } else {
        imu->gx = raw_rate_x;
        imu->gy = raw_rate_y;
        imu->gz = raw_rate_z;
    }

    imu->temperature = ((float)raw_tp / 340.0f) + 36.53f;
    return true;
}
