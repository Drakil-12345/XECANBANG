#include "mpu6050.h"
#include <stddef.h>
#include <math.h>
#include <string.h>

#define MPU_REG_SMPLRT_DIV       0x19U
#define MPU_REG_CONFIG           0x1AU
#define MPU_REG_GYRO_CONFIG      0x1BU
#define MPU_REG_ACCEL_CONFIG     0x1CU
#define MPU_REG_ACCEL_XOUT_H     0x3BU
#define MPU_REG_GYRO_XOUT_H      0x43U
#define MPU_REG_WHO_AM_I         0x75U
#define MPU_REG_PWR_MGMT_1       0x6BU
#define MPU_WHO_AM_I_VALUE       0x68U

#define MPU_I2C_TIMEOUT_MS       3U
#define MPU_RAD_TO_DEG           57.29577951308232f
#define MPU_COMPLEMENTARY_ALPHA  0.005f    /* tau ~= 0.80 s at 250 Hz: rejects linear accel */
#define MPU_ACCEL_NORM_MIN_SQ    0.5625f   /* (0.75 g)^2: allows clone accel Z offset */
#define MPU_ACCEL_NORM_MAX_SQ    1.5625f   /* (1.25 g)^2 */
#define MPU_MAX_CONSEC_ERRORS    3U

/*
 * angle_acc_raw = atan2(-ay, az) is a pitch angle in the sensor frame.
 * With a right-handed MPU6050 axis system, the derivative of that angle is
 * -gyro_x, NOT +gyro_x.  The complementary filter and the PD controller must
 * use the same sign, otherwise the D term becomes positive feedback.
 * Keep all pitch-rate consumers using gyro_x_dps (already sign-corrected).
 */
#define MPU_PITCH_RATE_SIGN      (-1.0f)

static I2C_HandleTypeDef *s_hi2c = NULL;

static bool write_register(uint8_t reg, uint8_t value)
{
    return (HAL_I2C_Mem_Write(s_hi2c, MPU6050_I2C_ADDR8, reg,
                              I2C_MEMADD_SIZE_8BIT, &value, 1U,
                              MPU_I2C_TIMEOUT_MS) == HAL_OK);
}

bool MPU6050_Init(I2C_HandleTypeDef *hi2c, MPU6050_Data_t *dev)
{
    uint8_t who = 0U;

    if ((hi2c == NULL) || (dev == NULL)) return false;

    s_hi2c = hi2c;
    memset(dev, 0, sizeof(*dev));

    if (HAL_I2C_IsDeviceReady(s_hi2c, MPU6050_I2C_ADDR8, 3U, 50U) != HAL_OK)
    {
        dev->last_hal_err = HAL_I2C_GetError(s_hi2c);
        return false;
    }

    if (HAL_I2C_Mem_Read(s_hi2c, MPU6050_I2C_ADDR8, MPU_REG_WHO_AM_I,
                         I2C_MEMADD_SIZE_8BIT, &who, 1U, 50U) != HAL_OK)
    {
        dev->last_hal_err = HAL_I2C_GetError(s_hi2c);
        return false;
    }

    dev->who_am_i = who;
    dev->whoami_ok = (who == MPU_WHO_AM_I_VALUE);
    if (!dev->whoami_ok) return false;

    /* Wake up, gyro +/-500 dps, accel +/-2 g, DLPF ~44 Hz.
     * Internal sample rate 1 kHz; the control loop reads it at 250 Hz, so
     * the freshest sample is only ~1 ms old (SMPLRT_DIV=3 would add up to
     * 4 ms of latency because the sensor would update at exactly the read
     * rate). */
    if (!write_register(MPU_REG_PWR_MGMT_1, 0x00U) ||
        !write_register(MPU_REG_GYRO_CONFIG, 0x08U) ||
        !write_register(MPU_REG_ACCEL_CONFIG, 0x00U) ||
        !write_register(MPU_REG_CONFIG, 0x03U) ||
        !write_register(MPU_REG_SMPLRT_DIV, 0x00U))
    {
        dev->last_hal_err = HAL_I2C_GetError(s_hi2c);
        return false;
    }

    dev->connected = true;
    dev->valid = false;
    dev->calibrated = false;
    dev->angle_initialized = false;
    dev->last_hal_err = HAL_OK;
    return true;
}

bool MPU6050_Calibrate(MPU6050_Data_t *dev,
                       uint16_t samples,
                       void (*delay_ms)(uint32_t))
{
    uint16_t count;
    float sum = 0.0f;
    float sum_sq = 0.0f;
    uint16_t valid_samples = 0U;
    uint8_t data[6];

    if ((dev == NULL) || (s_hi2c == NULL) || !dev->connected) return false;

    count = (samples > 0U) ? samples : MPU6050_CALIB_SAMPLES;

    for (uint16_t i = 0U; i < count; i++)
    {
        if (HAL_I2C_Mem_Read(s_hi2c, MPU6050_I2C_ADDR8, MPU_REG_GYRO_XOUT_H,
                             I2C_MEMADD_SIZE_8BIT, data, sizeof(data),
                             20U) == HAL_OK)
        {
            int16_t raw_gx = (int16_t)(((uint16_t)data[0] << 8U) | data[1]);
            float gx_dps = (float)raw_gx / MPU6050_GYRO_LSB_PER_DPS;
            sum += gx_dps;
            sum_sq += gx_dps * gx_dps;
            valid_samples++;
        }

        if (delay_ms != NULL) delay_ms(5U);
    }

    if (valid_samples == 0U)
    {
        dev->gyro_x_bias = 0.0f;
        dev->calibrated = true;
        dev->valid = false;
        dev->angle_initialized = false;
        return false;
    }

    {
        float mean = sum / (float)valid_samples;
        float variance = (sum_sq / (float)valid_samples) - (mean * mean);
        float std_dev = sqrtf((variance > 0.0f) ? variance : 0.0f);

        /* Use measured bias if reasonably still; otherwise fall back to 0 so power-on never blocks. */
        if ((std_dev < 5.0f) && (fabsf(mean) < 20.0f))
        {
            dev->gyro_x_bias = mean;
        }
        else
        {
            dev->gyro_x_bias = 0.0f;
        }
    }

    dev->calibrated = true;
    dev->valid = false;
    dev->angle_initialized = false;
    dev->err_count = 0U;
    return true;
}

bool MPU6050_Update(MPU6050_Data_t *dev, float dt_s)
{
    uint8_t data[14];
    float accel_norm_sq;

    if ((dev == NULL) || (s_hi2c == NULL) || !isfinite(dt_s) ||
        (dt_s < 0.001f) || (dt_s > 0.020f))
    {
        return false;
    }

    if (HAL_I2C_Mem_Read(s_hi2c, MPU6050_I2C_ADDR8, MPU_REG_ACCEL_XOUT_H,
                         I2C_MEMADD_SIZE_8BIT, data, sizeof(data),
                         MPU_I2C_TIMEOUT_MS) != HAL_OK)
    {
        dev->last_hal_err = HAL_I2C_GetError(s_hi2c);
        if (dev->err_count < 255U) dev->err_count++;
        dev->total_err++;
        dev->valid = false;
        if (dev->err_count >= MPU_MAX_CONSEC_ERRORS) dev->connected = false;
        return false;
    }

    dev->connected = true;
    dev->err_count = 0U;

    dev->ax_raw = (int16_t)(((uint16_t)data[0] << 8U) | data[1]);
    dev->ay_raw = (int16_t)(((uint16_t)data[2] << 8U) | data[3]);
    dev->az_raw = (int16_t)(((uint16_t)data[4] << 8U) | data[5]);
    dev->temp_raw = (int16_t)(((uint16_t)data[6] << 8U) | data[7]);
    dev->gx_raw = (int16_t)(((uint16_t)data[8] << 8U) | data[9]);
    dev->gy_raw = (int16_t)(((uint16_t)data[10] << 8U) | data[11]);
    dev->gz_raw = (int16_t)(((uint16_t)data[12] << 8U) | data[13]);

    dev->ax_g = (float)dev->ax_raw / MPU6050_ACCEL_LSB_PER_G;
    dev->ay_g = (float)dev->ay_raw / MPU6050_ACCEL_LSB_PER_G;
    dev->az_g = (float)dev->az_raw / MPU6050_ACCEL_LSB_PER_G;

    /* Raw chip X rate (after bias). Kept for diagnostics. */
    dev->gx_dps = ((float)dev->gx_raw / MPU6050_GYRO_LSB_PER_DPS) - dev->gyro_x_bias;

    /* Pitch rate that matches the sign of angle_acc_raw (see MPU_PITCH_RATE_SIGN). */
    dev->gyro_x_dps = MPU_PITCH_RATE_SIGN * dev->gx_dps;

    /* This axis pairing assumes MPU6050 X is parallel to the wheel axle. */
    dev->angle_acc_raw = atan2f(-dev->ay_g, dev->az_g) * MPU_RAD_TO_DEG;
    dev->angle_acc = dev->angle_acc_raw;

    accel_norm_sq = (dev->ax_g * dev->ax_g) +
                    (dev->ay_g * dev->ay_g) +
                    (dev->az_g * dev->az_g);

    if (!dev->angle_initialized)
    {
        /* Seed the complementary filter from gravity, not an assumed zero angle. */
        dev->angle = dev->angle_acc_raw;
        dev->angle_initialized = true;
    }
    else
    {
        /* Integrate the pitch rate, not the raw chip rate.  Both must have
           the same sign as angle_acc_raw or the filter fights itself. */
        dev->angle += dev->gyro_x_dps * dt_s;

        /* Correct gyro drift only when acceleration magnitude is close to 1 g. */
        if ((accel_norm_sq >= MPU_ACCEL_NORM_MIN_SQ) &&
            (accel_norm_sq <= MPU_ACCEL_NORM_MAX_SQ))
        {
            dev->angle += MPU_COMPLEMENTARY_ALPHA *
                          (dev->angle_acc_raw - dev->angle);
        }
    }

    dev->valid = true;
    dev->last_hal_err = HAL_OK;
    return true;
}

void MPU6050_ResetAngle(MPU6050_Data_t *dev)
{
    if (dev == NULL) return;
    dev->angle = 0.0f;
    dev->angle_acc_raw = 0.0f;
    dev->angle_acc = 0.0f;
    dev->angle_initialized = false;
}
