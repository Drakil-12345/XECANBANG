#ifndef MPU6050_H
#define MPU6050_H

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

#ifndef MPU6050_I2C_ADDR8
#define MPU6050_I2C_ADDR8          (0x68U << 1U) /* HAL expects 8-bit shifted address */
#endif

#ifndef MPU6050_ACCEL_LSB_PER_G
#define MPU6050_ACCEL_LSB_PER_G    16384.0f     /* +/-2 g */
#endif

#ifndef MPU6050_GYRO_LSB_PER_DPS
#define MPU6050_GYRO_LSB_PER_DPS   65.5f        /* +/-500 deg/s */
#endif

#ifndef MPU6050_CALIB_SAMPLES
#define MPU6050_CALIB_SAMPLES      300U
#endif

typedef struct
{
    bool connected;
    bool valid;
    bool calibrated;
    bool angle_initialized;

    uint8_t who_am_i;
    bool whoami_ok;
    uint8_t err_count;
    uint32_t total_err;
    HAL_StatusTypeDef last_hal_err;

    int16_t ax_raw;
    int16_t ay_raw;
    int16_t az_raw;
    int16_t temp_raw;
    int16_t gx_raw;
    int16_t gy_raw;
    int16_t gz_raw;

    float ax_g;
    float ay_g;
    float az_g;
    float gx_dps;          /* raw chip X rate after bias, diagnostics only */
    float gyro_x_dps;      /* pitch rate: d(angle)/dt, same sign as angle */
    float gyro_x_bias;     /* stored in deg/s (chip X, before pitch-sign conversion) */

    float angle_acc_raw;
    float angle_acc;
    float angle;
} MPU6050_Data_t;

bool MPU6050_Init(I2C_HandleTypeDef *hi2c, MPU6050_Data_t *dev);
bool MPU6050_Calibrate(MPU6050_Data_t *dev,
                       uint16_t samples,
                       void (*delay_ms)(uint32_t));
bool MPU6050_Update(MPU6050_Data_t *dev, float dt_s);
void MPU6050_ResetAngle(MPU6050_Data_t *dev);

#endif /* MPU6050_H */
