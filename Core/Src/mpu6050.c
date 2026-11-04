/**
  ******************************************************************************
  * @file    mpu6050.c
  * @brief   Driver MPU6050 qua I2C2 + Complementary Filter goc nghieng.
  *          THEO DUNG MO HINH CODE DA CHUNG MINH CHAY DUOC.
  ******************************************************************************
  */
#include "mpu6050.h"
#include <math.h>
#include <string.h>

#define RAD2DEG  (180.0f / 3.14159265358979f)
#define MPU6050_ADDR8   (0x68 << 1)   /* dia chi 8-bit nhu code chay duoc */

static I2C_HandleTypeDef *s_i2c = NULL;
static uint32_t s_last_hal_err = 0;

/* ---------------------------------------------------------------------------*/
bool MPU6050_Init(I2C_HandleTypeDef *hi2c, MPU6050_Data_t *d)
{
  s_i2c = hi2c;
  memset(d, 0, sizeof(*d));
  d->connected  = false;
  d->valid      = false;
  d->calibrated = false;
  d->whoami_ok  = false;

  uint8_t cfg;
  HAL_StatusTypeDef st = HAL_OK;

  /* Giong code chay duoc: IsDeviceReady -> doc WHO_AM_I -> ghi config */
  if (HAL_I2C_IsDeviceReady(s_i2c, MPU6050_ADDR8, 3, 50) != HAL_OK) {
    d->last_hal_err = (uint32_t)HAL_I2C_GetError(s_i2c);
    return false;
  }

  st |= HAL_I2C_Mem_Read(s_i2c, MPU6050_ADDR8, MPU6050_REG_WHO_AM_I,
                         I2C_MEMADD_SIZE_8BIT, &d->who_am_i, 1, 50);
  d->whoami_ok = (d->who_am_i == MPU6050_WHOAMI_EXPECTED);
  if (!d->whoami_ok) { d->last_hal_err = s_last_hal_err; return false; }

  cfg = 0x00; /* Exit sleep */
  st |= HAL_I2C_Mem_Write(s_i2c, MPU6050_ADDR8, MPU6050_REG_PWR_MGMT_1,
                          I2C_MEMADD_SIZE_8BIT, &cfg, 1, 50);
  cfg = 0x08; /* Gyro +-500 dps */
  st |= HAL_I2C_Mem_Write(s_i2c, MPU6050_ADDR8, MPU6050_REG_GYRO_CONFIG,
                          I2C_MEMADD_SIZE_8BIT, &cfg, 1, 50);
  cfg = 0x00; /* Accel +-2g -> /16384 */
  st |= HAL_I2C_Mem_Write(s_i2c, MPU6050_ADDR8, MPU6050_REG_ACCEL_CONFIG,
                          I2C_MEMADD_SIZE_8BIT, &cfg, 1, 50);
  cfg = 0x03; /* DLPF ~44Hz */
  st |= HAL_I2C_Mem_Write(s_i2c, MPU6050_ADDR8, MPU6050_REG_CONFIG,
                          I2C_MEMADD_SIZE_8BIT, &cfg, 1, 50);

  if (st != HAL_OK) { d->last_hal_err = s_last_hal_err; return false; }

  d->connected = true;
  d->valid     = true;
  d->last_hal_err = s_last_hal_err;
  return true;
}

/* ---------------------------------------------------------------------------*/
/* Hieu chinh gyro khi dung yen — giong code chay duoc: doc gyro raw,          */
/* kiem tra dung yen (stdv nho), retry toi da 5 lan.                            */
/* ---------------------------------------------------------------------------*/
bool MPU6050_Calibrate(MPU6050_Data_t *d, uint16_t samples, void (*delay_ms)(uint32_t))
{
  (void)samples;
  for (int attempt = 0; attempt < 5; attempt++) {
    float sum = 0.0f, sumsq = 0.0f;
    int n = 0;
    uint8_t b[6];

    for (int i = 0; i < 200; i++) {
      if (HAL_I2C_Mem_Read(s_i2c, MPU6050_ADDR8, 0x43 /* GYRO_XOUT_H */,
                           I2C_MEMADD_SIZE_8BIT, b, 6, 50) == HAL_OK) {
        float g = (int16_t)((b[0] << 8) | b[1]) / MPU6050_GYRO_LSB_PER_DPS;
        sum += g; sumsq += g * g;
        n++;
      }
      if (delay_ms) delay_ms(10);
    }
    if (n < 180) continue;

    float mean = sum / (float)n;
    float var  = sumsq / (float)n - mean * mean;
    float stdv = sqrtf(var > 0.0f ? var : 0.0f);
    if (stdv < 1.5f && fabsf(mean) < 20.0f) {
      d->gyro_x_bias  = mean;
      d->accel_y_bias = 0.0f;
      d->angle_offset_deg = 0.0f;
      d->angle_acc_raw    = 0.0f;
      d->angle_acc        = 0.0f;
      d->angle            = 0.0f;
      d->calibrated       = true;
      d->valid            = true;
      return true;
    }
  }
  return false;
}

/* ---------------------------------------------------------------------------*/
/* Doc 14 byte + complementary filter — THEO CODE CHAY DUOC.                   */
/* pitch_acc = atan2f(-Ay, Az). Goc duoc hieu chinh chi khi |a| gan 1g.        */
/* ---------------------------------------------------------------------------*/
bool MPU6050_Update(MPU6050_Data_t *d, float dt)
{
  uint8_t buf[14];

  if (HAL_I2C_Mem_Read(s_i2c, MPU6050_ADDR8, MPU6050_REG_ACCEL_XOUT_H,
                       I2C_MEMADD_SIZE_8BIT, buf, 14, 20) != HAL_OK) {
    s_last_hal_err = (uint32_t)HAL_I2C_GetError(s_i2c);
    d->last_hal_err = s_last_hal_err;
    d->err_count++;
    d->total_err++;
    d->valid = false;
    if (d->err_count >= MPU6050_MAX_ERR_RETRY) {
      d->connected = false;
    }
    return false;
  }

  d->connected = true;
  d->err_count = 0;

  d->ax_raw = (int16_t)((buf[0]  << 8) | buf[1]);
  d->ay_raw = (int16_t)((buf[2]  << 8) | buf[3]);
  d->az_raw = (int16_t)((buf[4]  << 8) | buf[5]);
  d->gx_raw = (int16_t)((buf[8]  << 8) | buf[9]);
  d->gy_raw = (int16_t)((buf[10] << 8) | buf[11]);
  d->gz_raw = (int16_t)((buf[12] << 8) | buf[13]);

  d->ax_g = d->ax_raw / MPU6050_ACCEL_LSB_PER_G;
  d->ay_g = d->ay_raw / MPU6050_ACCEL_LSB_PER_G;
  d->az_g = d->az_raw / MPU6050_ACCEL_LSB_PER_G;

  d->gx_dps = (d->gx_raw / MPU6050_GYRO_LSB_PER_DPS) - d->gyro_x_bias;
  d->gyro_x_dps = d->gx_dps;

  /* Cong thuc goc da chung minh: atan2(-Ay, Az) */
  d->angle_acc_raw = atan2f(-d->ay_g, d->az_g) * RAD2DEG;
  d->angle_acc     = d->angle_acc_raw;

  /* Complementary: 0.98*gyro + 0.02*accel, chi khi |a| gan 1g */
  d->angle += d->gx_dps * dt;
  float a_norm = sqrtf(d->ax_g*d->ax_g + d->ay_g*d->ay_g + d->az_g*d->az_g);
  if (fabsf(a_norm - 1.0f) < 0.25f) {
    d->angle += 0.02f * (d->angle_acc - d->angle);
  }

  d->valid = true;
  return true;
}

/* ---------------------------------------------------------------------------*/
void MPU6050_ResetAngle(MPU6050_Data_t *d)
{
  d->angle         = 0.0f;
  d->angle_acc_raw = 0.0f;
  d->angle_acc     = 0.0f;
}

/* Bus recovery — phat SCL pha lockup (chi goi tu main loop, khong trong chu ky) */
bool MPU6050_BusRecover(MPU6050_Data_t *d)
{
  GPIO_InitTypeDef gi = {0};
  d->recoveries++;

  HAL_I2C_DeInit(s_i2c);
  gi.Pin = GPIO_PIN_10; gi.Mode = GPIO_MODE_OUTPUT_OD; gi.Pull = GPIO_NOPULL; gi.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &gi);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET); HAL_Delay(1);
  for (int i = 0; i < 16; i++) {
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11) == GPIO_PIN_SET) break;
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET); HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);   HAL_Delay(1);
  }
  gi.Pin = GPIO_PIN_11; gi.Mode = GPIO_MODE_OUTPUT_OD;
  HAL_GPIO_Init(GPIOB, &gi);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET); HAL_Delay(1);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);   HAL_Delay(1);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);   HAL_Delay(1);
  gi.Pin = GPIO_PIN_10 | GPIO_PIN_11; gi.Mode = GPIO_MODE_AF_OD;
  gi.Pull = GPIO_NOPULL; gi.Speed = GPIO_SPEED_FREQ_VERY_HIGH; gi.Alternate = GPIO_AF4_I2C2;
  HAL_GPIO_Init(GPIOB, &gi);
  if (HAL_I2C_Init(s_i2c) != HAL_OK) return false;
  HAL_Delay(5);
  return (HAL_I2C_IsDeviceReady(s_i2c, MPU6050_ADDR8, 3, 50) == HAL_OK);
}

bool MPU6050_Reconnect(MPU6050_Data_t *d)
{
  MPU6050_BusRecover(d);
  HAL_Delay(20);
  if (!MPU6050_Init(s_i2c, d)) return false;
  (void)MPU6050_Calibrate(d, MPU6050_CALIB_SAMPLES, HAL_Delay);
  return true;
}
