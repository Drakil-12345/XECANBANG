/**
  ******************************************************************************
  * @file    balance_control.c
  * @brief   Bo dieu khien can bang PD. Xem balance_control.h (dac biet muc DAU).
  ******************************************************************************
  */
#include "balance_control.h"

/* Gia tri mac dinh AN TOAN khi khoi tao:
 *   kp = kd = ki = 0 -> khong tac dong, chi do dac dau ra.
 *   Nguoi dung tang dan tu debugger theo quy trinh hieu chinh.            */
#define BAL_DEFAULT_KP            0.0f
#define BAL_DEFAULT_KD            0.0f
#define BAL_DEFAULT_KI            0.0f

#define BAL_DEFAULT_ANGLE_LIMIT   30.0f   /* deg: disarm neu |angle| > 30   */
#define BAL_DEFAULT_RATE_LIMIT    800.0f  /* deg/s: bat thuong -> disarm    */
#define BAL_DEFAULT_PWM_LIMIT     3000    /* gioi han PWM dau ra (an toan)  */
#define BAL_DEFAULT_INT_LIMIT     1000.0f /* gioi han integral            */
#define BAL_DEFAULT_MAX_MISSED    5u      /* >5 chu ky lo lien tiep -> fault */

void Balance_Init(Balance_t *b)
{
  b->kp = BAL_DEFAULT_KP;
  b->kd = BAL_DEFAULT_KD;
  b->ki = BAL_DEFAULT_KI;
  b->integral       = 0.0f;
  b->integral_limit = BAL_DEFAULT_INT_LIMIT;

  b->angle_sign = 1.0f;
  b->gyro_sign  = 1.0f;
  b->out_sign   = 1.0f;

  b->target_angle    = 0.0f;
  b->angle_limit_deg = BAL_DEFAULT_ANGLE_LIMIT;
  b->rate_limit_dps  = BAL_DEFAULT_RATE_LIMIT;
  b->pwm_limit       = BAL_DEFAULT_PWM_LIMIT;

  b->control_out = 0.0f;
  b->angle_now   = 0.0f;
  b->rate_now    = 0.0f;

  b->armed         = false;
  b->status        = BAL_FAULT_NOT_ARMED;
  b->run_cycles    = 0;
  b->missed_cycles = 0;
  b->max_missed    = BAL_DEFAULT_MAX_MISSED;
}

void Balance_Arm(Balance_t *b)
{
  b->integral = 0.0f;
  b->status   = BAL_OK;
  b->armed    = true;
}

void Balance_Disarm(Balance_t *b)
{
  b->armed    = false;
  b->integral = 0.0f;
  if (b->status == BAL_OK) b->status = BAL_FAULT_NOT_ARMED;
}

void Balance_Reset(Balance_t *b)
{
  b->integral    = 0.0f;
  b->control_out = 0.0f;
}

Balance_Status_t Balance_Update(Balance_t *b, MPU6050_Data_t *imu, Motor_Handle_t *mot)
{
  /* ---- Kiem tra an toan truoc ---- */
  /* IMU phai: connected + calibrated + KHONG dang o trong chuoi loi lon.
   * Cho phep err_count nho (loi le do bus margin) — chi cat motor khi loi
   * lien tiep nghiem trong (>=3 mau), de tranh cat motor do nhay ao. */
  if (!imu->connected || !imu->calibrated || imu->err_count >= 3u) {
    Balance_Disarm(b);
    b->status = BAL_FAULT_IMU;
    Motor_Disable(mot);
    return b->status;
  }

  /* Goc va van toc goc da chuan hoa dau */
  float angle = imu->angle      * b->angle_sign;
  float rate  = imu->gyro_x_dps * b->gyro_sign;

  b->angle_now = angle;
  b->rate_now  = rate;

  /* Nguong goc an toan */
  if (angle > b->angle_limit_deg || angle < -b->angle_limit_deg) {
    Balance_Disarm(b);
    b->status = BAL_FAULT_ANGLE;
    Motor_Disable(mot);
    return b->status;
  }

  /* Van toc goc bat thuong (runaway) */
  if (rate > b->rate_limit_dps || rate < -b->rate_limit_dps) {
    Balance_Disarm(b);
    b->status = BAL_FAULT_RATE;
    Motor_Disable(mot);
    return b->status;
  }

  /* ---- Tinh dieu khien (luon tinh de quan sat, chi xuat khi armed) ---- */
  float error = b->target_angle - angle;

  /* PID don gian: P tren goc, D tren van toc goc (dung truc tiep gyro),
   * I tich luy error (co nhan dt de dung thu nguyen). Kd/Ki dat 0 neu
   * chua can thiet. */
  b->integral += error * 0.004f;   /* dt = chu ky dieu khien 4ms */
  if (b->integral >  b->integral_limit) b->integral =  b->integral_limit;
  if (b->integral < -b->integral_limit) b->integral = -b->integral_limit;

  float out = b->kp * error
            - b->kd * rate          /* triet tieu van toc goc (damping)   */
            + b->ki * b->integral;

  out *= b->out_sign;

  /* Gioi han dau ra theo ca pwm_limit cua balance va pwm_max cua driver */
  int16_t effective_limit = b->pwm_limit;
  if (effective_limit < 0) effective_limit = 0;
  if (effective_limit > (int16_t)mot->pwm_max) {
    effective_limit = (int16_t)mot->pwm_max;
  }
  if (out >  (float)effective_limit) out =  (float)effective_limit;
  if (out < -(float)effective_limit) out = -(float)effective_limit;

  b->control_out = out;
  b->run_cycles++;

  /* ---- Xuat motor ---- */
  if (b->armed) {
    b->status = BAL_OK;
    /* Xe 2 banh: ca hai motor nhan cung lenh de chuyen dong thang.
     * Neu co trim trai/phai thi them offset o day.                      */
    int16_t cmd = (int16_t)(out >= 0 ? out + 0.5f : out - 0.5f); /* round */
    Motor_SetBoth(mot, cmd, cmd);
  } else {
    /* Chua arm -> giu motor dung (nhung khong tat STBY de tranh giat) */
    Motor_Stop(mot);
  }

  return b->status;
}

void Balance_NotifyMissed(Balance_t *b, uint32_t missed, Motor_Handle_t *mot)
{
  b->missed_cycles += missed;
  if (b->missed_cycles > b->max_missed) {
    /* Nhieu chu ky bi lo lien tiep -> khong an toan de tiep tuc dieu khien */
    Balance_Disarm(b);
    b->status = BAL_FAULT_TIMING;
    Motor_Disable(mot);
  }
}
