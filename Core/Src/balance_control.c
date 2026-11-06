/**
  ******************************************************************************
  * @file    balance_control.c
  * @brief   Bo dieu khien can bang — cascade: vi tri -> toc do -> goc -> PWM.
  ******************************************************************************
  */
#include "balance_control.h"

#define BAL_DEFAULT_KP            35.0f
#define BAL_DEFAULT_KD            25.0f
#define BAL_DEFAULT_KI            0.0f
#define BAL_DEFAULT_ANGLE_LIMIT   50.0f
#define BAL_DEFAULT_RATE_LIMIT    800.0f
#define BAL_DEFAULT_PWM_LIMIT     2500
#define BAL_DEFAULT_INT_LIMIT     1000.0f
#define BAL_DEFAULT_MAX_MISSED    5u

#define BAL_DEFAULT_SPD_KP        0.05f
#define BAL_DEFAULT_SPD_KI        0.0f
#define BAL_DEFAULT_SPD_MAX_DEG   10.0f

#define BAL_DEFAULT_POS_KP        0.005f
#define BAL_DEFAULT_POS_KI        0.0f
#define BAL_DEFAULT_POS_MAX_DEG   4.0f

void Balance_Init(Balance_t *b)
{
  b->kp = BAL_DEFAULT_KP;
  b->kd = BAL_DEFAULT_KD;
  b->ki = BAL_DEFAULT_KI;
  b->integral       = 0.0f;
  b->integral_limit = BAL_DEFAULT_INT_LIMIT;

  b->angle_sign = 1.0f;
  b->gyro_sign  = 1.0f;
  b->out_sign   = -1.0f;   /* dao dau: nghieng toi -> banh quay TOI de cuu */

  b->target_angle    = 0.0f;
  b->angle_limit_deg = BAL_DEFAULT_ANGLE_LIMIT;
  b->rate_limit_dps  = BAL_DEFAULT_RATE_LIMIT;
  b->pwm_limit       = BAL_DEFAULT_PWM_LIMIT;
  b->deadband        = 80.0f;

  b->control_out = 0.0f;
  b->angle_now   = 0.0f;
  b->rate_now    = 0.0f;

  b->armed         = false;
  b->status        = BAL_FAULT_NOT_ARMED;
  b->run_cycles    = 0;
  b->missed_cycles = 0;
  b->max_missed    = BAL_DEFAULT_MAX_MISSED;

  b->spd_f       = 0.0f;
  b->spd_int     = 0.0f;
  b->spd_kp      = BAL_DEFAULT_SPD_KP;
  b->spd_ki      = BAL_DEFAULT_SPD_KI;
  b->spd_max_deg = BAL_DEFAULT_SPD_MAX_DEG;
  b->spd_loop_en = false;

  b->pos         = 0.0f;
  b->pos_kp      = BAL_DEFAULT_POS_KP;
  b->pos_ki      = BAL_DEFAULT_POS_KI;
  b->pos_int     = 0.0f;
  b->pos_max_deg = BAL_DEFAULT_POS_MAX_DEG;
  b->pos_loop_en = false;

  b->tune_ok       = 0;
  b->tune_ok_cnt   = 0;
  b->max_angle_abs = 0.0f;
  b->max_rate_abs  = 0.0f;
  b->max_spd_abs   = 0.0f;
  b->ok_window     = 250u;
}

void Balance_SetWheelSpeed(Balance_t *b, float spd)
{
  b->spd_f = 0.7f * b->spd_f + 0.3f * spd;
}

void Balance_SetPosition(Balance_t *b, float pos)
{
  b->pos = pos;
}

void Balance_Arm(Balance_t *b)
{
  b->integral = 0.0f;
  b->spd_int  = 0.0f;
  b->pos_int  = 0.0f;
  b->status   = BAL_OK;
  b->armed    = true;
}

void Balance_Disarm(Balance_t *b)
{
  b->armed    = false;
  b->integral = 0.0f;
  b->spd_int  = 0.0f;
  b->pos_int  = 0.0f;
  if (b->status == BAL_OK) b->status = BAL_FAULT_NOT_ARMED;
}

void Balance_Reset(Balance_t *b)
{
  b->integral    = 0.0f;
  b->spd_int     = 0.0f;
  b->pos_int     = 0.0f;
  b->control_out = 0.0f;
}

Balance_Status_t Balance_Update(Balance_t *b, MPU6050_Data_t *imu, Motor_Handle_t *mot)
{
  /* ---- Kiem tra an toan ---- */
  if (!imu->connected || !imu->calibrated || imu->err_count >= 3u) {
    Balance_Disarm(b);
    b->status = BAL_FAULT_IMU;
    Motor_Disable(mot);
    return b->status;
  }

  float angle = imu->angle      * b->angle_sign;
  float rate  = imu->gyro_x_dps * b->gyro_sign;
  b->angle_now = angle;
  b->rate_now  = rate;

  if (angle > b->angle_limit_deg || angle < -b->angle_limit_deg) {
    Balance_Disarm(b);
    b->status = BAL_FAULT_ANGLE;
    Motor_Disable(mot);
    return b->status;
  }
  if (rate > b->rate_limit_dps || rate < -b->rate_limit_dps) {
    Balance_Disarm(b);
    b->status = BAL_FAULT_RATE;
    Motor_Disable(mot);
    return b->status;
  }

  /* ---- Vong ngoai 1: VI TRI -> keo xe ve cho cu ---- */
  float target = b->target_angle;
  if (b->pos_loop_en) {
    b->pos_int += b->pos;
    if (b->pos_int >  5000.0f) b->pos_int =  5000.0f;
    if (b->pos_int < -5000.0f) b->pos_int = -5000.0f;
    float pos_adj = b->pos_kp * b->pos + b->pos_ki * b->pos_int;
    if (pos_adj >  b->pos_max_deg) pos_adj =  b->pos_max_deg;
    if (pos_adj < -b->pos_max_deg) pos_adj = -b->pos_max_deg;
    target -= pos_adj;
  }

  /* ---- Vong ngoai 2: TOC DO -> ham troi nhanh ---- */
  if (b->spd_loop_en) {
    b->spd_int += b->spd_f * 0.004f;
    float int_lim = 1000.0f;
    if (b->spd_int >  int_lim) b->spd_int =  int_lim;
    if (b->spd_int < -int_lim) b->spd_int = -int_lim;
    float spd_adj = b->spd_kp * b->spd_f + b->spd_ki * b->spd_int;
    if (spd_adj >  b->spd_max_deg) spd_adj =  b->spd_max_deg;
    if (spd_adj < -b->spd_max_deg) spd_adj = -b->spd_max_deg;
    target -= spd_adj;
  }

  /* ---- Vong trong: PD goc ---- */
  float error = target - angle;

  b->integral += error * 0.004f;
  if (b->integral >  b->integral_limit) b->integral =  b->integral_limit;
  if (b->integral < -b->integral_limit) b->integral = -b->integral_limit;

  float out = b->kp * error - b->kd * rate + b->ki * b->integral;
  out *= b->out_sign;

  /* Bu vung chet motor */
  if (out >  0.5f) out += b->deadband;
  else if (out < -0.5f) out -= b->deadband;

  /* Gioi han dau ra */
  int16_t effective_limit = b->pwm_limit;
  if (effective_limit < 0) effective_limit = 0;
  if (effective_limit > (int16_t)mot->pwm_max) effective_limit = (int16_t)mot->pwm_max;
  if (out >  (float)effective_limit) out =  (float)effective_limit;
  if (out < -(float)effective_limit) out = -(float)effective_limit;

  b->control_out = out;
  b->run_cycles++;

  /* ---- Log ---- */
  b->log_angle   = angle;
  b->log_rate    = rate;
  b->log_control = out;
  b->log_target  = target;
  b->log_spd     = b->spd_f;
  b->log_error   = error;
  b->log_armed   = b->armed ? 1 : 0;
  b->log_status  = (uint8_t)b->status;
  b->log_tune_ok = b->tune_ok;

  /* ---- Danh gia can bang ---- */
  {
    float aa = (angle < 0) ? -angle : angle;
    float ra = (rate  < 0) ? -rate  : rate;
    float sa = (b->spd_f < 0) ? -b->spd_f : b->spd_f;
    if (aa > b->max_angle_abs) b->max_angle_abs = aa;
    if (ra > b->max_rate_abs)  b->max_rate_abs  = ra;
    if (sa > b->max_spd_abs)   b->max_spd_abs   = sa;
    if (aa < 5.0f && ra < 30.0f && sa < 100.0f) {
      if (b->tune_ok_cnt < 65000u) b->tune_ok_cnt++;
    } else {
      b->tune_ok_cnt = 0;
      b->max_angle_abs = aa;
      b->max_rate_abs  = ra;
      b->max_spd_abs   = sa;
    }
    b->tune_ok = (b->tune_ok_cnt >= 100u) ? 1 : 0;
  }

  /* ---- Xuat motor ---- */
  if (b->armed) {
    b->status = BAL_OK;
    int16_t cmd = (int16_t)(out >= 0 ? out + 0.5f : out - 0.5f);
    Motor_SetBoth(mot, cmd, cmd);
  } else {
    Motor_Stop(mot);
  }

  return b->status;
}

void Balance_NotifyMissed(Balance_t *b, uint32_t missed, Motor_Handle_t *mot)
{
  b->missed_cycles += missed;
  if (b->missed_cycles > b->max_missed) {
    Balance_Disarm(b);
    b->status = BAL_FAULT_TIMING;
    Motor_Disable(mot);
  }
}
