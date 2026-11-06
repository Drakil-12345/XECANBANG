/**
  ******************************************************************************
  * @file    balance_control.c
  * @brief   Bo dieu khien can bang PD + vong toc do banh (xe tu can bang).
  *
  * Cau truc dung chuan xe tu can bang:
  *   Vong ngoai: toc do banh -> target_angle thich ung (giup xe tu ham).
  *   Vong trong: PD tren goc nghieng.
  *
  * Thu tu: doc imu -> tinh spd_f tu encoder -> dieu khien -> gioi han + motor.
  ******************************************************************************
  */
#include "balance_control.h"

/* Gia tri mac dinh cho xe nho, motor JGA25-370 12V.
 * Kp/Kd/Spd co the chinh qua debugger; gia tri nay la diem khoi dau on dinh. */
#define BAL_DEFAULT_KP            35.0f
#define BAL_DEFAULT_KD            8.0f
#define BAL_DEFAULT_KI            0.0f
#define BAL_DEFAULT_ANGLE_LIMIT   50.0f   /* deg: disarm neu |angle| > 50   */
#define BAL_DEFAULT_RATE_LIMIT    800.0f  /* deg/s: bat thuong -> disarm    */
#define BAL_DEFAULT_PWM_LIMIT     3000    /* gioi han PWM dau ra            */
#define BAL_DEFAULT_INT_LIMIT     1000.0f
#define BAL_DEFAULT_MAX_MISSED    5u

/* Vong toc do: thong so an toan — chi dieu chinh goc muc tieu nho,
 * khong duoc de no chiem het dieu khien va lam xe dao dong.          */
#define BAL_DEFAULT_SPD_KP        0.02f   /* do / (counts/s)               */
#define BAL_DEFAULT_SPD_KI        0.0f    /* I toc do = 0 de on dinh       */
#define BAL_DEFAULT_SPD_MAX_DEG   8.0f    /* goc lech toi da do toc do     */

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
  b->deadband        = 0.0f;

  b->control_out = 0.0f;
  b->angle_now   = 0.0f;
  b->rate_now    = 0.0f;

  b->armed         = false;
  b->status        = BAL_FAULT_NOT_ARMED;
  b->run_cycles    = 0;
  b->missed_cycles = 0;
  b->max_missed    = BAL_DEFAULT_MAX_MISSED;

  /* Vong toc do — mac dinh BAT voi he so an toan nho */
  b->spd_f       = 0.0f;
  b->spd_int     = 0.0f;
  b->spd_kp      = BAL_DEFAULT_SPD_KP;
  b->spd_ki      = BAL_DEFAULT_SPD_KI;
  b->spd_max_deg = BAL_DEFAULT_SPD_MAX_DEG;
  b->spd_loop_en = true;    /* bat san voi he so an toan */

  /* Danh gia can bang */
  b->tune_ok       = 0;
  b->tune_ok_cnt   = 0;
  b->max_angle_abs = 0.0f;
  b->max_rate_abs  = 0.0f;
  b->max_spd_abs   = 0.0f;
  b->ok_window     = 250u;
}

void Balance_SetWheelSpeed(Balance_t *b, float spd)
{
  /* Loc nhe de giam nhieu encoder: low-pass 0.7*cu + 0.3*moi */
  b->spd_f = 0.7f * b->spd_f + 0.3f * spd;
}

void Balance_Arm(Balance_t *b)
{
  b->integral = 0.0f;
  b->spd_int  = 0.0f;
  b->status   = BAL_OK;
  b->armed    = true;
}

void Balance_Disarm(Balance_t *b)
{
  b->armed    = false;
  b->integral = 0.0f;
  b->spd_int  = 0.0f;
  if (b->status == BAL_OK) b->status = BAL_FAULT_NOT_ARMED;
}

void Balance_Reset(Balance_t *b)
{
  b->integral    = 0.0f;
  b->spd_int     = 0.0f;
  b->control_out = 0.0f;
}

Balance_Status_t Balance_Update(Balance_t *b, MPU6050_Data_t *imu, Motor_Handle_t *mot)
{
  /* ---- Kiem tra an toan truoc ---- */
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

  /* ---- Vong ngoai: toc do banh -> target_angle thich ung ----
   * Khi banh chay nhanh (xe troi), dieu chinh target_angle nguoc lai de
   * than xe nghieng nguoc chieu chuyen dong -> tu ham, giu vi tri.
   * Gioi han spd_max_deg de vong nay chi lam viec phu, khong dao dong. */
  float target = b->target_angle;
  if (b->spd_loop_en) {
    b->spd_int += b->spd_f * 0.004f;
    /* gioi han tich phan */
    float int_lim = 1000.0f;
    if (b->spd_int >  int_lim) b->spd_int =  int_lim;
    if (b->spd_int < -int_lim) b->spd_int = -int_lim;

    float spd_adj = b->spd_kp * b->spd_f + b->spd_ki * b->spd_int;
    if (spd_adj >  b->spd_max_deg) spd_adj =  b->spd_max_deg;
    if (spd_adj < -b->spd_max_deg) spd_adj = -b->spd_max_deg;

    /* Khi banh chay theo chieu tien (xe troi), nghieng target nguoc lai
     * de than xe nga ve phia truoc -> banh quay lai de ham.
     * Dau can test: neu troi -> doi dau spd_kp. */
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

  /* ---- Danh gia chat luong can bang ---- */
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
