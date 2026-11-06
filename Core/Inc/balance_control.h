/**
  ******************************************************************************
  * @file    balance_control.h
  * @brief   Bo dieu khien can bang xe 2 banh — cascade day du.
  *
  * CAU TRUC CHUAN XE TU CAN BANG:
  *   VI TRI (encoder) ─┐
  *   TOC DO (encoder) ─┼─> target_angle (goc muc tieu thich ung)
  *   GOC (MPU6050)  ───┴─> PD: u = Kp*err - Kd*rate + Ki*integral -> PWM
  *
  * Y nghia: xe nghieng -> banh quay; banh lan di (vi tri/toc do lon) ->
  * target_angle dich nguoc -> xe tu ham, giu cho, khong troi.
  ******************************************************************************
  */
#ifndef __BALANCE_CONTROL_H
#define __BALANCE_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "mpu6050.h"
#include "motor.h"
#include "encoder.h"
#include <stdint.h>
#include <stdbool.h>

typedef enum {
  BAL_OK = 0,
  BAL_FAULT_IMU,        /* IMU loi / mat ket noi */
  BAL_FAULT_ANGLE,      /* goc vuot nguong an toan */
  BAL_FAULT_RATE,       /* van toc goc qua lon (bat thuong) */
  BAL_FAULT_NOT_ARMED,
  BAL_FAULT_TIMING      /* chu ky dieu khien bi lo nghiem trong */
} Balance_Status_t;

typedef struct {
  /* He so dieu khien (co the chinh qua debugger khi dang chay) */
  float kp;               /* P gain: PWM per degree */
  float kd;               /* D gain: PWM per (deg/s)  */
  float ki;               /* I gain (de 0 ban dau)    */
  float integral;         /* tich luy integral        */
  float integral_limit;   /* gioi han integral        */

  /* Dau/quy uoc dieu chinh khi test */
  float angle_sign;       /* +1 hoac -1: dau cua goc nghieng        */
  float gyro_sign;        /* +1 hoac -1: dau cua gyro truc can bang  */
  float out_sign;         /* +1 hoac -1: dau lenh motor              */

  float target_angle;     /* goc dich co so (deg), thuong = 0        */
  float angle_limit_deg;  /* nguong goc an toan (disarm neu vuot)    */
  float rate_limit_dps;   /* nguong van toc goc bat thuong (deg/s)   */
  int16_t pwm_limit;      /* gioi han PWM dau ra (<= motor pwm_max)  */
  float   deadband;       /* bu vung chet motor (PWM)                */

  float control_out;      /* dau ra tinh duoc (PWM, co dau, da gioi han) */
  float angle_now;        /* goc hien tai (deg)      */
  float rate_now;         /* van toc goc hien tai (deg/s)            */

  bool armed;             /* true = cho phep xuat motor              */
  Balance_Status_t status;
  uint32_t run_cycles;    /* dem so chu ky dieu khien da chay         */
  uint32_t missed_cycles; /* dem so lan xu ly cham / miss deadline    */
  uint32_t max_missed;    /* nguong overrun truoc khi fault timing     */

  /* Vong ngoai TOC DO banh */
  float   spd_f;            /* toc do banh trung binh (counts/s, da loc) */
  float   spd_int;          /* tich phan toc do                          */
  float   spd_kp;           /* he so P toc do (do / counts_s)            */
  float   spd_ki;           /* he so I toc do                            */
  float   spd_max_deg;      /* goc lech toi da do vong toc do            */
  bool    spd_loop_en;

  /* Vong ngoai VI TRI (encoder tich luy) */
  float   pos;              /* vi tri trung binh (counts)                */
  float   pos_kp;           /* he so P vi tri (do / count)               */
  float   pos_ki;           /* he so I vi tri                            */
  float   pos_int;          /* tich phan vi tri                          */
  float   pos_max_deg;      /* goc lech toi da do vong vi tri            */
  bool    pos_loop_en;

  /* Log real-time (xem trong debugger) */
  float   log_angle;
  float   log_rate;
  float   log_control;
  float   log_target;
  float   log_spd;
  float   log_error;
  uint8_t log_armed;
  uint8_t log_status;
  uint8_t log_tune_ok;

  /* Danh gia chat luong can bang */
  uint8_t  tune_ok;
  uint8_t  tune_ok_cnt;
  float    max_angle_abs;
  float    max_rate_abs;
  float    max_spd_abs;
  uint16_t ok_window;
} Balance_t;

void Balance_Init(Balance_t *b);
void Balance_Arm(Balance_t *b);
void Balance_Disarm(Balance_t *b);
void Balance_Reset(Balance_t *b);
Balance_Status_t Balance_Update(Balance_t *b, MPU6050_Data_t *imu, Motor_Handle_t *mot);
void Balance_SetWheelSpeed(Balance_t *b, float spd);
void Balance_SetPosition(Balance_t *b, float pos);
void Balance_NotifyMissed(Balance_t *b, uint32_t missed, Motor_Handle_t *mot);

#ifdef __cplusplus
}
#endif

#endif /* __BALANCE_CONTROL_H */
