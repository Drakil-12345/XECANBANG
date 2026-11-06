/**
  ******************************************************************************
  * @file    balance_control.h
  * @brief   Bo dieu khien can bang PD cho goc nghieng (chu ky 4ms / 250Hz).
  *
  * Cau truc:
  *   error     = target_angle - measured_angle        (deg)
  *   control   = Kp*error - Kd*angular_velocity       (don vi PWM)
  *   out_left  = control
  *   out_right = control
  *
  *   (voi xe 2 banh song song, ca hai motor nhan cung 1 lenh de chuyen dong
  *    thang; hieu chinh sai so trai/phai bang trim neu can)
  *
  * !!! CANH BAO VE DAU (sign) !!!
  *   Dau cua `Kp` va `Kd` trong code nay la DAU THAM KHAO theo quy uoc:
  *     - angle > 0  = nghieng ve phia "truoc" (chieu tien)
  *     - control > 0 = motor keo banh ve phia "truoc" de dua banh duoi trong tam
  *   DAU THUC TE phu thuoc: chieu lap IMU, dau gyro, chieu quay motor.
  *   PHAI xac minh bang thi nghiem truoc khi ARM:
  *     1) Nghieng xe nhe ve truoc -> angle nen >0 (hoac doi dau angle_sign).
  *     2) Cho phep Kp nho, Kd=0 -> khi nghieng truoc, banh phai quay TIEN
  *        (dua banh ve phia trong tam). Neu quay nguoc -> doi dau (out_sign
  *        hoac Kp). Khi dang roi dung thi Kp/Kd moi tang dan.
  *   Khong xac minh dau = xe se TANG TOC NGA thay vi can bang.
  *
  * AN TOAN:
  *   - armed = false mac dinh; Balance_Arm() bat, Balance_Disarm() tat.
  *   - Neu |angle| > limit_deg -> tu dong disarm + cat motor.
  *   - Neu IMU mat ket noi / khong valid -> tu dong disarm + cat motor.
  *   - Sau khi fault, KHONG tu arm lai; phai goi Balance_Arm() lai co y thuc.
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

  float target_angle;     /* goc dich (deg), thuong = 0 sau hieu chinh */
  float angle_limit_deg;  /* nguong goc an toan (disarm neu vuot)    */
  float rate_limit_dps;   /* nguong van toc goc bat thuong (deg/s)   */
  int16_t pwm_limit;      /* gioi han PWM dau ra (<= motor pwm_max)  */
  float   deadband;       /* bu vung chet motor (PWM): lenh |u|>0 thi cong
                           * them deadband de vuot ma sat nhớt. Do bang
                           * thi nghiem: tang dan den khi banh bat dau quay. */

  float control_out;      /* dau ra tinh duoc (PWM, co dau, da gioi han) */
  float angle_now;        /* goc hien tai (deg)      */
  float rate_now;         /* van toc goc hien tai (deg/s)            */

  bool armed;             /* true = cho phep xuat motor              */
  Balance_Status_t status;
  uint32_t run_cycles;    /* dem so chu ky dieu khien da chay         */
  uint32_t missed_cycles; /* dem so lan xu ly cham / miss deadline    */
  uint32_t max_missed;    /* nguong overrun truoc khi fault timing     */

  /* Vong ngoai: toc do banh -> goc muc tieu (giup xe tu ham, khong trôi) */
  float spd_f;            /* toc do banh trung binh (counts/s, da loc)  */
  float spd_int;          /* tich phan toc do                          */
  float spd_kp;           /* he so P vong toc do (do / (counts/s))      */
  float spd_ki;           /* he so I vong toc do                        */
  float spd_max_deg;      /* goc lech toi da do vong toc do tao ra      */
  bool  spd_loop_en;      /* bat vong toc do khi da on dinh vong goc    */

  /* Danh gia chat luong can bang tu dong (khong can nguoi do) */
  uint8_t  tune_ok;       /* 1 = dang can bang on dinh trong cua so do  */
  uint8_t  tune_ok_cnt;   /* so chu ky lien tiep dat nguong can bang    */
  float    max_angle_abs; /* |angle| lon nhat trong cua so 1s gan nhat  */
  float    max_rate_abs;  /* |rate| lon nhat trong cua so 1s gan nhat   */
  float    max_spd_abs;   /* |spd_f| lon nhat trong cua so 1s gan nhat  */
  uint16_t ok_window;     /* do rong cua so danh gia (chu ky, ~1s)      */
} Balance_t;

/* ===================== API ===================== */

/**
 * @brief Khoi tao bo dieu khien voi cau hinh mac dinh AN TOAN.
 *        kp/kd bat dau = 0 (khong tac dong), armed=false.
 */
void Balance_Init(Balance_t *b);

/**
 * @brief ARM: cho phep xuat motor. Chi nen goi khi da kiem tra dau + san sang.
 *        Khong tu dong arm; goi tu debugger hoac dieu kien an toan trong code.
 */
void Balance_Arm(Balance_t *b);

/**
 * @brief DISARM: cat xuat motor ngay (an toan). Goi khi fault hoac muon dung.
 */
void Balance_Disarm(Balance_t *b);

/**
 * @brief Cap nhat mot chu ky dieu khien (goi o 250Hz tu vong lap chinh).
 *        Doc imu->angle + gyro, tinh control, xuat motor (neu armed).
 * @param b    bo dieu khien
 * @param imu  du lieu MPU6050 (da Update xong)
 * @param mot  handle motor
 * @retval trang thai sau chu ky nay.
 * @note  Neu fault -> tu dong Disarm + cat motor, tra ve ma loi.
 */
Balance_Status_t Balance_Update(Balance_t *b, MPU6050_Data_t *imu, Motor_Handle_t *mot);

/**
 * @brief Cap nhat toc do banh trung binh (tu encoder) cho vong ngoai.
 *        Goi tu main loop moi chu ky truoc Balance_Update.
 * @param b    bo dieu khien
 * @param spd  toc do trung binh (counts/s) cua 2 encoder
 */
void Balance_SetWheelSpeed(Balance_t *b, float spd);

/**
 * @brief Bao cho bo dieu khien biet co chu ky bi lo (overrun).
 *        Goi tu main loop khi ticks>1. Neu vuot nguong -> BAL_FAULT_TIMING.
 */
void Balance_NotifyMissed(Balance_t *b, uint32_t missed, Motor_Handle_t *mot);

/**
 * @brief Dat lai integral + trang thai sau khi hieu chinh / disarm.
 */
void Balance_Reset(Balance_t *b);

#ifdef __cplusplus
}
#endif

#endif /* __BALANCE_CONTROL_H */
