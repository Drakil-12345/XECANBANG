/**
  ******************************************************************************
  * @file    motor.h
  * @brief   Driver TB6612FNG dieu khien 2 dong co DC (trai = kenh A, phai = kenh B).
  *
  * Ket noi (theo so do):
  *   STBY  = PC4
  *   PWMA  = PE5  (TIM9_CH1)   AIN1 = PE0   AIN2 = PE1   -> Motor TRAI
  *   PWMB  = PE6  (TIM9_CH2)   BIN1 = PE2   BIN2 = PC1   -> Motor PHAI
  *
  * Quy uoc dau (chuan hoa o lop LOGIC, khong phu thuoc day noi):
  *   value > 0 : quay theo chieu TIEN (da chuan hoa)
  *   value < 0 : quay theo chieu LUI
  *   value = 0 : khong tao luc keo (short-brake trong cai dat hien tai)
  *
  * Hai dong co lap doi xung nen chieu quay VAT LY cho cung mot dau logic co
  * the NGUOC NHAU. Dung Motor_SetInvert() de chuan hoa SAU KHI kiem tra thuc te.
  *
  * AN TOAN:
  *   - STBY giu LOW cho toi khi Motor_Enable() duoc goi (sau khi IMU hop le + ARM).
  *   - PWM gioi han boi motor_pwm_max (khong bao gio vuot nguong an toan).
  *   - Kiem tra kha nang chiu dong cua TB6612 (1.2A lien tuc, 3.2A peak) truoc
  *     khi tang motor_pwm_max tren phan cung that.
  ******************************************************************************
  */
#ifndef __MOTOR_H
#define __MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* Gioi han PWM tuyet doi: gia tri nay <= TIM9 ARR (8399).
 * Dat ban dau thap de an toan khi chua xac minh dien ap/dong dong co.
 * motor_pwm_max (runtime) co the nho hon de gioi han them khi test.         */
#define MOTOR_PWM_ABS_MAX   8399

typedef struct {
  TIM_HandleTypeDef *htim_pwm;   /* TIM9 */
  uint32_t ch_left;              /* TIM_CHANNEL_1 (PWMA) */
  uint32_t ch_right;             /* TIM_CHANNEL_2 (PWMB) */

  bool enabled;                  /* STBY da bat chua */
  bool invert_left;              /* dao chieu logic motor trai  */
  bool invert_right;             /* dao chieu logic motor phai  */

  uint16_t pwm_max;              /* nguong PWM toi da cho phep (<= MOTOR_PWM_ABS_MAX) */
  int16_t  cmd_left;             /* lenh cuoi cung da gui (co dau, da gioi han) */
  int16_t  cmd_right;

  /* Trim bu luc cho tung motor (PWM cong them theo dau).
   * Vi motor hop so co ma sat khac nhau -> mot ben yeu hon. Khi lenh khac 0,
   * cong them trim_<side> vao magnitude de bu yeu. Dat bang thi nghiem.   */
  int16_t  trim_left;
  int16_t  trim_right;
} Motor_Handle_t;

/* ===================== API ===================== */

/**
 * @brief Khoi tao motor: khoi dong PWM TIM9 CH1/CH2, duty=0, STBY=LOW (an toan).
 */
void Motor_Init(Motor_Handle_t *m, TIM_HandleTypeDef *htim9);

/**
 * @brief Bat STBY (cho phep motor chay). Chi goi khi da san sang can bang.
 */
void Motor_Enable(Motor_Handle_t *m);

/**
 * @brief Tat STBY + duty=0 -> cat dong co (an toan, dung khi loi/disarm).
 */
void Motor_Disable(Motor_Handle_t *m);

/**
 * @brief Dung nhanh (short-brake): duty=0, chieu brake, khong tat STBY.
 *        Dung khi can dung motor nhung van muon kha nang hoat dong lai nhanh.
 */
void Motor_Stop(Motor_Handle_t *m);

/**
 * @brief Dat lenh motor TRAI (kenh A). value co dau, don vi = dem PWM (0..pwm_max).
 *        Dau da chuan hoa: + tien, - lui (sau khi invert dung).
 */
void Motor_SetLeft(Motor_Handle_t *m, int16_t value);

/**
 * @brief Dat lenh motor PHAI (kenh B).
 */
void Motor_SetRight(Motor_Handle_t *m, int16_t value);

/**
 * @brief Dat lenh ca hai motor cung luc (tien/lui cung dau = chuyen dong thang).
 */
void Motor_SetBoth(Motor_Handle_t *m, int16_t left, int16_t right);

/**
 * @brief Chon dao chieu logic cho tung motor (sau khi kiem tra chieu quay thuc te).
 * @param invert_left   true = dao chieu motor trai
 * @param invert_right  true = dao chieu motor phai
 * @note  Xac dinh bang thi nghiem: cho value>0, neu banh quay "lui" thi dat invert.
 */
void Motor_SetInvert(Motor_Handle_t *m, bool invert_left, bool invert_right);

/**
 * @brief Dat nguong PWM toi da (gioi han cong suat). <= MOTOR_PWM_ABS_MAX.
 *        Bat dau thap (vi du 2000-3000) khi chua xac minh dong co.
 */
void Motor_SetPwmLimit(Motor_Handle_t *m, uint16_t pwm_max);

/**
 * @brief Dat trim bu luc cho tung motor (PWM cong them khi lenh != 0).
 *        Dung khi mot ben yeu hon — trim_left > 0 bu them cho trai.
 */
void Motor_SetTrim(Motor_Handle_t *m, int16_t trim_left, int16_t trim_right);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_H */
