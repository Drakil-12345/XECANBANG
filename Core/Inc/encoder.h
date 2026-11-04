/**
  ******************************************************************************
  * @file    encoder.h
  * @brief   Doc encoder quadrature qua TIM2 (phai) va TIM3 (trai).
  *
  *   Encoder TRAI : TIM3, CH1=PC6, CH2=PC7, 16-bit counter (Period=65535)
  *   Encoder PHAI : TIM2, CH1=PA5, CH2=PA1, 32-bit counter (Period=0xFFFFFFFF)
  *
  * Day la cau hinh da dong bo voi so do phan cung trong XECANBANG.ioc.
  * CubeMX se sinh MSP tuong ung: TIM3 dung GPIOC, TIM2 dung GPIOA.
  * Neu thay doi day noi sau nay, phai cap nhat ca .ioc va MSP.
 *
  * Dau encoder duoc chuan hoa qua Encoder_SetInvert() sao cho:
  *   count > 0 khi banh quay theo chieu TIEN da chuan hoa.
  ******************************************************************************
  */
#ifndef __ENCODER_H
#define __ENCODER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

typedef struct {
  TIM_HandleTypeDef *htim;   /* TIM3 (trai) hoac TIM2 (phai) */
  bool    is_32bit;          /* true = TIM2 (32-bit), false = TIM3 (16-bit) */
  bool    invert;            /* dao dau dem */
  int64_t count_total;       /* tong so xung tich luy (co dau) */
  int32_t count_prev;        /* gia tri counter lan doc truoc (raw) */
  int32_t delta;             /* so xung trong chu ky vua doc (co dau) */
  float   speed_cps;         /* counts/second (neu da goi Update voi dt) */
} Encoder_Handle_t;

/* ===================== API ===================== */

/**
 * @brief Khoi tao encoder: start TIM encoder mode, reset bien dem.
 * @param e       handle encoder
 * @param htim    TIM3 (trai) hoac TIM2 (phai)
 * @param is_32   true neu TIM2 (32-bit)
 */
void Encoder_Init(Encoder_Handle_t *e, TIM_HandleTypeDef *htim, bool is_32);

/**
 * @brief Doc counter hien tai, cap nhat delta & count_total.
 *        Goi dinh ky (trong chu ky dieu khien).
 * @param e   handle
 * @param dt  chu ky goi (giay) de tinh speed_cps; truyen 0 neu khong can toc do.
 */
void Encoder_Update(Encoder_Handle_t *e, float dt);

/**
 * @brief Lay so xung tich luy co dau (da chuan hoa dau).
 */
int64_t Encoder_GetCount(const Encoder_Handle_t *e);

/**
 * @brief Lay delta xung trong chu ky vua doc (co dau).
 */
int32_t Encoder_GetDelta(const Encoder_Handle_t *e);

/**
 * @brief Reset bo dem tich luy ve 0.
 */
void Encoder_Reset(Encoder_Handle_t *e);

/**
 * @brief Dao dau encoder (sau khi xac minh chieu quay thuc te).
 */
void Encoder_SetInvert(Encoder_Handle_t *e, bool invert);

#ifdef __cplusplus
}
#endif

#endif /* __ENCODER_H */
