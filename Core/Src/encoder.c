/**
  ******************************************************************************
  * @file    encoder.c
  * @brief   Doc encoder TIM3 (trai, 16-bit) va TIM2 (phai, 32-bit). Xem encoder.h.
  ******************************************************************************
  */
#include "encoder.h"

void Encoder_Init(Encoder_Handle_t *e, TIM_HandleTypeDef *htim, bool is_32)
{
  e->htim        = htim;
  e->is_32bit    = is_32;
  e->invert      = false;
  e->count_total = 0;
  e->count_prev  = 0;
  e->delta       = 0;
  e->speed_cps   = 0.0f;

  /* Reset counter ve giua de tranh tran ngay khi bat dau (chi can cho 16-bit) */
  __HAL_TIM_SET_COUNTER(htim, 0);
  HAL_TIM_Encoder_Start(htim, TIM_CHANNEL_ALL);

  e->count_prev = (int32_t)__HAL_TIM_GET_COUNTER(htim);
}

/* Doc counter voi kieu du lieu phu hop, xu ly tran/wrap dung cach */
static int32_t read_counter(Encoder_Handle_t *e)
{
  if (e->is_32bit) {
    return (int32_t)__HAL_TIM_GET_COUNTER(e->htim);   /* 32-bit: cast truc tiep */
  } else {
    return (int32_t)(int16_t)__HAL_TIM_GET_COUNTER(e->htim); /* 16-bit: sign-extend */
  }
}

void Encoder_Update(Encoder_Handle_t *e, float dt)
{
  int32_t now = read_counter(e);
  int32_t diff;

  if (e->is_32bit) {
    diff = now - e->count_prev;   /* 32-bit: tru truc tiep, wrap tu xu ly */
  } else {
    /* 16-bit: wrap nam trong int16; tru roi sign-extend */
    diff = (int16_t)(now - e->count_prev);
  }

  e->count_prev = now;

  if (e->invert) diff = -diff;

  e->delta        = diff;
  e->count_total += diff;

  if (dt > 0.0f) {
    e->speed_cps = (float)diff / dt;   /* counts per second */
  }
}

int64_t Encoder_GetCount(const Encoder_Handle_t *e)
{
  return e->count_total;
}

int32_t Encoder_GetDelta(const Encoder_Handle_t *e)
{
  return e->delta;
}

void Encoder_Reset(Encoder_Handle_t *e)
{
  e->count_total = 0;
  e->delta       = 0;
}

void Encoder_SetInvert(Encoder_Handle_t *e, bool invert)
{
  e->invert = invert;
}
