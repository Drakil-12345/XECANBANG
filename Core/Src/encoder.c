#include "encoder.h"
#include <stddef.h>
#include <string.h>

#define ENCODER_SPEED_FILTER_ALPHA 0.30f

void Encoder_Init(Encoder_Handle_t *e, TIM_HandleTypeDef *htim, bool invert)
{
    if (e == NULL) return;

    memset(e, 0, sizeof(*e));
    e->htim = htim;
    e->invert = invert;

    if (htim == NULL) return;

    if (HAL_TIM_Encoder_Start(htim, TIM_CHANNEL_ALL) == HAL_OK)
    {
        e->started = true;
        e->previous_count = __HAL_TIM_GET_COUNTER(htim);
    }
}

void Encoder_Update(Encoder_Handle_t *e, float dt_s)
{
    uint32_t current_count;
    int32_t delta;

    if ((e == NULL) || (e->htim == NULL) || !e->started ||
        (dt_s < 0.001f) || (dt_s > 0.020f))
    {
        if (e != NULL)
        {
            e->delta_counts = 0;
            e->speed_raw_cps = 0.0f;
        }
        return;
    }

    current_count = __HAL_TIM_GET_COUNTER(e->htim);

    if (e->htim->Init.Period > 0xFFFFU)
    {
        /* 32-bit timer (TIM2): signed subtraction naturally handles wraparound. */
        delta = (int32_t)(current_count - e->previous_count);
    }
    else
    {
        /* 16-bit timer (TIM3): signed 16-bit subtraction handles wraparound. */
        delta = (int16_t)((uint16_t)current_count - (uint16_t)e->previous_count);
    }

    e->previous_count = current_count;
    if (e->invert) delta = -delta;

    e->delta_counts = delta;
    e->count_total += delta;
    e->speed_raw_cps = (float)delta / dt_s;
    e->speed_cps += ENCODER_SPEED_FILTER_ALPHA *
                    (e->speed_raw_cps - e->speed_cps);
}

void Encoder_Reset(Encoder_Handle_t *e)
{
    if (e == NULL || e->htim == NULL) return;
    e->previous_count = __HAL_TIM_GET_COUNTER(e->htim);
    e->delta_counts = 0;
    e->count_total = 0;
    e->speed_raw_cps = 0.0f;
    e->speed_cps = 0.0f;
}
