#ifndef ENCODER_H
#define ENCODER_H

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    TIM_HandleTypeDef *htim;
    uint32_t previous_count;
    bool invert;
    bool started;
    int32_t delta_counts;
    int32_t count_total;   /* signed accumulated count, debug only */
    float speed_raw_cps;
    float speed_cps;
} Encoder_Handle_t;

/* invert=true when this timer counts negative while the chassis moves forward. */
void Encoder_Init(Encoder_Handle_t *encoder, TIM_HandleTypeDef *htim, bool invert);
void Encoder_Update(Encoder_Handle_t *encoder, float dt_s);
void Encoder_Reset(Encoder_Handle_t *encoder);

#endif /* ENCODER_H */
