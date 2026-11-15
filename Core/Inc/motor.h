#ifndef MOTOR_H
#define MOTOR_H

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

/* Must match TIM9 ARR + 1 in main.c. */
#ifndef MOTOR_PWM_ABS_MAX
#define MOTOR_PWM_ABS_MAX 8399U
#endif

typedef struct
{
    TIM_HandleTypeDef *htim_pwm;
    uint32_t ch_left;
    uint32_t ch_right;

    uint16_t pwm_max;
    bool enabled;
    bool invert_left;
    bool invert_right;

    /* Signed correction added to each logical wheel command. Keep at 0 initially. */
    int16_t trim_left;
    int16_t trim_right;

    /* Minimum duty once a non-zero command is requested. Keep at 0 until measured. */
    uint16_t min_pwm_left;
    uint16_t min_pwm_right;

    /* Debug: logical commands before electrical inversion, and actual signed outputs. */
    int16_t cmd_left;
    int16_t cmd_right;
    int16_t applied_left;
    int16_t applied_right;
} Motor_Handle_t;

void Motor_Init(Motor_Handle_t *motor, TIM_HandleTypeDef *htim_pwm);
void Motor_Enable(Motor_Handle_t *motor);
void Motor_Disable(Motor_Handle_t *motor);
void Motor_Stop(Motor_Handle_t *motor);
void Motor_SetLeft(Motor_Handle_t *motor, int16_t command);
void Motor_SetRight(Motor_Handle_t *motor, int16_t command);
void Motor_SetBoth(Motor_Handle_t *motor, int16_t left_command, int16_t right_command);
void Motor_SetInvert(Motor_Handle_t *motor, bool invert_left, bool invert_right);
void Motor_SetPwmLimit(Motor_Handle_t *motor, uint16_t pwm_max);
void Motor_SetTrim(Motor_Handle_t *motor, int16_t trim_left, int16_t trim_right);
void Motor_SetMinPwm(Motor_Handle_t *motor, uint16_t min_left, uint16_t min_right);

#endif /* MOTOR_H */
