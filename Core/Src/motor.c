#include "motor.h"
#include <stddef.h>

/* TB6612: IN1=IN2=1 is short brake; STBY=0 disables the driver. */
static void set_bridge(GPIO_TypeDef *port1, uint16_t pin1,
                       GPIO_TypeDef *port2, uint16_t pin2,
                       int8_t direction)
{
    if (direction > 0)
    {
        port1->BSRR = pin1;
        port2->BSRR = (uint32_t)pin2 << 16U;
    }
    else if (direction < 0)
    {
        port1->BSRR = (uint32_t)pin1 << 16U;
        port2->BSRR = pin2;
    }
    else
    {
        /* Short brake. PWM is set to zero by the caller. */
        port1->BSRR = pin1;
        port2->BSRR = pin2;
    }
}

static void apply_one(Motor_Handle_t *m,
                      int16_t command,
                      int16_t trim,
                      uint16_t min_pwm,
                      bool invert,
                      GPIO_TypeDef *port1, uint16_t pin1,
                      GPIO_TypeDef *port2, uint16_t pin2,
                      uint32_t channel,
                      int16_t *logical_store,
                      int16_t *applied_store)
{
    int32_t logical;
    int32_t electrical;
    int32_t magnitude;
    int8_t direction;
    uint16_t duty;

    (void)channel; /* Also explicit for HAL macro implementations that inline it. */
    if ((m == NULL) || (m->htim_pwm == NULL)) return;

    if (!m->enabled || (command == 0))
    {
        set_bridge(port1, pin1, port2, pin2, 0);
        __HAL_TIM_SET_COMPARE(m->htim_pwm, channel, 0U);
        *logical_store = 0;
        *applied_store = 0;
        return;
    }

    /* Trim is a signed command offset, not a fixed amount added to |PWM|. */
    logical = (int32_t)command + (int32_t)trim;
    if (logical > (int32_t)m->pwm_max) logical = m->pwm_max;
    if (logical < -(int32_t)m->pwm_max) logical = -(int32_t)m->pwm_max;

    electrical = invert ? -logical : logical;
    if (electrical == 0)
    {
        set_bridge(port1, pin1, port2, pin2, 0);
        __HAL_TIM_SET_COMPARE(m->htim_pwm, channel, 0U);
        *logical_store = (int16_t)logical;
        *applied_store = 0;
        return;
    }

    direction = (electrical > 0) ? 1 : -1;
    magnitude = (electrical > 0) ? electrical : -electrical;

    /* Minimum PWM is a floor to overcome gearbox friction, NOT a replacement
       for the controller output.  The old code forced every non-zero command
       to exactly min_pwm, which turned the PD controller into bang-bang and
       made balancing impossible. */
    if ((min_pwm > 0U) && (magnitude < (int32_t)min_pwm))
    {
        magnitude = (int32_t)min_pwm;
    }

    if (magnitude > (int32_t)m->pwm_max) magnitude = m->pwm_max;
    duty = (uint16_t)magnitude;

    set_bridge(port1, pin1, port2, pin2, direction);
    __HAL_TIM_SET_COMPARE(m->htim_pwm, channel, duty);

    *logical_store = (int16_t)logical;
    *applied_store = (int16_t)(direction * (int32_t)duty);
}

void Motor_Init(Motor_Handle_t *m, TIM_HandleTypeDef *htim_pwm)
{
    if (m == NULL) return;

    m->htim_pwm = htim_pwm;
    m->ch_left = TIM_CHANNEL_1;
    m->ch_right = TIM_CHANNEL_2;
    m->pwm_max = MOTOR_PWM_ABS_MAX;
    m->enabled = false;
    m->invert_left = false;
    m->invert_right = false;
    m->trim_left = 0;
    m->trim_right = 0;
    m->min_pwm_left = 0U;
    m->min_pwm_right = 0U;
    m->cmd_left = 0;
    m->cmd_right = 0;
    m->applied_left = 0;
    m->applied_right = 0;

    HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_RESET);
    set_bridge(AIN1_GPIO_Port, AIN1_Pin, AIN2_GPIO_Port, AIN2_Pin, 0);
    set_bridge(BIN1_GPIO_Port, BIN1_Pin, BIN2_GPIO_Port, BIN2_Pin, 0);

    __HAL_TIM_SET_COMPARE(m->htim_pwm, m->ch_left, 0U);
    __HAL_TIM_SET_COMPARE(m->htim_pwm, m->ch_right, 0U);
    (void)HAL_TIM_PWM_Start(m->htim_pwm, m->ch_left);
    (void)HAL_TIM_PWM_Start(m->htim_pwm, m->ch_right);
}

void Motor_Enable(Motor_Handle_t *m)
{
    if (m == NULL || m->htim_pwm == NULL) return;

    __HAL_TIM_SET_COMPARE(m->htim_pwm, m->ch_left, 0U);
    __HAL_TIM_SET_COMPARE(m->htim_pwm, m->ch_right, 0U);
    HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_SET);
    m->enabled = true;
    m->cmd_left = 0;
    m->cmd_right = 0;
    m->applied_left = 0;
    m->applied_right = 0;
}

void Motor_Stop(Motor_Handle_t *m)
{
    if (m == NULL || m->htim_pwm == NULL) return;

    set_bridge(AIN1_GPIO_Port, AIN1_Pin, AIN2_GPIO_Port, AIN2_Pin, 0);
    set_bridge(BIN1_GPIO_Port, BIN1_Pin, BIN2_GPIO_Port, BIN2_Pin, 0);
    __HAL_TIM_SET_COMPARE(m->htim_pwm, m->ch_left, 0U);
    __HAL_TIM_SET_COMPARE(m->htim_pwm, m->ch_right, 0U);
    m->cmd_left = 0;
    m->cmd_right = 0;
    m->applied_left = 0;
    m->applied_right = 0;
}

void Motor_Disable(Motor_Handle_t *m)
{
    if (m == NULL || m->htim_pwm == NULL) return;

    Motor_Stop(m);
    HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_RESET);
    m->enabled = false;
}

void Motor_SetLeft(Motor_Handle_t *m, int16_t command)
{
    if (m == NULL) return;
    apply_one(m, command, m->trim_left, m->min_pwm_left, m->invert_left,
              AIN1_GPIO_Port, AIN1_Pin, AIN2_GPIO_Port, AIN2_Pin,
              m->ch_left, &m->cmd_left, &m->applied_left);
}

void Motor_SetRight(Motor_Handle_t *m, int16_t command)
{
    if (m == NULL) return;
    apply_one(m, command, m->trim_right, m->min_pwm_right, m->invert_right,
              BIN1_GPIO_Port, BIN1_Pin, BIN2_GPIO_Port, BIN2_Pin,
              m->ch_right, &m->cmd_right, &m->applied_right);
}

void Motor_SetBoth(Motor_Handle_t *m, int16_t left_command, int16_t right_command)
{
    Motor_SetLeft(m, left_command);
    Motor_SetRight(m, right_command);
}

void Motor_SetInvert(Motor_Handle_t *m, bool invert_left, bool invert_right)
{
    if (m == NULL) return;
    m->invert_left = invert_left;
    m->invert_right = invert_right;
}

void Motor_SetPwmLimit(Motor_Handle_t *m, uint16_t pwm_max)
{
    if (m == NULL) return;
    if (pwm_max > MOTOR_PWM_ABS_MAX) pwm_max = MOTOR_PWM_ABS_MAX;
    m->pwm_max = pwm_max;
}

void Motor_SetTrim(Motor_Handle_t *m, int16_t trim_left, int16_t trim_right)
{
    if (m == NULL) return;
    m->trim_left = trim_left;
    m->trim_right = trim_right;
}

void Motor_SetMinPwm(Motor_Handle_t *m, uint16_t min_left, uint16_t min_right)
{
    if (m == NULL) return;
    if (min_left > m->pwm_max) min_left = m->pwm_max;
    if (min_right > m->pwm_max) min_right = m->pwm_max;
    m->min_pwm_left = min_left;
    m->min_pwm_right = min_right;
}
