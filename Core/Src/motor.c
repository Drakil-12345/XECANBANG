/**
  ******************************************************************************
  * @file    motor.c
  * @brief   Driver TB6612FNG. Xem motor.h.
  ******************************************************************************
  */
#include "motor.h"

/* Chon hanh vi khi value=0:
 *   TB6612 khi IN1=IN2=LOW -> coast (tru do). IN1=IN2=HIGH -> short brake.
 *   Dung short brake khi value=0 giup chong truot, nhung cho viec can bang
 *   ta viet "0 = khong tao luc keo". Dung STOP = brake (IN1=IN2=HIGH) la
 *   phu hop hon voi can bang vi brake manh hon coast. Chon brake o Stop(),
 *   coast cung duoc; o day dung SHORT BRAKE cho dut khoat.
 *   -> khi value=0: IN1=HIGH, IN2=HIGH (short brake), duty=0.             */

static void set_dir(GPIO_TypeDef *p1, uint16_t pin1,
                    GPIO_TypeDef *p2, uint16_t pin2, int8_t dir)
{
  if (dir > 0) {         /* chieu A: IN1=1, IN2=0 */
    HAL_GPIO_WritePin(p1, pin1, GPIO_PIN_SET);
    HAL_GPIO_WritePin(p2, pin2, GPIO_PIN_RESET);
  } else if (dir < 0) {  /* chieu B: IN1=0, IN2=1 */
    HAL_GPIO_WritePin(p1, pin1, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(p2, pin2, GPIO_PIN_SET);
  } else {               /* short brake: IN1=1, IN2=1 */
    HAL_GPIO_WritePin(p1, pin1, GPIO_PIN_SET);
    HAL_GPIO_WritePin(p2, pin2, GPIO_PIN_SET);
  }
}

void Motor_Init(Motor_Handle_t *m, TIM_HandleTypeDef *htim9)
{
  m->htim_pwm     = htim9;
  m->ch_left      = TIM_CHANNEL_1;
  m->ch_right     = TIM_CHANNEL_2;
  m->enabled      = false;
  m->invert_left  = false;
  m->invert_right = false;
  m->pwm_max      = MOTOR_PWM_ABS_MAX;   /* nguoi dung nen giam khi test dau */
  m->cmd_left     = 0;
  m->cmd_right    = 0;
  m->trim_left    = 0;
  m->trim_right   = 0;

  /* An toan: STBY LOW, chieu brake, duty 0 */
  HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_SET);

  /* Bat PWM (duty 0) */
  __HAL_TIM_SET_COMPARE(m->htim_pwm, m->ch_left,  0);
  __HAL_TIM_SET_COMPARE(m->htim_pwm, m->ch_right, 0);
  HAL_TIM_PWM_Start(m->htim_pwm, m->ch_left);
  HAL_TIM_PWM_Start(m->htim_pwm, m->ch_right);
}

void Motor_Enable(Motor_Handle_t *m)
{
  /* Giu duty 0 khi mo STBY */
  __HAL_TIM_SET_COMPARE(m->htim_pwm, m->ch_left,  0);
  __HAL_TIM_SET_COMPARE(m->htim_pwm, m->ch_right, 0);
  HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_SET);
  m->enabled = true;
}

void Motor_Disable(Motor_Handle_t *m)
{
  __HAL_TIM_SET_COMPARE(m->htim_pwm, m->ch_left,  0);
  __HAL_TIM_SET_COMPARE(m->htim_pwm, m->ch_right, 0);
  HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_RESET);
  m->enabled = false;
  m->cmd_left = 0;
  m->cmd_right = 0;
}

void Motor_Stop(Motor_Handle_t *m)
{
  /* short brake */
  set_dir(AIN1_GPIO_Port, AIN1_Pin, AIN2_GPIO_Port, AIN2_Pin, 0);
  set_dir(BIN1_GPIO_Port, BIN1_Pin, BIN2_GPIO_Port, BIN2_Pin, 0);
  __HAL_TIM_SET_COMPARE(m->htim_pwm, m->ch_left,  0);
  __HAL_TIM_SET_COMPARE(m->htim_pwm, m->ch_right, 0);
  m->cmd_left = 0;
  m->cmd_right = 0;
}

/* Ham loi chung: ap dung invert + gioi han + xuat GPIO/PWM cho 1 kenh */
static void apply(Motor_Handle_t *m,
                  bool invert,
                  int16_t trim,
                  GPIO_TypeDef *p_in1, uint16_t pin_in1,
                  GPIO_TypeDef *p_in2, uint16_t pin_in2,
                  uint32_t channel,
                  int16_t value,
                  int16_t *cmd_store)
{
  if (!m->enabled) {
    /* Neu chua enable: giu brake, duty 0 */
    set_dir(p_in1, pin_in1, p_in2, pin_in2, 0);
    __HAL_TIM_SET_COMPARE(m->htim_pwm, channel, 0);
    *cmd_store = 0;
    return;
  }

  /* Dao chieu logic */
  int32_t v = value;
  if (invert) v = -v;

  /* Trim bu luc: khi lenh khac 0, cong them trim theo dau de bu motor yeu hon */
  if (v > 0)       v += trim;
  else if (v < 0)  v -= trim;

  /* Gioi han */
  if (v >  m->pwm_max) v =  m->pwm_max;
  if (v < -m->pwm_max) v = -m->pwm_max;

  int8_t dir;
  uint16_t duty;
  if (v > 0)      { dir = +1; duty = (uint16_t)v; }
  else if (v < 0) { dir = -1; duty = (uint16_t)(-v); }
  else            { dir =  0; duty = 0; }

  set_dir(p_in1, pin_in1, p_in2, pin_in2, dir);
  __HAL_TIM_SET_COMPARE(m->htim_pwm, channel, duty);
  *cmd_store = (int16_t)v;
}

void Motor_SetLeft(Motor_Handle_t *m, int16_t value)
{
  apply(m, m->invert_left, m->trim_left,
        AIN1_GPIO_Port, AIN1_Pin, AIN2_GPIO_Port, AIN2_Pin,
        m->ch_left, value, &m->cmd_left);
}

void Motor_SetRight(Motor_Handle_t *m, int16_t value)
{
  apply(m, m->invert_right, m->trim_right,
        BIN1_GPIO_Port, BIN1_Pin, BIN2_GPIO_Port, BIN2_Pin,
        m->ch_right, value, &m->cmd_right);
}

void Motor_SetBoth(Motor_Handle_t *m, int16_t left, int16_t right)
{
  Motor_SetLeft(m, left);
  Motor_SetRight(m, right);
}

void Motor_SetInvert(Motor_Handle_t *m, bool invert_left, bool invert_right)
{
  m->invert_left  = invert_left;
  m->invert_right = invert_right;
}

void Motor_SetPwmLimit(Motor_Handle_t *m, uint16_t pwm_max)
{
  if (pwm_max > MOTOR_PWM_ABS_MAX) pwm_max = MOTOR_PWM_ABS_MAX;
  m->pwm_max = pwm_max;
  /* Neu lenh hien tai vuot nguong moi, cat lai ngay */
  if (m->cmd_left  >  (int16_t)pwm_max) Motor_SetLeft(m,  pwm_max);
  if (m->cmd_left  < -(int16_t)pwm_max) Motor_SetLeft(m, -pwm_max);
  if (m->cmd_right >  (int16_t)pwm_max) Motor_SetRight(m,  pwm_max);
  if (m->cmd_right < -(int16_t)pwm_max) Motor_SetRight(m, -pwm_max);
}

void Motor_SetTrim(Motor_Handle_t *m, int16_t trim_left, int16_t trim_right)
{
  m->trim_left  = trim_left;
  m->trim_right = trim_right;
}
