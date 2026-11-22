#include "balance_control.h"
#include <stddef.h>
#include <math.h>

#define BAL_DEFAULT_KP                120.0f
#define BAL_DEFAULT_KD                 10.0f
#define BAL_DEFAULT_KI                  0.0f
#define BAL_DEFAULT_ANGLE_LIMIT        50.0f
#define BAL_DEFAULT_RATE_LIMIT        700.0f
#define BAL_DEFAULT_PWM_LIMIT        5500
#define BAL_DEFAULT_INTEGRAL_LIMIT     100.0f
#define BAL_DEFAULT_SPEED_KP             0.0f
#define BAL_DEFAULT_SPEED_KI             0.0f
#define BAL_DEFAULT_SPEED_MAX_DEG        5.0f
#define BAL_DEFAULT_SPEED_ALPHA         0.25f

static float clampf_local(float value, float min_value, float max_value)
{
    if (value > max_value) return max_value;
    if (value < min_value) return min_value;
    return value;
}

void Balance_Init(Balance_t *b)
{
    if (b == NULL) return;

    b->kp = BAL_DEFAULT_KP;
    b->kd = BAL_DEFAULT_KD;
    b->ki = BAL_DEFAULT_KI;
    b->integral = 0.0f;
    b->integral_limit = BAL_DEFAULT_INTEGRAL_LIMIT;

    /* These signs follow the convention documented in README.md. */
    b->angle_sign = 1.0f;
    b->gyro_sign = 1.0f;
    b->output_sign = 1.0f;

    b->target_angle_deg = 0.0f;
    b->angle_limit_deg = BAL_DEFAULT_ANGLE_LIMIT;
    b->rate_limit_dps = BAL_DEFAULT_RATE_LIMIT;
    b->pwm_limit = BAL_DEFAULT_PWM_LIMIT;

    b->speed_loop_enabled = false;
    b->speed_kp = BAL_DEFAULT_SPEED_KP;
    b->speed_ki = BAL_DEFAULT_SPEED_KI;
    b->speed_integral = 0.0f;
    b->speed_integral_limit = 10000.0f;
    b->speed_target_cps = 0.0f;
    b->speed_filtered_cps = 0.0f;
    b->speed_filter_alpha = BAL_DEFAULT_SPEED_ALPHA;
    b->speed_angle_limit_deg = BAL_DEFAULT_SPEED_MAX_DEG;

    b->angle_deg = 0.0f;
    b->rate_dps = 0.0f;
    b->angle_error_deg = 0.0f;
    b->effective_target_deg = 0.0f;
    b->p_term = 0.0f;
    b->d_term = 0.0f;
    b->i_term = 0.0f;
    b->output_unclamped = 0.0f;
    b->output_pwm = 0.0f;
    b->update_count = 0U;
    b->stable_count = 0U;
    b->stable = 0U;
    b->status = BAL_FAULT_NOT_ARMED;
    b->armed = false;
}

void Balance_Reset(Balance_t *b)
{
    if (b == NULL) return;

    b->integral = 0.0f;
    b->speed_integral = 0.0f;
    b->speed_filtered_cps = 0.0f;
    b->output_unclamped = 0.0f;
    b->output_pwm = 0.0f;
    b->stable_count = 0U;
    b->stable = 0U;
}

void Balance_Arm(Balance_t *b)
{
    if (b == NULL) return;

    Balance_Reset(b);
    b->armed = true;
    b->status = BAL_OK;
}

void Balance_Disarm(Balance_t *b)
{
    if (b == NULL) return;

    b->armed = false;
    Balance_Reset(b);
    if (b->status == BAL_OK)
    {
        b->status = BAL_FAULT_NOT_ARMED;
    }
}

void Balance_SetWheelSpeed(Balance_t *b, float speed_cps)
{
    float alpha;

    if (b == NULL || !isfinite(speed_cps)) return;

    alpha = clampf_local(b->speed_filter_alpha, 0.0f, 1.0f);
    b->speed_filtered_cps += alpha * (speed_cps - b->speed_filtered_cps);
}

Balance_Status_t Balance_Update(Balance_t *b,
                                const MPU6050_Data_t *imu,
                                Motor_Handle_t *motor,
                                float dt_s)
{
    float angle;
    float rate;
    float target;
    float error;
    float speed_adjust = 0.0f;
    float output;
    float p;
    float d;
    float i;
    int32_t pwm_command;
    int32_t pwm_limit;

    if (b == NULL || imu == NULL || motor == NULL)
    {
        return BAL_FAULT_NUMERIC;
    }

    if (!b->armed)
    {
        b->status = BAL_FAULT_NOT_ARMED;
        Motor_Stop(motor);
        return b->status;
    }

    if (!isfinite(dt_s) || dt_s <= 0.0f)
    {
        return b->status;
    }

    angle = imu->angle * b->angle_sign;
    rate = imu->gyro_x_dps * b->gyro_sign;

    if (!isfinite(angle) || !isfinite(rate))
    {
        return b->status;
    }

    b->angle_deg = angle;
    b->rate_dps = rate;

    /* Only safety cutoff kept: shut off motors when tilt exceeds angle_limit_deg (50 deg). */
    if (fabsf(angle) > b->angle_limit_deg)
    {
        b->status = BAL_FAULT_ANGLE;
        b->output_pwm = 0.0f;
        b->output_unclamped = 0.0f;
        b->integral = 0.0f;
        b->speed_integral = 0.0f;
        b->stable_count = 0U;
        b->stable = 0U;
        Motor_Disable(motor);
        return BAL_FAULT_ANGLE;
    }

    if (!motor->enabled)
    {
        Motor_Enable(motor);
    }

    target = b->target_angle_deg;

    /* Outer speed loop is optional and OFF by default.
       Encoders are normalized so positive speed_filtered_cps means rolling
       forward, and angle_sign defines which angle sign is a forward lean.
       To brake forward motion, target angle must shift backward
       (-angle_sign * speed_adjust), independent of motor wiring output_sign. */
    if (b->speed_loop_enabled)
    {
        float speed_error = b->speed_filtered_cps - b->speed_target_cps;
        b->speed_integral += speed_error * dt_s;
        b->speed_integral = clampf_local(b->speed_integral,
                                         -b->speed_integral_limit,
                                          b->speed_integral_limit);
        speed_adjust = b->speed_kp * speed_error + b->speed_ki * b->speed_integral;
        speed_adjust = clampf_local(speed_adjust,
                                    -b->speed_angle_limit_deg,
                                     b->speed_angle_limit_deg);
        target -= b->angle_sign * speed_adjust;
    }
    else
    {
        b->speed_integral = 0.0f;
    }

    /* Convention: angle and rate share the same sign (guaranteed by the
       MPU6050 driver).  Positive output drives the wheels toward the lean.
       Therefore the P and D terms must have the same sign.  Ki is optional. */
    error = angle - target;
    b->angle_error_deg = error;
    b->effective_target_deg = target;

    p = b->kp * error;
    d = b->kd * rate;

    /* Integral is retained for later tuning; default Ki is exactly zero. */
    if (b->ki != 0.0f)
    {
        b->integral += error * dt_s;
        b->integral = clampf_local(b->integral,
                                   -b->integral_limit,
                                    b->integral_limit);
    }
    else
    {
        b->integral = 0.0f;
    }
    i = b->ki * b->integral;

    output = b->output_sign * (p + d + i);
    if (!isfinite(output))
    {
        Motor_Stop(motor);
        return BAL_FAULT_NUMERIC;
    }

    b->p_term = p;
    b->d_term = d;
    b->i_term = i;
    b->output_unclamped = output;

    pwm_limit = b->pwm_limit;
    if (pwm_limit < 0) pwm_limit = 0;
    if (pwm_limit > motor->pwm_max) pwm_limit = motor->pwm_max;

    output = clampf_local(output, -(float)pwm_limit, (float)pwm_limit);
    b->output_pwm = output;

    pwm_command = (int32_t)((output >= 0.0f) ? (output + 0.5f) : (output - 0.5f));
    Motor_SetBoth(motor, (int16_t)pwm_command, (int16_t)pwm_command);

    b->update_count++;
    if ((fabsf(angle - target) < 2.0f) && (fabsf(rate) < 12.0f) &&
        (fabsf(b->speed_filtered_cps) < 80.0f))
    {
        if (b->stable_count < 1000U) b->stable_count++;
    }
    else
    {
        b->stable_count = 0U;
    }
    b->stable = (b->stable_count >= 100U) ? 1U : 0U;

    b->status = BAL_OK;
    return b->status;
}
